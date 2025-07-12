#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray
from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
import tf2_ros
import tf2_geometry_msgs
import cv2
import numpy as np
from ultralytics import YOLO
import collections

class YOLOv8ToAprilTagBridge:
    def __init__(self):
        rospy.init_node('yolov8_to_apriltag_bridge')
        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        self.fx = self.fy = self.cx = self.cy = None
        self.depth_image = None

        self.min_confidence = 0.9
        self.valid_classes = ["stop sign"]
        self.max_depth = 2.0
        self.min_depth = 0.1

        self.history = collections.defaultdict(list)
        self.min_history_frames = 3
        self.position_threshold = 0.1

        self.known_landmarks = []  # (id, position)
        self.min_dist_between_landmarks = 0.5
        self.tag_id_counter = 0

        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.image_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)
        self.info_sub = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.camera_info_callback)

        self.tag_pub = rospy.Publisher("/tag_detections", AprilTagDetectionArray, queue_size=10)
        self.marker_pub = rospy.Publisher("/rtabmap/tag_markers", MarkerArray, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if self.depth_image.dtype == np.uint16:
            self.depth_image = self.depth_image.astype(np.float32) / 1000.0  # mm to meters

    def image_callback(self, msg):
        if self.fx is None or self.depth_image is None:
            return

        rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(rgb)[0]

        detections_msg = AprilTagDetectionArray()
        detections_msg.header.frame_id = "camera_rgb_optical_frame"
        detections_msg.header.stamp = msg.header.stamp

        marker_header = rospy.Header()
        marker_header.frame_id = "map"
        marker_header.stamp = msg.header.stamp

        markers = MarkerArray()

        for det in results.boxes:
            conf = float(det.conf.cpu().numpy())
            if conf < self.min_confidence:
                continue

            cls_id = int(det.cls.cpu().numpy())
            label = self.model.names[cls_id]
            if label not in self.valid_classes:
                continue

            x1, y1, x2, y2 = det.xyxy[0].cpu().numpy()
            x_c = int((x1 + x2) / 2)
            y_c = int((y1 + y2) / 2)

            patch = self.depth_image[max(y_c-2,0):y_c+3, max(x_c-2,0):x_c+3]
            patch = patch[~np.isnan(patch)]
            patch = patch[(patch > self.min_depth) & (patch < self.max_depth)]
            if len(patch) == 0:
                continue

            Z = np.median(patch)
            X = (x_c - self.cx) * Z / self.fx
            Y = (y_c - self.cy) * Z / self.fy

            pos_cam = np.array([X, Y, Z])

            self.history[label].append(pos_cam)
            if len(self.history[label]) > self.min_history_frames:
                self.history[label].pop(0)

            # Avoid duplicate landmarks near each other
            is_duplicate = False
            for _, existing_pos in self.known_landmarks:
                if np.linalg.norm(existing_pos - pos_cam) < self.min_dist_between_landmarks:
                    is_duplicate = True
                    break
            if is_duplicate:
                continue

            tag_id = self.tag_id_counter
            self.tag_id_counter += 1
            self.known_landmarks.append((tag_id, pos_cam))

            # Publish detection to RTAB-Map (in camera frame)
            detection = AprilTagDetection()
            detection.id = [tag_id]
            detection.size = [0.2]
            detection.pose.header = detections_msg.header
            detection.pose.header.stamp = msg.header.stamp
            detection.pose.pose.pose.position.x = X
            detection.pose.pose.pose.position.y = Y
            detection.pose.pose.pose.position.z = Z
            detection.pose.pose.pose.orientation.w = 1.0
            detections_msg.detections.append(detection)

            # Transform marker to map frame for RViz visualization
            try:
                point = tf2_geometry_msgs.PointStamped()
                point.header.frame_id = detections_msg.header.frame_id
                point.header.stamp = msg.header.stamp
                point.point.x = X
                point.point.y = Y
                point.point.z = Z

                self.tf_buffer.can_transform("map", point.header.frame_id, point.header.stamp, rospy.Duration(0.5))
                point_map = self.tf_buffer.transform(point, "map", rospy.Duration(0.5))

                marker = Marker()
                marker.header = marker_header
                marker.id = tag_id
                marker.ns = "yolo"
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD
                marker.pose.position = point_map.point
                marker.scale.z = 0.3
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.text = f"{label} ({conf:.2f})"
                markers.markers.append(marker)

            except Exception as e:
                rospy.logwarn(f"[TF] Failed to transform marker to map frame: {e}")
                continue

            rospy.loginfo(f"[YOLO-AprilTag] {label} @ ({X:.2f}, {Y:.2f}, {Z:.2f}) with ID={tag_id}")

        if detections_msg.detections:
            self.tag_pub.publish(detections_msg)
        if markers.markers:
            self.marker_pub.publish(markers)

if __name__ == "__main__":
    YOLOv8ToAprilTagBridge()
    rospy.spin()

