#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
import sys, termios, tty, select

def get_key(timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def create_joy_msg():
    msg = Joy()
    msg.axes = [0.0] * 6 
    msg.buttons = [0] * 12 
    return msg

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('keyboard_joy')
    pub = rospy.Publisher('/joy', Joy, queue_size=1)
    rate = rospy.Rate(10)

    print("""
Controls:
  w/s: +Z / -Z
  i/k: +X / -X
  j/l: +Y / -Y
  a/d: +Yaw / -Yaw

  t: Enable motors
  y: Disable motors
  x: Interrupt
  z: Slow mode

  q or Ctrl+C: Quit
""")

    try:
        while not rospy.is_shutdown():
            key = get_key()

            joy_msg = create_joy_msg()

            # Axes
            if key == 'i':
                joy_msg.axes[4] = 0.1 
            elif key == 'k':
                joy_msg.axes[4] = -0.1 
            elif key == 'j':
                joy_msg.axes[3] = 0.1   
            elif key == 'l':
                joy_msg.axes[3] = -0.1  
            elif key == 'w':
                joy_msg.axes[1] = 0.1   
            elif key == 's':
                joy_msg.axes[1] = -0.1 
            elif key == 'a':
                joy_msg.axes[0] = 0.1   
            elif key == 'd':
                joy_msg.axes[0] = -0.1 
            # Buttons
            elif key == 't':
                joy_msg.buttons[5] = 1  
            elif key == 'y':
                joy_msg.buttons[1] = 1  
            elif key == 'x':
                joy_msg.buttons[2] = 1 
            elif key == 'z':
                joy_msg.buttons[3] = 1  

            elif key == 'q':
                rospy.signal_shutdown()
                break

            pub.publish(joy_msg)
            rate.sleep()

    except KeyboardInterrupt:
        rospy.signal_shutdown()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


