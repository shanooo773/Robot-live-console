export const ROBOT_TYPES = ["arm", "hand", "turtlebot"];

export const ROBOT_CODE_SNIPPETS = {
  arm: `#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time

def move_arm():
    # Initialize ROS node
    rospy.init_node('arm_controller', anonymous=True)
    
    # Publishers for joint control
    joint1_pub = rospy.Publisher('/robot_arm/joint1_position_controller/command', Float64, queue_size=10)
    joint2_pub = rospy.Publisher('/robot_arm/joint2_position_controller/command', Float64, queue_size=10)
    
    # Wait for publishers to initialize
    rospy.sleep(1)
    
    # Move joint 1 to 45 degrees (0.785 radians)
    joint1_pub.publish(0.785)
    rospy.sleep(2)
    
    # Move joint 2 to 30 degrees (0.524 radians)
    joint2_pub.publish(0.524)
    rospy.sleep(2)
    
    # Return to home position
    joint1_pub.publish(0.0)
    joint2_pub.publish(0.0)
    
    print("Arm movement completed!")

if __name__ == '__main__':
    try:
        move_arm()
    except rospy.ROSInterruptException:
        pass`,

  hand: `#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import time

def control_hand():
    # Initialize ROS node
    rospy.init_node('hand_controller', anonymous=True)
    
    # Publishers for finger control
    finger1_pub = rospy.Publisher('/robot_hand/finger1_position_controller/command', Float64, queue_size=10)
    finger2_pub = rospy.Publisher('/robot_hand/finger2_position_controller/command', Float64, queue_size=10)
    thumb_pub = rospy.Publisher('/robot_hand/thumb_position_controller/command', Float64, queue_size=10)
    
    # Wait for publishers to initialize
    rospy.sleep(1)
    
    # Open hand
    finger1_pub.publish(0.0)
    finger2_pub.publish(0.0)
    thumb_pub.publish(0.0)
    rospy.sleep(2)
    
    # Close fingers to grasp
    finger1_pub.publish(1.2)
    finger2_pub.publish(1.2)
    thumb_pub.publish(0.3)
    rospy.sleep(2)
    
    # Open hand again
    finger1_pub.publish(0.0)
    finger2_pub.publish(0.0)
    thumb_pub.publish(0.0)
    
    print("Hand grasping completed!")

if __name__ == '__main__':
    try:
        control_hand()
    except rospy.ROSInterruptException:
        pass`,

  turtlebot: `#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import time

def move_turtlebot():
    # Initialize ROS node
    rospy.init_node('turtlebot_controller', anonymous=True)
    
    # Publisher for velocity commands
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Create velocity message
    move_cmd = Twist()
    
    # Wait for publisher to initialize
    rospy.sleep(1)
    
    # Move forward
    move_cmd.linear.x = 0.2
    move_cmd.angular.z = 0.0
    vel_pub.publish(move_cmd)
    rospy.sleep(2)
    
    # Turn left
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.5
    vel_pub.publish(move_cmd)
    rospy.sleep(2)
    
    # Move forward again
    move_cmd.linear.x = 0.2
    move_cmd.angular.z = 0.0
    vel_pub.publish(move_cmd)
    rospy.sleep(2)
    
    # Stop
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    vel_pub.publish(move_cmd)
    
    print("TurtleBot movement completed!")

if __name__ == '__main__':
    try:
        move_turtlebot()
    except rospy.ROSInterruptException:
        pass`
};
