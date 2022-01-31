#!/usr/bin/env python
# license removed for brevity
import rospy
#from std_msgs.msg import String
from geometry_msgs.msg import Twist

def move_turtle():
    pub = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
    rospy.init_node('move_turtle', anonymous=True)
    rate = rospy.Rate(10) # 1hz
    while not rospy.is_shutdown():
        #hello_str = "hello world %s" % rospy.get_time()
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.8
        #rospy.loginfo(hello_str)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass