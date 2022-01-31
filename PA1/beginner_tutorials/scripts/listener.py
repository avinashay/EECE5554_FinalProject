#!/usr/bin/env python
import rospy
#from std_msgs.msg import String
from geometry_msgs.msg import Twist
#from geometry_msgs.msg import Point
from turtlesim.msg import Pose


def callback1(data1):
    #rospy.loginfo(rospy.get_caller_id() + " I am at %s", data.x)
    rate = rospy.Rate(0.5) # 1hz
    while not rospy.is_shutdown():
        rospy.loginfo(" I am at (x,y) = %s",(data1.x,data1.y))
        rate.sleep()

def callback2(data2):
    #rospy.loginfo(rospy.get_caller_id() + " I am at %s", data.x)
    rate = rospy.Rate(0.5) # 1hz
    while not rospy.is_shutdown():
        rospy.loginfo(" I am moving with (Vx,thetaz) = %s",(data2.linear.x,data2.angular.z))
        rate.sleep()
    
def poisition():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('position', anonymous=True)

    rospy.Subscriber('/turtle1/pose', Pose, callback1)
    rospy.Subscriber('/turtle1/cmd_vel',Twist, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    poisition()