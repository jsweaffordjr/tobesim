#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import Float64


def talker():
    p1=rospy.Publisher('/tobe/l_ankle_lateral_joint_position_controller/command',Float64,queue_size=10)
    p2=rospy.Publisher('/tobe/l_ankle_swing_joint_position_controller/command',Float64,queue_size=10)
    p3=rospy.Publisher('/tobe/l_knee_joint_position_controller/command',Float64,queue_size=10)
    p4=rospy.Publisher('/tobe/l_hip_swing_joint_position_controller/command',Float64,queue_size=10)
    p5=rospy.Publisher('/tobe/l_hip_lateral_joint_position_controller/command',Float64,queue_size=10)
    p6=rospy.Publisher('/tobe/l_hip_twist_joint_position_controller/command',Float64,queue_size=10)
    p7=rospy.Publisher('/tobe/r_hip_twist_joint_position_controller/command',Float64,queue_size=10)
    p8=rospy.Publisher('/tobe/r_hip_lateral_joint_position_controller/command',Float64,queue_size=10)
    p9=rospy.Publisher('/tobe/r_hip_swing_joint_position_controller/command',Float64,queue_size=10)
    p10=rospy.Publisher('/tobe/r_knee_joint_position_controller/command',Float64,queue_size=10)
    p11=rospy.Publisher('/tobe/r_ankle_swing_joint_position_controller/command',Float64,queue_size=10)
    p12=rospy.Publisher('/tobe/r_ankle_lateral_joint_position_controller/command',Float64,queue_size=10)
    p13=rospy.Publisher('/tobe/l_shoulder_swing_joint_position_controller/command',Float64,queue_size=10)
    p14=rospy.Publisher('/tobe/l_shoulder_lateral_joint_position_controller/command',Float64,queue_size=10)
    p15=rospy.Publisher('/tobe/l_elbow_joint_position_controller/command',Float64,queue_size=10)
    p16=rospy.Publisher('/tobe/r_shoulder_swing_joint_position_controller/command',Float64,queue_size=10)
    p17=rospy.Publisher('/tobe/r_shoulder_lateral_joint_position_controller/command',Float64,queue_size=10)
    p18=rospy.Publisher('/tobe/r_elbow_joint_position_controller/command',Float64,queue_size=10)
    rospy.init_node('tobe_demo',anonymous=True)    
    rate=rospy.Rate(10) #10 Hz 
    while not rospy.is_shutdown():
    	p1.publish(-0.2) 
    	p2.publish(-1.1932)
   	p3.publish(-1.7264)
    	p4.publish(0.4132)
    	p5.publish(-0.15)
    	p6.publish(0)
    	p7.publish(0)
    	p8.publish(0.15)
    	p9.publish(-0.4132)
    	p10.publish(1.7264) 
    	p11.publish(1.1932)
    	p12.publish(0.2)
    	p13.publish(-0.3927)
    	p14.publish(-0.3491)
    	p15.publish(-0.5236)
    	p16.publish(0.3927)
    	p17.publish(-0.3491)
    	p18.publish(0.5236)
    	rate.sleep()

if __name__=="__main__":
    try:
    	talker()
    except rospy.ROSInterruptException:
	pass

