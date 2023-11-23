#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import math
import numpy as np
from std_msgs.msg import Float32, Int8MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped

def main():
    rospy.init_node("example_node")
    pub=rospy.Publisher('/utm', PoseStamped, queue_size=1)
    rate = rospy.Rate(5) 

    data=PoseStamped()
    while not rospy.is_shutdown():
        data.pose.position.x = 330247.310833+0.019166 + 0.00001
        data.pose.position.y = 4156753.599475+0.150524

        pub.publish(data)

        rate.sleep()

if __name__=='__main__':
    main()
