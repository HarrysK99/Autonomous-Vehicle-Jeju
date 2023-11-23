#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32MultiArray, Bool
from PythonRobotics.PathTracking.pure_pursuit import pure_pursuit
import numpy as np

# Global variables for storing the path and vehicle position
global_path = Path()
global_vehicle_position = PoseStamped()
trigger = False
success = False

def path_callback(msg):
    global trigger

    if trigger == False:
        global global_path
        global_path = msg
        trigger = True


def vehicle_position_callback(msg):
    global global_vehicle_position
    global_vehicle_position = msg

def main():
    global global_path, global_vehicle_position, trigger, success
    rospy.init_node("pure_pursuit_controller")

    path_sub = rospy.Subscriber("/rrt_star_reeds_shepp_path", Path, path_callback)
    vehicle_position_sub = rospy.Subscriber("/vehicle_position", PoseStamped, vehicle_position_callback)
    control_pub = rospy.Publisher("SpeedAngleGear", Float32MultiArray, queue_size=1)
    success_pub = rospy.Publisher("Success", Bool, queue_size=1)

    speed = 3.0/3.6 # [m/s]

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # 경로 추종 시작
        if trigger == True:
            success = False

            # target course
            cx = np.array([global_path.poses.pose.position.x for i in range(0,len(global_path.poses))])
            cy = [global_path.poses.pose.position.y for i in range(0,len(global_path.poses))]
            lastIndex=len(cx)-1
            target_course = pure_pursuit.TargetCourse(cx, cy)

            # target speed
            target_speed = 7.0/3.6 #[m/s]

            # initial State
            state=pure_pursuit.State(x=-0.0,y=-0.0,yaw=0.0, v=speed)

            target_ind, _ = target_course.search_target_index(state)

        # 경로 추종 
        if trigger==True and \
            global_vehicle_position.pose.position.x < 7 and \
            lastIndex > target_ind and \
            success == False : 
            
            #Calc control input
            ai = pure_pursuit.proportional_control(target_speed, state.v)
            di, target_ind = pure_pursuit.pure_pursuit_steer_control(
                state, target_course, target_ind
            )

            state.update(ai, di)

            drive_msg = Float32MultiArray()
            drive_msg.data = (state.v*3.6, state.yaw, 0.0)
            control_pub.publish(drive_msg)

            rate.sleep()

        # 경로 추종 종료
        if global_vehicle_position.pose.position.x >= 7 :
            success = True

            success_msg = Bool()
            success_msg.data = True

            success_pub.publish(success_msg)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
