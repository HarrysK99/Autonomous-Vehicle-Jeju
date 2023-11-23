#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import math
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped

current_steer = 0.0
current_velocity = 0.0

vehicle_pos=np.array([0,0])

def find_nearest_point_index(vehicle_pos, global_path):
    """
    주어진 차량 위치에서 가장 가까운 global path 상의 좌표의 인덱스를 반환하는 함수
    """
    deltas = global_path-vehicle_pos
    distance=math.inf
    min_index=0
    for i,delta in enumerate(deltas):
        tmp=math.sqrt(delta[0]**2+delta[1]**2)
        if tmp<distance:
            min_index=i
            distance=tmp

    #distance = np.linalg.norm(global_path-vehicle_pos, axis=1)
    #print(distance)
    #nearest_point_index = np.argmin(distance)

    # 지나온 점의 좌표인지 구분하기 귀찮아서 좌표 인덱스 +1.
    return min_index+1


def utm52_callback(data):
    global vehicle_pos
    vehicle_pos = np.array([data.pose.position.x, data.pose.position.y])

def calculate_angle(a, b):
    delta = b-a
    print(delta)
    angle = math.degrees(math.atan(delta[0]/delta[1]))
    print("angle: "+str(angle))
    return angle

def calculate_distance(a, b):
    #print("vehicle_pos : ")
    #print(a)
    #print("point pos: ")
    #print(b)
    delta = b-a
    distance=math.sqrt(delta[0]**2+delta[1]**2)

    return distance

def main():
    global current_steer, current_velocity, vehicle_pos
    coordinates = []
    with open('pos_new.txt','r') as file:
        for line in file:
            x, y = map(float, line.strip().split(','))
            coordinates.append([x, y])

    global_path = np.array(coordinates)
    delta=global_path[1]-global_path[0]
    print(delta)
    print("distance:"+str(math.sqrt(delta[0]**2+delta[1]**2)))
    origin_angle = math.degrees(math.atan(delta[0]/delta[1]))
    print(origin_angle)
    #print(global_path)
    #print(len(global_path))

    vehicle_pos = global_path[0]

    rospy.init_node("vehicle_navigation")
    rospy.Subscriber("/utm", PoseStamped, utm52_callback)
    pub=rospy.Publisher('SpeedAngleGear', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(5) 

    while not rospy.is_shutdown():
        nearest_point_index = find_nearest_point_index(vehicle_pos, global_path)
        print(nearest_point_index)
        nearest_point = global_path[nearest_point_index]
        print(nearest_point)
        
        distance=calculate_distance(vehicle_pos,nearest_point)
        print(distance)
        current_velocity = distance*1.2*3.6
        current_steer = origin_angle-calculate_angle(nearest_point, vehicle_pos) 
        if(current_steer>=25):
            current_steer=25
        elif(current_steer<=-25):
            current_steer=-25
        actuating_data = Float32MultiArray()
        actuating_data.data=[current_velocity, current_steer, 0]

        pub.publish(actuating_data)

        rate.sleep()

if __name__=='__main__':
    main()
