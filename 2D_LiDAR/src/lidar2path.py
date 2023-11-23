#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
import math

def lidar_callback(scan_msg):

    # 측정 범위 및 해상도 설정
    x_size=7
    y_size=4
    resolution = 0.1 #point 간 간격 = 0.1m = 10cm

    # ==============================================

    # Raw Data에 대한 PreProcessing==================
    # LaserScan 메시지에서 거리 데이터 추출
    ranges = np.array(scan_msg.ranges)

    # 거리 값 중 NaN(무한대)인 부분을 max_range로 대체
    max_range = scan_msg.range_max
    ranges[np.isnan(ranges)] = max_range

    # 거리 값 중 Inf(무한대)인 부분을 max_range로 대체
    ranges[np.isinf(ranges)] = max_range

    # ROI 영역 내의 레이저 스캔 데이터 추출
    angles=np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(scan_msg.ranges))
    valid_indices=np.where((angles>=0 & angles<=math.pi/2))
    valid_ranges=ranges[valid_indices]
    valid_angles=angles[valid_indices]

    valid_ranges=valid_ranges[::-1]
    valid_angles=valid_angles[::-1]
    # ==============================================

    # PoseStamped 메시지 생성
    lidar_path = PoseArray()

    # Header 정보 설정
    lidar_path.header.stamp = rospy.Time.now()
    lidar_path.header.frame_id=scan_msg.header.frame_id

    # 일단 오른쪽 라인만 따라가도록.
    # 점유 정보 설정 (레이저 센서에서 측정한 거리 값에 따라)
    first = False
    y_offset=0
    for r,angle in np.nditer([valid_ranges, valid_angles]):
        if r == max_range:
            # 무한대 거리 값은 무시
            continue

        if (first == False):
            y_offset=abs(r*math.sin(angle))
            first=True

        x=r*math.cos(angle)
        y=-r*math.sin(angle) + y_offset
        
        if(x>0 & x<7 & y>-2 & y<2):
            new_pose=Pose()
            new_pose.position.x = x
            new_pose.position.y = y
            new_pose.position.z = 0.0
            new_pose.orientation.x = 0.0
            new_pose.orientation.y = 0.0
            new_pose.orientation.z = 0.0
            new_pose.orientation.w = 1.0

            lidar_path.poses.append(new_pose)

    # OccupancyGrid 메시지 발행
    lidar_path_pub.publish(lidar_path)

if __name__ == '__main__':
    rospy.init_node('lidar_to_path')
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    lidar_path_pub = rospy.Publisher('/lidar_path', PoseArray, queue_size=1)
    rospy.spin()
