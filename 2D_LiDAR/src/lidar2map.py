#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

def lidar_callback(scan_msg):

    # 맵데이터 담을 변수 생성 ============================
    # 그리드 맵의 크기와 해상도 설정
    x_size=7
    y_size=4
    resolution = 0.1 #10cm/셀
    grid_size_x=int(x_size/resolution) #70
    grid_size_y=int(y_size/resolution) #40

    center = grid_size_y/2*resolution #2

    # 그리드 맵 데이터 초기화
    grid_map_data = np.zeros((grid_size_y, grid_size_x), dtype=np.int8)
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
    valid_indices=np.where((angles>=-math.pi/2) & (angles<=math.pi/2))
    valid_ranges=ranges[valid_indices]
    valid_angles=angles[valid_indices]
    # ==============================================

    # y= rsin(theta) -> -2m < y < 2m
    # x= rcos(theta) -> 0m < x < 7m

    # OccupancyGrid 메시지 생성
    occ_grid = OccupancyGrid()

    # Header 정보 설정
    occ_grid.header.stamp = rospy.Time.now()
    occ_grid.header.frame_id = scan_msg.header.frame_id

    # 맵 정보 설정 (해상도, 맵 크기, 원점)
    occ_grid.info.resolution = resolution
    occ_grid.info.width = grid_size_x
    occ_grid.info.height = grid_size_y
    occ_grid.info.origin.position.x = 0.0 
    occ_grid.info.origin.position.y = -center
    occ_grid.info.origin.position.z = 0.0
    occ_grid.info.origin.orientation.x = 0.0
    occ_grid.info.origin.orientation.y = 0.0
    occ_grid.info.origin.orientation.z = 0.0
    occ_grid.info.origin.orientation.w = 1.0

    # 점유 정보 설정 (레이저 센서에서 측정한 거리 값에 따라)
    for r,angle in np.nditer([valid_ranges, valid_angles]):
        if r == max_range:
            # 무한대 거리 값은 무시
            continue
        x=r*math.cos(angle)
        y=r*math.sin(angle)

        if(x>0 & x<7 & y>-2 & y<2):
            grid_x = int(x/resolution)  # 그리드 맵의 x 좌표 계산
            grid_y = int((-y+center)/resolution)  # 그리드 맵의 y 좌표 계산
            # 그리드 맵의 셀에 라인을 표시 (100: 점유된 공간)
            grid_map_data[grid_y, grid_x] = 100
        
    occ_grid.data = list(grid_map_data.flatten())

    # OccupancyGrid 메시지 발행
    occ_grid_pub.publish(occ_grid)

if __name__ == '__main__':
    rospy.init_node('lidar_to_occupancy_map')
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    occ_grid_pub = rospy.Publisher('/occupancy_grid', OccupancyGrid, queue_size=1)
    rospy.spin()
