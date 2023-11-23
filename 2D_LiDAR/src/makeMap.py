#!/usr/bin/env python3

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid

# 그리드 맵의 크기와 해상도 설정
x_size=7
y_size=4
resolution = 0.1 #10cm/셀
grid_size_x=int(x_size/resolution) #70
grid_size_y=int(y_size/resolution) #40

center = grid_size_y/2*resolution #2

# 그리드 맵 데이터 초기화
grid_map_data = np.zeros((grid_size_y, grid_size_x), dtype=np.int8)

# 라인 좌표값 설정 (예시)
line_points= [(i*resolution,center-2) for i in range(0,70)] #원소 위치는 실제 거리.
line_points+=[(i*resolution,center+2) for i in range(0,70)]

line_points+=[(3, o*resolution) for o in range(5,30)]
obstacle_points=[(3+0.3, o*resolution) for o in range(5, 30)]
obstacle_points+=[(3-0.3, o*resolution) for o in range(5, 30)]
obstacle_points+=[(o*0.1, 0.5) for o in range(27,34)]
obstacle_points+=[(o*0.1, 3.0) for o in range(27,34)]

line_points+=[(5, o*resolution) for o in range(15,40)]
obstacle_points+=[(5+0.3, o*resolution) for o in range(15, 40)]
obstacle_points+=[(5-0.3, o*resolution) for o in range(15, 40)]
obstacle_points+=[(o*0.1, 1.5) for o in range(47,54)]
obstacle_points+=[(o*0.1, 3.9) for o in range(47,54)]



# line_points+=[(0,2)]

# 그리드 맵에 라인 표시
for point in line_points:
    x, y = point
    grid_x = int(x / resolution)  # 그리드 맵의 x 좌표 계산
    grid_y = int(y / resolution)  # 그리드 맵의 y 좌표 계산
    if grid_y == 40:
        grid_y=39
    # 그리드 맵의 셀에 라인을 표시 (100: 점유된 공간)
    grid_map_data[grid_y, grid_x] = 100

for point in obstacle_points:
    x, y = point
    grid_x = int(x / resolution)  # 그리드 맵의 x 좌표 계산
    grid_y = int(y / resolution)  # 그리드 맵의 y 좌표 계산
    if grid_y == 40:
        grid_y=39
    # 그리드 맵의 셀에 라인을 표시 (100: 점유된 공간)
    grid_map_data[grid_y, grid_x] = 50

print(grid_map_data)
print(grid_map_data.shape)

# ROS 메시지로 그리드 맵 정보 전송
rospy.init_node('grid_map_publisher')
pub = rospy.Publisher('grid_map', OccupancyGrid, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # ROS 메시지 생성 및 설정
    grid_map = OccupancyGrid()
    grid_map.header.stamp = rospy.Time.now()
    grid_map.header.frame_id = 'base_link'
    grid_map.info.width = grid_size_x
    grid_map.info.height = grid_size_y
    grid_map.info.resolution = resolution
    grid_map.info.origin.position.x = 0.0
    grid_map.info.origin.position.y = -center
    grid_map.info.origin.position.z = 0.0
    grid_map.info.origin.orientation.x = 0.0
    grid_map.info.origin.orientation.y = 0.0
    grid_map.info.origin.orientation.z = 0.0
    grid_map.info.origin.orientation.w = 1.0
    grid_map.data = list(grid_map_data.flatten())

    # ROS 메시지 발행
    pub.publish(grid_map)
    rate.sleep()
