#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import math

def lidar_callback(scan):
    # LaserScan 메시지로부터 좌표 데이터 추출
    angles = [scan.angle_min + (scan.angle_increment * i) for i in range(len(scan.ranges))] # 리스트로 변환
    ranges = scan.ranges

    # ROI(Region of Interest) 설정
    roi_start_angle = math.pi  # 후방 전체
    roi_end_angle = 2 * math.pi  # 후방 전체
    roi_min_range = 0.0             # 최소 거리
    roi_max_range = 5.0             # 최대 거리

    # MarkerArray 메시지 생성
    marker_array = MarkerArray()

    for i in range(len(ranges)):
        # range 값이 inf인 경우 처리
        if math.isinf(ranges[i]):
            continue  # 처리를 원하는 방식으로 수정 가능

        # ROI 내에 있는 데이터만 처리
        if roi_start_angle <= angles[i] <= roi_end_angle and roi_min_range <= ranges[i] <= roi_max_range:
            # Marker 메시지 생성
            marker = Marker()
            marker.header.frame_id = scan.header.frame_id
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = ranges[i] * math.cos(angles[i])
            marker.pose.position.y = ranges[i] * math.sin(angles[i])
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
            marker.lifetime = rospy.Duration(1.0)

            # MarkerArray에 Marker 메시지 추가
            marker_array.markers.append(marker)

    # MarkerArray 메시지 Publish
    marker_array_publisher.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('lidar_visualization', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, lidar_callback)
    marker_array_publisher = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=1)
    rospy.spin()
