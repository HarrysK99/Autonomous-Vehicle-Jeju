#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32MultiArray
import numpy as np


class PurePursuit:
    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance
        self.path = []  # 경로 데이터를 저장하는 변수
        self.current_pose = Point(0.0, 0.0, 0.0)  # 현재 로봇의 위치를 저장하는 변수
        self.speed = 10.0 / 3.6  # 초기 속도를 10 km/h (2.78 m/s)로 설정

        # ROS 메시지 퍼블리셔 및 서브스크라이버 생성
        self.drive_pub = rospy.Publisher('SpeedAngleGear', Float32MultiArray, queue_size=1)
        self.path_sub = rospy.Subscriber('/path', Path, self.path_callback)

    def adjust_speed(self, delta):
        # 조향각에 따른 속도 조절 로직 구현
        max_speed = 10.0 / 3.6  # 최대 속도 10 km/h (2.78 m/s)
        min_speed = 0.5  # 최소 속도 0.5 m/s

        # 각도에 따라 속도를 조절합니다.
        # 조향각이 작을수록 차량이 직진하는 것이므로 높은 속도를 유지할 수 있습니다.
        # 조향각이 클수록 차량이 급커브를 돌아야 하므로 속도를 낮춰야 합니다.
        normalized_angle = abs(delta) / np.radians(25)  # 조향각을 정규화합니다. (0 ~ 1 사이의 값)
        speed = max_speed - (max_speed - min_speed) * normalized_angle

        return speed


    def path_callback(self, msg):
        # 경로 데이터 콜백 함수
        self.path = [pose.pose.position for pose in msg.poses]

        # 현재 로봇의 위치와 경로 데이터를 기반으로 Pure Pursuit 알고리즘 수행
        if not self.path:
            return

        # 현재 로봇의 위치와 경로 데이터를 Numpy 배열로 변환
        current_x = self.current_pose.x
        current_y = self.current_pose.y
        path_x = np.array([p.x for p in self.path])
        path_y = np.array([p.y for p in self.path])

        # 현재 로봇의 위치에서 가장 가까운 경로 상의 점을 찾음
        distances = np.sqrt((path_x - current_x) ** 2 + (path_y - current_y) ** 2)
        closest_index = np.argmin(distances)

        # 현재 로봇의 위치에서 미래에 가장 가까운 경로 상의 점을 찾음
        lookahead_index = closest_index
        while lookahead_index < len(self.path) - 1 and \
                np.sqrt((path_x[lookahead_index] - current_x) ** 2 +
                        (path_y[lookahead_index] - current_y) ** 2) < self.lookahead_distance:
            lookahead_index += 1

        # Pure Pursuit 알고리즘을 통해 제어 명령 계산
        tx = path_x[lookahead_index]
        ty = path_y[lookahead_index]
        alpha = np.arctan2(ty - current_y, tx - current_x)
        delta = np.degrees(alpha)

        steering_angle = np.clip(delta, -np.radians(25), np.radians(25))  # 조향각을 -25° ~ 25°로 제한

        # 조향각에 따라 속도를 조절.
        self.speed = self.adjust_speed(steering_angle)

        # AckermannDriveStamped 메시지 생성 및 퍼블리시
        drive_msg = Float32MultiArray()        
        drive_msg.data=(self.speed, steering_angle, 0.0)
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_node')
    lookahead_distance = 1.0  # Pure Pursuit 알고리즘에서 사용할 미래에 찾을 점의 거리 설정
    pure_pursuit = PurePursuit(lookahead_distance)
    rospy.spin()
