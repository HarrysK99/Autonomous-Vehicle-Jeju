<div align="justify">
<h2>1/5 EV Autonomous System</h2>
  
**2023 제2회 국제대학생 EV 자율주행 경진대회** 자율주행 시스템입니다.

차선 인식 구간과 GPS waypoints 구간으로 나누어, 정적/동적 장애물을 피하면서 완주하는 미션을 수행했습니다.

</div>

<p align="center">
  <br>
  <img src="https://github.com/HarrysK99/Autonomous-Vehicle-Jeju/assets/81846798/d75de636-157d-4a0b-b3a3-b076d6f86ed5" width="280" height="400">  <img src="https://github.com/HarrysK99/Autonomous-Vehicle-Jeju/assets/81846798/a862ffa8-7d77-4f66-b58b-e22b0f3eaa21" width="500" height="400">
  <br>
</p>

## 목차
  - [개요](#개요) 
  - [구현 기능](#구현-기능)
  - [보완할 점](#보완할-점)

<br>

## 개요
- 프로젝트 지속기간: 2023.03-2022.05 (2개월)
- 개발 언어 및 기술: C++ & Python & ROS
- 팀 규모: 5인 1팀
- 개발 환경
  - OS: Ubuntu 20.04
  - GPU: RTXTM 3060
  - ROS noetic
- HW 구성
  - 아두이노 보드
  - 모터 드라이버
  - Zed2i stereo camera
  - C099-f9p GPS
  - 2D rp-Lidar
  - RC 송수신기 (수동 모드 동작용)

### 수상
● 🥈최우수상 - 제2회 국제 대학생 EV 자율주행 경진대회

<br>

## 구현 기능

### Block Diagram

<img src="https://github.com/HarrysK99/Autonomous-Vehicle-Jeju/assets/81846798/3b33cb0e-61d8-4876-8ea7-e6a10d2bdebc">
<br>

### 구현
1. 2D LiDAR Object Detection: DBSCAN Clusteringimg

2. Camera Lane Detection: OpenCV Sliding Window
<img src="https://github.com/HarrysK99/Autonomous-Vehicle-Jeju/assets/81846798/026822bb-8d1d-42d0-b033-dafe7e7a889f" width="400" height="200">

3. GPS Data Processing: VRS-RTK

4. Local Planning: RRT* with reeds-shepp path

5. High-Level Control

    6.1. Pure Pursuit algorithm → Steering $\delta={{2L sin \alpha}\over{l_{d}}}$

    6.2. Velocity $v=v_{max}-|{\delta \over \delta_{max}}|(v_{max}-v_{min})$

6. Low-Level Control

    6.1. PID Controller for each state → $state=K_{P}e+K_{I}\int{e}dt+K_{D}\dot{e}$

    <img src="https://github.com/HarrysK99/EV-Autonomous/assets/81846798/27d1bc34-106d-4c2e-8fe3-071f25a8486b" width="600" height="300">
<br>

### VRS-RTK
<p align="center">
<img src="https://github.com/HarrysK99/Autonomous-Vehicle-Jeju/assets/81846798/c75a8a5f-74fa-4da0-bfb5-638114b39d03" width="250" height="250">
</p>

RTK는 Real-Time Kinematic의 약자로, 두 개의 GPS 수신기를 이용해 cm 수준의 정확도를 달성할 수 있다. 하나는 기지국(Base Station), 다른 하나는 이동국(Rover)로, 정확한 고정 위치에 설치되어 있는 기준국을 통해 보정 신호를 만들어내고, 이동국은 이 보정 신호를 받아 연산을 통해 정확한 위도, 경도를 얻게 되는 것이다. RTK를 이용하면 대기 중의 전리층 지연, 위성의 궤도 오차, 위성 시계의 오차 등의 오차 요소를 보정할 수 있다.
<br/><br/>
VRS는 단일 기준국 대신 여러 기준국 네트워크를 사용하여 실시간으로 정확한 위치 정보를 제공할 수 있다. VRS의 특징은 가상 기준국을 생성한다는 것이다. 가상 기준국이기 때문에 내가 위치를 임의로 지정할 수 있고, 이동국과 최대한 가까운 위치에 가상 기준국을 생성하면 거리에 따른 오차가 최소화된다. 또한 여러 네트워크를 통해 보정신호를 생성하기 때문에 넓은 구간에서 정확도를 가질 수 있다. 다만 네트워크를 구축한 서비스 제공자가 있어야 한다.
<br/><br/>
우리는 국토지리정보원을 통해 VRS를 이용하였다. 아래 표는 국토지리정보원에서 제공하는 서비스의 종류이다.
<br/><br/>
RTK를 이용한 시스템 흐름은 다음과 같다.
![image](https://github.com/HarrysK99/Autonomous-Vehicle-Jeju/assets/81846798/f1b792e7-7ee7-480a-aac0-6d0f5a5b1985)

<br>

## 보완할 점
- [ ] MPTG(Motion Predictive Trajectory Generator) 적용해보기
- [ ] Dynamic Vehicle Model 적용해보기

</p>

<br>

## 라이센스

MIT &copy; [NoHack](mailto:lbjp114@gmail.com)
