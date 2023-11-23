Path Planning은 Global Path Planning과 Local Path Planning으로 나뉜다.
1) Global Path Planning : planning 전에 주위 정보들을 모두 안다.
2) Local Path Planning : 주위 정보를 모른다.


Global Path Planning은 다음 step을 따른다.

![image](https://user-images.githubusercontent.com/97833069/231522185-eb8616e3-2f16-47d8-8e53-4bab06f250af.png)

```
(1) Environmental Modeling: 
    알려진 지도 정보에 따라 환경 모델링을 구축하여 작업을 수행할 수 있는 실제 환경을 지도 피처 정보로 변환하여 편리하게 저장.
(2) Optimization Criteria
    목적 함수에 따라 최적화를 진행.
(3) Path Search Algorithm
    Path Search 알고리즘은 starting 지점과 target 지점 사이의 충돌에 자유로운 Path를 찾는 것이 목적.
    * Path length, smoothness, safety degree 등 최적 criteria를 만족해야 함.
```

차선 경로 : nav_msgs/Path 메시지 타입. (경로는 차량 기준 몇 m 앞까지 만드는가?)

→ 두 경로의 중심 포인트를 초기 경로로 설정. (nav_msgs/Path)

장애물 : 2D LiDAR 혹은 3D LiDAR를 이용하여 장애물의 위치 정보를 뽑아냄. 

→ 장애물과 차선 경로를 map 정보로 제공하여 장애물을 회피하도록 경로 수정을 한다.
(이때 map 정보는 경로 수정 알고리즘에서 np.array 배열로 변환해 사용)

#2차원 np.array 배열로 그리드 맵을 만들어 사용하는게 효율적이고 빠를 것이라고 한다.
#맵 간격을 0.2m로 한다고 하고 맵의 크기를 가로 5m 세로 5m라고 한다면 25x25=625칸이 발생함.

일단은 https://velog.io/@ghost_dog/%EA%B3%A0%EB%93%B1%ED%95%99%EA%B5%90-A-star-%EC%95%8C%EA%B3%A0%EB%A6%AC%EC%A6%98 ]
이 친구의 A* 알고리즘을 사용해서 프로토 완성할 예정..
