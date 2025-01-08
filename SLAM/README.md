# Google Cartographer를 이용한 SLAM

## 1. 목적
이 프로젝트는 Google Cartographer를 이용한 SLAM(Simultaneous Localization and Mapping)을 구현합니다. 2D LiDAR와 IMU 센서 토픽을 활용하여 포즈 그래프 기반의 2D occupancy grid map을 생성하고, 동시에 생성된 맵에서 DAMVI의 위치를 추정합니다.

## 2. 기능
### 매핑
- 저속 주행 시 `Damvi_carto_launch.py`를 실행하여 맵을 생성합니다.
- 매핑 작업 시 `f1tenth bringup`과 `imu stella`를 실행한 상태에서 주행해야 합니다.
- **주의:** 과도한 제자리 회전은 매핑 오류를 유발할 수 있습니다.
- 루프 클로저를 수행하지 않을 경우 occupancy grid map이 불완전하게 작성될 수 있습니다.
- 고속 주행에서도 급격한 변화만 피하면 정상적으로 매핑이 수행됩니다.

### 로컬라이제이션
- 이 구성은 SLAM이 아닌 로컬라이제이션에 중점을 둡니다.
- 기존의 `pbstream` 맵 파일(예: `final_map.pbstream`)을 사용하여 로컬라이제이션을 수행합니다. (이에 대한 코드는 추후 작성될 예정입니다. 현재는 SLAM 코드를 임시로 작동하고 있습니다)
- 실시간 `/scan` 및 `/imu/data` 토픽을 사용할 수 있고, 또는 ROS bag 파일에서 데이터를 처리할 수도 있습니다.

## 3. 추가 사항
- **오도메트리 처리:** Cartographer는 기본적으로 `odom`을 발행하지 않습니다.
- 맞춤형 C++ 노드(`trajectory_to_odom.cpp`)를 개발하여 아래 기능을 추가했습니다:
  - MarkerArray와 IMU 데이터를 이용해 절대 위치와 헤딩(방향)을 계산합니다.
  - 다른 ROS 구성 요소와 통합을 위한 프레임을 발행합니다.

## 4. 요구사항
- ROS2 Humble, TurtleBot3 및 Cartographer 패키지 설치 필요.
- 구성 파일:
  - `Damvi_carto_config.lua`: 매핑 및 로컬라이제이션 설정.
  - `Damvi_rosbag_launch.py`: ROS bag을 사용한 데이터 재생을 위한 실행 파일.
  - `Damvi_carto_launch.py`: Cartographer 실행을 위한 주요 실행 파일.
  - `localization.lua`: 로컬라이제이션 전용 설정 파일.

## 5. 사용법
### 매핑 모드
1. 필요한 노드를 시작합니다:
   ```bash
   ros2 launch f1tenth_bringup bringup.launch.py
   ros2 launch imu_stella imu_stella.launch.py
   ```
2. Cartographer 매핑을 실행합니다:
   ```bash
   ros2 launch damvi_cartographer Damvi_carto_launch.py
   ```
   현재 이 코드를 수행해서 Localization을 수행하고 있습니다. 하지만 추후 수정할 예정입니다. 

3. 과도한 회전을 피하면서 DAMVI를 주행합니다.

### 로컬라이제이션 모드
1. 기존 맵을 이용하여 로컬라이제이션을 수행합니다:
   ```bash
   ros2 launch damvi_cartographer Damvi_carto_launch.py use_pbstream:=true
   ```
2. 필요한 `pbstream` 맵 파일을 제공합니다:
   ```bash
   --pbstream_filename final_map.pbstream
   ```
### Rosbag 모드
0. ROS bag 데이터를 재생합니다(선택 사항):
   ```bash
   ros2 launch damvi_cartographer Damvi_rosbag_launch.py
   ```

## 6. 주의사항
- 맵의 정확도를 높이기 위해 루프 클로저를 반드시 활성화하세요.
- 매핑 중 급격한 회전이나 방향 전환을 피하세요.
- 고속 주행 전에 설정을 철저히 테스트하세요.

