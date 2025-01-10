# Google Cartographer를 이용한 SLAM

## 1. 목적
이 프로젝트는 Google Cartographer를 이용한 SLAM(Simultaneous Localization and Mapping)을 수행한다. 사용한 센서는 imu와 2D LiDAR를 사용하였다.

## 2. 기능
### 매핑
- Mapping을 할 떄에는 `f1tenth bringup`과 `imu stella`를 실행한 상태에서 주행해야 한다. 
  이 때, `Damvi_carto_launch.py`를 실행하여 맵을 생성합니다. 이후 맵을 저장할 때에는 `nav2` 패키지의 `map saver`를 사용하여 저장한다.
  
- **주의:**
- 과도한 제자리 회전은 매핑 오류를 유발할 수 있다.
- mapping 시 loop closure를 수행하지 않을 경우 occupancy grid map이 불완전하게 작성될 수 있습니다.
- 고속 주행에서도 급격한 변화만 피하면 정상적으로 SLAM이 수행된다.

- **TODO:**
- 맵이 없는 상황에 대해서는 이 코드를 실행하면 된다. 하지만 맵을 다 따고 난 상황에서, 즉 이미 지도가 있을 때 이 지도에서 /scan과 /imu/data만을 가지고 localization을 수행하기 위해서는 cartographer의 `.lua` 파일에서 `pure_localization_only = true`로 설정해주고 관련 맵 파일인 `.pbstream`을 처리하는 코드를 추가해주어야 한다. 아직 작업을 시작하지 않았지만, 만일 이미 맵을 다 땄을 때, 계속 SLAM 코드를 실행해서 기존의 맵이 영향을 받기 시작한다면 이에 대한 작업을 시작해서 이 문제를 해결할 계획이다. 

### 로컬라이제이션
- 실시간 `/scan` 및 `/imu/data` 토픽을 사용할 수 있고, 또는 ROS bag 파일에서 데이터를 처리할 수도 있다. 이에 대한 코드는 서로 다르니 아래 내용을 참조하면 된다. 

## 3. 기존 cartograpehr 코드 외 추가 사항
- **odometry 처리:** Cartographer는 기본적으로 `odom` topic을 발행하지 않는다.
- 따라서 연산 처리 속도를 고려하여  C++ 노드(`trajectory_to_odom.cpp`)를 작성하여 아래 기능을 추가하였다:
  - 기존 `node_options.hpp`에 있는 어느 한 bool 값을 true로 변경하여 pose와 관련된 토픽이 발행되도록 변경해주었다.
  - map -> odom, odom->base_link에 대한 tf 변환관계를 처리하도록 코드를 수정하였다. 
  
## 4. 재현을 위한 요구사항
- ROS2 Humble. 
- 구성 파일:
  - `Damvi_carto_config.lua`: 매핑 및 로컬라이제이션 설정.
  - `Damvi_rosbag_launch.py`: ROS bag을 사용한 데이터 재생을 위한 실행 파일.
  - `Damvi_carto_launch.py`: Cartographer 실행을 위한 주요 실행 파일.
  - `localization.lua`: [TODO에서 쓰일]로컬라이제이션 전용 설정 파일.
  - `node_options.hpp`: odometry 처리에서 쓰이는 `/tracked_pose` 토픽을 발행하도록 설정하는 헤더파일.
    
## 5. 사용법

### [map-less, 실제 데이터 사용] SLAM 모드

1. 필요한 launch 파일을 시작:
   ```bash
   ros2 launch f1tenth_bringup bringup.launch.py
   ros2 launch stella_ahrs stella_ahrs_launch.py
   ```
   이때 반드시 imu 코드를 실행한 후 아래 2번으로 넘어가야 한다.
   
2. Cartographer 매핑을 실행:
   ```bash
   ros2 launch cartographer_ros Damvi_carto_launch.py
   ```
 
3. 과도한 회전을 피하면서 DAMVI를 주행.

### [rosbag 사용] SLAM 모드

1. `Damvi_carto_config.lua` 에서 rosbag 파일 경로 및 이름을 적절하게 변경해준다. 
   ```python
   rosbag_file = os.path.join(package_dir, 'data', 'example.bag')  # rosbag 파일 위치 지정
   ```

2. rosbag 기반 localization을 수행한다.
   ```bash
   ros2 launch damvi_cartographer Damvi_carto_launch.py use_pbstream:=true
   ```

