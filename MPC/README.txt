안녕하세요 MPC 입니다

Scan2cost Package
말 그대로, 2d lidar 의 /scan 토픽을 받아서 local_costmap을 출력해줍니다.
local_costmap은 base_link frame 기준으로 생성됩니다. 따라서 laser frame의 /scan 과 겹쳐서 rviz2에 시각화하면, x축으로 -0.2m 만큼 /scan 데이터와 차이가 납니다.
static_transform 값을 수정하여 이 값을 조절할 수 있습니다. 나중에 tf calibration을 진행한 후, 해당 값을 입력해주면 될듯 합니다. 

Config.yaml 파일에서 설정 가능한 것 :
input topic, output topic 이름 변경 가능
base frame 변경 가능
map resolution, origin_x, origin_y 단위 [m]
width, heigt 단위 [cell]

static_transform 은 base_link 와 laser 사이의 tf 관계입니다.

[To-Do]
tf 토픽을 받아와서 자동으로 base_link 와 laser 사이의 관계 파악 (나중에 통합할 때 하나의 config에 합칠때 그냥 불러오도록 해도 됨)
