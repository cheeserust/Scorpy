# Raspberry Pi 5 / PC 패키지 분리 구성

## 목표

구성은 아래를 기준으로 한다.

```text
카메라 2대
  -> Raspberry Pi 5
  -> LAN
  -> PC
  -> USB2CANFD
  -> STM32 Board1/Board2/Board3
  -> 팔 + 그리퍼
```

지연을 줄이는 원칙은 단순하다.

1. 카메라 원본 영상은 Raspberry Pi 5 안에서 최대한 처리한다.
2. USB2CANFD와 직접 붙는 팔 제어 노드는 PC에서 실행한다.
3. Raspberry Pi 5와 PC 사이 LAN에는 원본 영상이나 세부 CAN 명령이 아니라, 목표값/상태/Action 결과 같은 작은 메시지만 보낸다.

## 권장 배치 요약

| 장비 | 넣을 패키지 | 이유 |
| --- | --- | --- |
| Raspberry Pi 5 | 카메라 드라이버, 카메라 보정, ArUco/객체/엘리베이터/층 인식 패키지 | 카메라 원본을 LAN으로 보내지 않고 로컬에서 처리하기 위해 |
| Raspberry Pi 5 | VicPinky Nav2, localization, map server, base controller 관련 패키지 | 주행 센서/베이스 제어와 가까운 곳에서 주행 루프를 돌리기 위해 |
| Raspberry Pi 5 | `mission_manager` | 주행/인지 쪽 상태를 가까이 두고 전체 미션을 조율하기 위해 |
| Raspberry Pi 5 | `vicpinky_nav_adapter` | `/nav/go_to`를 같은 장비의 Nav2 `/navigate_to_pose`로 넘기기 위해 |
| Raspberry Pi 5 | `vicpinky_interfaces` | PC와 같은 Action/Msg 타입을 쓰기 위해 필수 |
| Raspberry Pi 5 | `vicpinky_gui` 선택 | 로봇 자체에서 웹 대시보드를 띄우고 싶을 때 |
| PC | `arm_can_bridge` | USB2CANFD가 PC에 있으므로 반드시 PC에서 실행 |
| PC | `arm_task_server` | `/arm/pick`, `/arm/place`, `/arm/press_button`을 PC 안에서 바로 trajectory로 바꾸기 위해 |
| PC | `roscue_arm_description` | MoveIt/RViz/robot_state_publisher에서 팔 모델을 쓰기 위해 |
| PC | `roscue_arm_moveit_config` | PC에서 팔 경로 계획을 하고, 바로 `arm_can_bridge`로 넘기기 위해 |
| PC | `vicpinky_interfaces` | Raspberry Pi 5와 같은 Action/Msg 타입을 쓰기 위해 필수 |
| PC | `vicpinky_gui` 선택 | 조작자가 PC에서 대시보드를 보고 직접 제어할 때 |

## Raspberry Pi 5에 두는 것

Raspberry Pi 5는 카메라와 주행 중심 장비로 둔다.

권장 실행 대상:

```text
vicpinky_interfaces
mission_manager
vicpinky_nav_adapter
vicpinky_gui             # 선택
카메라 드라이버 패키지
카메라 인식/마커 인식 패키지
VicPinky Nav2 / localization / map 관련 패키지
베이스 하드웨어 제어 패키지
```

Raspberry Pi 5에서 피하는 것이 좋은 대상:

```text
arm_can_bridge           # USB2CANFD가 PC에 있으므로 실행하지 않음
roscue_arm_moveit_config # PC에서 팔 계획을 하는 구성이 지연/부하 면에서 유리
board1_simulator         # 실제 STM32 연결 시 실행하지 않음
mock_task_servers        # 개발/테스트용
dummy_servers            # 개발/테스트용
```

Raspberry Pi 5 빌드 예시:

```bash
cd ~/vicpinky_server_ws
colcon build --symlink-install --packages-select \
  vicpinky_interfaces \
  mission_manager \
  vicpinky_nav_adapter \
  vicpinky_gui
source install/setup.bash
```

실제 운용에서는 아래의 `실제 운용 실행 순서`처럼 launch마다 터미널을 나눠서 실행한다. 카메라/Nav2 관련 launch는 VicPinky 본체 패키지 기준으로 Raspberry Pi 5에서 같이 실행한다.

## PC에 두는 것

PC는 팔/그리퍼 제어 중심 장비로 둔다.

권장 실행 대상:

```text
vicpinky_interfaces
arm_can_bridge
arm_task_server
roscue_arm_description
roscue_arm_moveit_config
vicpinky_gui             # 선택
```

PC에서 피하는 것이 좋은 대상:

```text
카메라 원본 처리 노드       # 원본 영상을 LAN으로 계속 받아야 하므로 지연/트래픽 증가
Nav2 local controller     # 베이스 제어가 Raspberry Pi 5에 있다면 PC에서 돌리지 않음
board1_simulator          # 실제 STM32 연결 시 실행하지 않음
mock_task_servers         # 개발/테스트용
dummy_servers             # 개발/테스트용
```

PC 빌드 예시:

```bash
cd ~/vicpinky_server_ws
colcon build --symlink-install --packages-select \
  vicpinky_interfaces \
  arm_can_bridge \
  arm_task_server \
  roscue_arm_description \
  roscue_arm_moveit_config \
  vicpinky_gui
source install/setup.bash
```

CAN interface 준비 예시:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

현재 `arm_can_bridge` 프로토콜은 8-byte classic CAN frame 기준이다. USB2CANFD 장치를 쓰더라도 STM32 firmware와 서버 프로토콜이 CAN FD로 바뀌기 전에는 classic CAN 모드로 맞추는 것이 안전하다.

실제 운용에서는 아래의 `실제 운용 실행 순서`처럼 `arm_can_bridge`, `arm_task_server`, `move_group`, RViz 선택 실행을 터미널별로 나눠서 실행한다.

## 실제 운용 실행 순서

중요한 점은 `source install/setup.bash`를 한 번만 하는 것이 아니라, ROS 2 노드를 실행하는 모든 새 터미널마다 해줘야 한다는 것이다. `ros2 launch ...` 명령은 계속 실행 상태로 남기 때문에 보통 launch 하나당 터미널 하나를 쓴다.

아래 예시는 팀에서 사용하는 ROS 2 Jazzy 기준이다.

### 공통 터미널 준비

Raspberry Pi 5와 PC의 모든 터미널에서 노드를 실행하기 전에 아래를 먼저 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호
```

두 장비에서 RMW 구현을 고정해서 쓸 경우에는 아래도 같은 값으로 맞춘다.

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

이 설정을 매번 치기 번거로우면 Raspberry Pi 5와 PC의 `~/.bashrc`에 같은 내용을 넣어도 된다. 그래도 workspace를 새로 빌드한 뒤에는 새 터미널을 열거나 `source ~/vicpinky_server_ws/install/setup.bash`를 다시 실행하는 것이 안전하다.

### PC 실행 순서

PC는 USB2CANFD와 팔/그리퍼 제어를 담당한다. Raspberry Pi 5에서 미션을 시작하기 전에 PC 쪽 팔 서버가 먼저 떠 있어야 한다.

PC 터미널 1, CAN interface 준비:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

PC 터미널 2, STM32 CAN bridge:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

ros2 launch arm_can_bridge arm_can_bridge.launch.py can_interface:=can0
```

PC 터미널 3, semantic arm task server:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

ros2 launch arm_task_server arm_task_server.launch.py
```

PC 터미널 4, MoveIt2 move_group:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

ros2 launch roscue_arm_moveit_config move_group.launch.py
```

PC 터미널 5, RViz 선택:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

ros2 launch roscue_arm_moveit_config moveit_rviz.launch.py
```

PC에서 GUI를 볼 경우에는 PC 터미널 6에서 실행한다. GUI는 Raspberry Pi 5 또는 PC 중 한 곳에서만 실행하면 된다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

ros2 launch vicpinky_gui vicpinky_gui.launch.py
```

팔 보드 상태 확인:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
```

초기 구동 시에는 상태 확인 후 enable, homing 순서로 진행한다.

```bash
ros2 service call /arm_board/enable std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/home_all std_srvs/srv/Trigger '{}'
ros2 service call /arm_board/status std_srvs/srv/Trigger '{}'
```

### Raspberry Pi 5 실행 순서

Raspberry Pi 5는 카메라, 인식, 주행, 미션 관리를 담당한다.

Raspberry Pi 5 터미널 1, 카메라 2대:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

# 실제 카메라 패키지 launch로 교체
ros2 launch <camera_package> <camera_launch>.launch.py
```

Raspberry Pi 5 터미널 2, 인식/마커/엘리베이터 상태 노드:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

# 실제 인식 패키지 launch로 교체
ros2 launch <perception_package> <perception_launch>.launch.py
```

Raspberry Pi 5 터미널 3, Nav2/localization/base control:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

# 실제 VicPinky Nav2 launch로 교체
ros2 launch <vicpinky_nav_package> <nav_bringup>.launch.py
```

Raspberry Pi 5 터미널 4, Nav adapter:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

ros2 launch vicpinky_nav_adapter nav_adapter.launch.py
```

Raspberry Pi 5 터미널 5, mission manager:

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

ros2 launch mission_manager mission_manager.launch.py
```

Raspberry Pi 5에서 GUI를 띄울 경우에는 Raspberry Pi 5 터미널 6에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

ros2 launch vicpinky_gui vicpinky_gui.launch.py
```

### 미션 실행 전 확인

두 장비에서 같은 ROS graph가 보이는지 확인한다.

```bash
ros2 node list
ros2 action list
ros2 service list
```

최소한 아래 항목들이 보여야 한다.

```text
/mission/execute
/nav/go_to
/arm/pick
/arm/place
/arm/press_button
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
/arm_board/status
```

GUI를 쓰지 않고 CLI로 미션을 보낼 때는 Raspberry Pi 5 또는 PC 어느 쪽에서든 실행할 수 있다. 보통 mission manager가 있는 Raspberry Pi 5에서 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ~/vicpinky_server_ws/install/setup.bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호

ros2 run mission_manager send_mission --list-locations
ros2 run mission_manager send_demo_mission
```

## 정보 흐름

### 1. 카메라/인지

```text
카메라 2대
  -> Raspberry Pi 5 카메라 노드
  -> Raspberry Pi 5 인식 노드
  -> marker_id, object pose, floor state, door state 같은 작은 결과 메시지
```

원본 이미지는 Raspberry Pi 5 내부에서만 사용한다. PC에서 모니터링이 꼭 필요할 때만 `image_transport/compressed` 같은 압축 영상으로 낮은 FPS를 보낸다.

### 2. 주행

```text
mission_manager
  -> /nav/go_to                  vicpinky_interfaces/action/RunTask
  -> vicpinky_nav_adapter
  -> /navigate_to_pose           nav2_msgs/action/NavigateToPose
  -> VicPinky Nav2
  -> 베이스 제어
```

주행 루프는 Raspberry Pi 5에서 닫히고, PC는 주행 세부 제어에 끼지 않는다.

### 3. 팔/그리퍼 미션

```text
Raspberry Pi 5 mission_manager
  -> /arm/pick, /arm/place, /arm/press_button
     vicpinky_interfaces/action/RunTask over LAN
  -> PC arm_task_server
  -> /arm_controller/follow_joint_trajectory
     control_msgs/action/FollowJointTrajectory
  -> PC arm_can_bridge
  -> can0
  -> USB2CANFD
  -> STM32 Board1/Board2/Board3
```

Raspberry Pi 5에서 PC로 넘어가는 것은 `pick`, `place`, `press_button` 같은 의미 단위 Action이다. 여러 개의 trajectory point와 CAN frame 변환은 PC 내부에서 처리한다.

### 4. STM32 피드백

```text
STM32 Board1/2/3
  -> CAN status/position feedback
  -> USB2CANFD can0
  -> PC arm_can_bridge
  -> /joint_states
  -> /arm_board/status_log
  -> /arm_board/status service 응답
```

`arm_can_bridge`는 아래 인터페이스를 제공한다.

| 인터페이스 | 방향 | 용도 |
| --- | --- | --- |
| `/arm_controller/follow_joint_trajectory` | Action Server | 팔 5축 trajectory 수신 |
| `/gripper_controller/follow_joint_trajectory` | Action Server | 그리퍼 9축 trajectory 수신 |
| `/arm_board/enable` | Service | STM32 enable |
| `/arm_board/disable` | Service | STM32 disable |
| `/arm_board/home_all` | Service | homing/home posture |
| `/arm_board/clear_error` | Service | error clear |
| `/arm_board/estop` | Service | emergency stop |
| `/arm_board/status` | Service | 보드 상태 확인 |
| `/joint_states` | Topic | 실제 위치 피드백 또는 commanded estimate |
| `/arm_board/status_log` | Topic | 주기적 보드 상태 로그 |

### 5. GUI

GUI는 Raspberry Pi 5 또는 PC 중 한 곳에서만 실행하면 된다.

```text
브라우저
  -> vicpinky_gui HTTP
  -> /mission/execute
  -> /mission/status, /mission/event_log 구독
  -> /arm_board/* service 호출
  -> /joint_states, /arm_board/status_log 구독
```

조작자가 PC 앞에서 작업한다면 PC에서 `vicpinky_gui`를 실행하는 편이 편하다. 로봇 IP 하나로 접속하고 싶다면 Raspberry Pi 5에서 실행해도 된다. 지연에 큰 영향을 주는 패키지는 아니다.

## 네트워크 설정 체크

Raspberry Pi 5와 PC는 같은 ROS 2 domain에 있어야 한다.

```bash
export ROS_DOMAIN_ID=핑키에서_정한_같은_번호
```

권장 사항:

1. Raspberry Pi 5와 PC는 유선 LAN과 고정 IP를 사용한다.
2. 두 장비의 시간을 `chrony` 같은 방식으로 동기화한다.
3. `ROS_DOMAIN_ID`를 두 장비에서 동일하게 맞춘다.
4. 같은 RMW 구현을 쓰는 것이 좋다. 예: 두 장비 모두 `rmw_fastrtps_cpp` 또는 둘 다 `rmw_cyclonedds_cpp`.
5. 카메라 raw topic은 PC가 구독하지 않게 한다.
6. 실제 하드웨어 운용 때는 `board1_simulator`, `mock_task_servers`, `dummy_servers`를 실행하지 않는다.

## 최종 추천

가장 추천하는 최종 구조는 아래와 같다.

```text
Raspberry Pi 5
  - camera / perception
  - Nav2 / localization / base control
  - mission_manager
  - vicpinky_nav_adapter
  - vicpinky_interfaces
  - vicpinky_gui 선택

LAN
  - /mission/*
  - /nav/go_to 결과
  - /arm/* RunTask
  - /joint_states, /arm_board/status_log 같은 상태 topic

PC
  - arm_task_server
  - arm_can_bridge
  - roscue_arm_description
  - roscue_arm_moveit_config
  - vicpinky_interfaces
  - vicpinky_gui 선택
  - USB2CANFD can0

STM32
  - Board1: arm_joint_1 ~ arm_joint_4
  - Board2: base_joint
  - Board3: gripper 9축
```

이렇게 두면 카메라/주행의 빠른 루프는 Raspberry Pi 5 안에서 닫히고, 팔/그리퍼의 빠른 CAN 제어 루프는 PC 안에서 닫힌다. LAN에는 미션 goal, semantic arm task, 상태 피드백 위주로만 흐르기 때문에 전체 지연과 네트워크 부하를 가장 낮게 유지할 수 있다.
