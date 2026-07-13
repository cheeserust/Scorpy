# 서버 저장소 담당 GPT에게 그대로 전달할 프롬프트

당신은 ROS2 중앙 서버/Raspberry Pi SocketCAN 송신 코드를 수정하는 담당자다. MoveIt2 `JointTrajectory` waypoint streaming은 더 이상 사용하지 않는다. 서버는 최종 관절각 5개와 duration 하나만 Board1/Board2에 보내고, 각 보드가 내부에서 사다리꼴 가감속하여 최종 위치로 이동한다.

아래 계약은 실제 펌웨어에 구현된 V3 wire protocol이다. 기존 Queue Free, 32/124 credit, point sequence, preload/refill 또는 V2 코드를 일부라도 섞지 마라.

## 1. 수정 전 저장소 조사

저장소 전체를 탐색해 다음 위치를 실제 파일명과 함수명으로 먼저 보고한 후 수정한다.

1. MoveIt2 `JointTrajectory`/trajectory action을 받는 진입점
2. 20ms/40ms timer로 trajectory point를 반복 송신하는 코드
3. global joint와 board/local motor ID 매핑
4. `pack_position_command()` 및 Board1/2/3 packer
5. SocketCAN socket을 직접 쓰는 모든 callback/thread/timer
6. 송신 큐, retry, timeout, cancel 처리
7. `0x201/0x202` Status parser와 `0x301/0x302` Position parser
8. Queue Free를 31/32/124로 해석하거나 축 수로 곱하고 나누는 코드
9. `arm_task_server_node.py`의 `duration + grace` 상위 취소 로직
10. `send_test_trajectory` current→target→current 테스트
11. `roscue_arm_pick` duration/queue 설정 validator
12. Board3 `0x103`을 사용하는 공용 packer와 회귀 테스트

`git status`와 현재 diff를 먼저 확인하고 사용자 소유 변경을 되돌리지 마라. 관련 없는 리팩터링과 formatting 전면 변경은 금지한다.

Board3 `0x103/0x203/0x303` 프로토콜은 절대 변경하지 않는다. 공용 packer를 V3로 바꾸지 말고 Board1/Board2 전용 V3 packer를 만든다.

## 2. 새 서버 입력 모델

서버의 모션 입력은 다음 의미의 단일 goal이다.

```text
target joint angles: base_joint, arm_joint_1, arm_joint_2,
                     arm_joint_3, arm_joint_4
duration_ms: uint16 의미, 1..65535ms
```

- MoveIt2가 만든 중간 waypoint를 읽거나 변환하거나 송신하지 않는다.
- 기존 상위 task API가 최종 joint angle과 duration을 이미 제공하면 그 API를 유지한다.
- 기존 API가 `JointTrajectory`만 제공한다면 서버 저장소 구조를 조사해 최소 범위의 direct joint-goal API로 분리한다. MoveIt trajectory의 마지막 point를 몰래 재사용하는 임시 호환 경로를 만들지 않는다.
- 새로운 goal은 현재 goal이 완전히 끝나거나 CANCEL된 뒤에만 받는다.
- 동시에 여러 goal을 queueing하지 않는다. 두 번째 goal은 BUSY로 거절하거나 상위 호출자가 완료 후 다시 요청하게 한다.

최종각 직행은 MoveIt 충돌 회피 경로를 보존하지 않는다. 이 변경 사실을 API 문서와 운영 경고에 명시한다. 서버에서 별도 joint/workspace 안전 검증이 있다면 유지하되, MoveIt waypoint 송신은 제거한다.

## 3. 축 매핑

joint 배열 순서를 추측하지 말고 joint name 기반 명시적 매핑을 사용한다.

| Global | Joint name | Board | Move CAN ID | Local motor |
|---:|---|---:|---:|---:|
| 0 | `base_joint` | 1 | `0x101` | 3 |
| 1 | `arm_joint_1` | 1 | `0x101` | 0 |
| 2 | `arm_joint_2` | 1 | `0x101` | 1 |
| 3 | `arm_joint_3` | 1 | `0x101` | 2 |
| 4 | `arm_joint_4` | 2 | `0x102` | 0 |

Board1 송신 순서는 local motor `0,1,2,3`으로 고정한다.

```text
arm_joint_1 -> 0
arm_joint_2 -> 1
arm_joint_3 -> 2
base_joint  -> 3
```

## 4. V3 Goal frame

Board1 Move ID `0x101`, Board2 Move ID `0x102`, DLC 8:

```text
Byte0    flags + local_motor_id
Byte1~4 int32 absolute target, little-endian
Byte5    uint8 goal_id
Byte6~7 uint16 duration_ms, little-endian
```

Byte0:

```text
bit7 0x80 Execute = 1
bit6 0x40 Relative = 0; V3에서 relative는 금지
bit5 0x20 Step Mode = 0; 일반 서버는 각도 모드만 사용
bit4 0x10 Goal V3 = 1
bit3~0 local motor ID
```

일반 명령은 `byte0 = 0x90 | motor_id`다. Board1 네 frame은 `0x90,0x91,0x92,0x93`, Board2는 `0x90`이다.

Byte5~6을 speed나 duration tick으로 해석하는 구 코드를 제거한다. Byte7 단독 5ms tick도 더 이상 존재하지 않는다. duration은 Byte6~7의 실제 밀리초 uint16이다.

각도 단위는 `0.01도`다.

```text
target_raw = nearest_integer(position_rad * 180 / pi * 100)
```

표준 signed int32 little-endian packing을 사용한다. 범위를 벗어나면 clamp하지 말고 goal 생성 자체를 실패시킨다.

펌웨어 실제 제한:

| Joint | 최소 | 최대 |
|---|---:|---:|
| arm_joint_1 / Board1 motor0 | -86.50° | 90.00° |
| arm_joint_2 / Board1 motor1 | -78.10° | 80.00° |
| arm_joint_3 / Board1 motor2 | -91.50° | 90.00° |
| base_joint / Board1 motor3 | -90.00° | 180.00° |
| arm_joint_4 / Board2 motor0 | -90.00° | 90.00° |

duration 범위는 `1..65535ms`다. 0, 음수, 65535 초과를 clamp/wrap하지 말고 거절한다. 다섯 frame 모두 같은 goal ID와 duration을 사용한다.

## 5. goal ID

- uint8 goal ID를 goal마다 증가시킨다.
- Board1과 Board2에 동일한 goal ID를 사용한다.
- 완료 또는 CANCEL 후 다음 goal은 반드시 다른 ID를 사용한다.
- 펌웨어는 마지막 CANCEL ID를 tombstone 처리하므로 같은 ID를 즉시 재사용하면 CONFLICT가 난다.
- 완료된 goal frame이 늦게 재수신되면 DUPLICATE가 날 수 있다.

## 6. 정확한 payload 예시

```text
goal_id = 0x2A
duration_ms = 5000 = 0x1388
Board1 motor0 = 30.00°  = 3000
Board1 motor1 = -15.50° = -1550
Board1 motor2 = 0.00°   = 0
Board1 motor3 = 90.00°  = 9000
Board2 motor0 = 10.00°  = 1000
```

정확한 frame:

```text
CAN 0x101: 90 B8 0B 00 00 2A 88 13
CAN 0x101: 91 F2 F9 FF FF 2A 88 13
CAN 0x101: 92 00 00 00 00 2A 88 13
CAN 0x101: 93 28 23 00 00 2A 88 13
CAN 0x102: 90 E8 03 00 00 2A 88 13
```

## 7. frame 순서와 송신 직렬화

Board1 펌웨어는 네 축 frame을 motor 순서와 무관하게 같은 goal ID로 조립한다. 예를 들어 `motor2→motor0→motor3→motor1`도 정상 READY가 된다. 그러나 서버는 디버깅과 일관성을 위해 계속 `0→1→2→3`으로 보낸다.

- Board1 네 frame은 하나의 application batch로 단일 CAN writer가 연속 enqueue한다.
- 그 뒤 Board2 한 frame을 enqueue한다.
- 여러 ROS callback/timer가 직접 CAN socket을 쓰지 못하게 한다.
- 7ms 또는 40ms 고정 간격은 필요 없다.
- 각 `send()` 반환값이 정확한 CAN frame 크기인지 검사한다.
- 송신 실패/ENOBUFS는 명시적으로 retry하고 성공하지 않은 frame을 성공 처리하지 않는다.

Board1은 100ms 안에 네 축이 모두 오지 않으면 partial staging을 폐기하고 STAGING_TIMEOUT ACK를 보낸다. READY가 250ms 안에 없으면 같은 goal ID, 같은 duration으로 Board1 네 frame 전체를 재전송한다. 최대 3회 후 통신 실패 처리한다.

`READY timeout after three staging attempts`를 generic 문자열만으로 보고하지 마라. 실패 로그에는 각 attempt별로 다음을 반드시 포함한다.

- 실제 송신한 네 `0x101` payload 전체와 송신 성공/실패
- goal ID와 duration
- 수신한 모든 `0x401` ACK의 version/result/mask/state/duration
- 마지막 STAGING_TIMEOUT mask
- `0x201` state/error/goal_slot_free/status sequence/age
- SocketCAN interface와 TX/RX error counter

ACK result가 INVALID/CONFLICT/BUSY였는데도 이를 무시하고 READY timeout으로 바꾸지 마라. 해당 NACK를 즉시 최종 원인으로 보고한다. STAGING_TIMEOUT mask가 `0x0F` 미만이면 누락 motor ID를 mask에서 계산해 로그에 표시한다.

Board2 READY가 유실됐을 때 같은 한 frame을 재전송하면 DUPLICATE를 받는다. mask가 Board1 `0x0F`, Board2 `0x01`인 DUPLICATE는 READY ACK 유실 복구로 취급할 수 있다.

## 8. START/CANCEL

Control CAN ID는 공통 `0x040`, DLC 8이다.

```text
Byte0 1=START, 2=CANCEL
Byte1 goal_id
Byte2~7 반드시 0
```

정상 실행 순서:

1. Board1 네 goal frame 송신
2. Board2 한 goal frame 송신
3. Board1 READY(mask `0x0F`) 확인
4. Board2 READY(mask `0x01`) 확인
5. START `01 <goal_id> 00 00 00 00 00 00` 한 번 broadcast
6. Board1과 Board2 양쪽 STARTED 확인
7. Status 기반 완료 대기

READY 전에 START를 보내지 않는다. Control ID `0x040`은 Move ID보다 CAN 우선순위가 높으므로 ACK를 기다리지 않고 바로 START를 enqueue하면 START가 goal frame보다 먼저 도착할 수 있다.

CANCEL 순서:

1. application 송신 큐에서 아직 안 보낸 해당 goal frame 제거
2. 단일 CAN writer에서 이전 송신 작업 종료 확인
3. `02 <goal_id> 00...00` 송신
4. Board1/Board2 양쪽 CANCELLED ACK 확인
5. 다음 goal은 다른 ID 사용

Software E-stop은 `0x001`, DLC 8, payload
`01 00 00 00 00 00 00 00`을 최우선 broadcast한다. 서버는 pending
goal/START/CANCEL을 제거하고 active goal을 abort하며, 이전 goal의 늦은 ACK를
무시한다. 펌웨어는 STEP과 작업을 정지하지만 기존 motor enable 상태를 유지한다.
`0x010 Enable=1` 또는 `0x030 Clear Error`가 E-stop을 해제하며 취소된 작업은
재시작하지 않는다. `0x010 Enable=0`은 명시적 관리자/사용자 Disable 요청에서만
보내고 E-stop, timeout, retry 실패, BUSY, INVALID, exception 또는 연결 종료
경로에서는 보내지 않는다. 이것은 안전 인증된 STO가 아닌 software powered
hold다.

서버 시작/CAN 재연결 후 V3 capability probe:

1. 실제 goal에 쓰지 않을 probe ID 선택
2. CANCEL frame 전송
3. `0x401`과 `0x402` 양쪽에서 version 3, result CANCELLED를 확인
4. 확인 전에는 V3 Move를 보내지 않음
5. probe ID는 이후 사용하지 않음

## 9. ACK/NACK

Board1 ACK ID `0x401`, Board2 ACK ID `0x402`, DLC 8:

```text
Byte0 protocol_version = 3
Byte1 result
Byte2 goal_id
Byte3 received_axis_mask
Byte4 state snapshot
Byte5 reserved = 0
Byte6~7 duration_ms echo, little-endian
```

Result:

```text
0 READY
1 STARTED
2 DUPLICATE
3 BUSY
4 STAGING_TIMEOUT
5 CONFLICT
6 CANCELLED
7 INVALID
```

처리 규칙:

- READY: 목표가 준비됐지만 아직 움직이지 않음.
- STARTED: START가 승인되고 이동 시작.
- DUPLICATE: 같은 goal/motor/value 재전송. full mask면 READY ACK 유실 복구 가능.
- BUSY: 다른 goal이 partial/ready/moving 상태. 새 goal을 queue하지 말고 현재 goal 완료 또는 CANCEL을 기다림.
- STAGING_TIMEOUT: Board1 네 축 일부 유실. mask를 로그하고 네 frame 전체 재전송.
- CONFLICT: 같은 goal ID에서 duration 또는 같은 motor target이 달라짐, 다른/tombstoned ID 충돌. 자동으로 값 하나만 이어 보내지 말고 원인을 확인한 뒤 전체 goal을 다시 시작.
- CANCELLED: goal 또는 capability probe 취소 완료.
- INVALID: DLC/flags/relative/duration/target 범위/START 상태 오류. 동일 payload 무한 retry 금지.

Move protocol 오류는 Status의 fatal `ERR_INVALID_CMD`가 아니라 ACK/NACK로 처리된다. 따라서 ACK parser를 반드시 구현한다.

## 10. Status: Queue Free가 완전히 없어짐

Board1 Status `0x201`, Board2 Status `0x202`, DLC 8 형식은 유지하지만 Byte5 의미가 변경됐다.

```text
Byte0 state
Byte1 error
Byte2 axis0/1 flags
Byte3 axis2/3 flags
Byte4 limit bits
Byte5 goal_slot_free
Byte6 enabled
Byte7 status sequence
```

`goal_slot_free`는 반드시 0 또는 1이다.

```text
1: 새 goal을 받을 수 있음
0: partial, READY 또는 MOVING goal이 있음
```

이 값은 Queue Free가 아니다. 다음 코드를 모두 제거한다.

- Queue Free 최대 31/32
- 32 point capacity
- Board1 4 frame credit
- 124 frame credit
- Queue Free를 4로 곱하거나 나누는 코드
- preload/refill threshold
- queue occupancy UI/metric

Status Byte5가 1보다 크면 구 firmware/parser 혼용으로 판단하고 V3 송신을 중단한다. 124를 clamp해서 계속하면 안 된다.

축 flag nibble:

```text
bit0 homed/position valid
bit1 ready
bit2 moving
bit3 target reached
```

Status parser의 aggregate mask 유효 범위는 다음과 같다.

```text
Board1 ready/moving/target_reached mask: 0x00..0x0F
Board2 ready/moving/target_reached mask: 0x00..0x01
```

`moving=255` 또는 `ready=255`는 펌웨어가 보낼 수 있는 정상 값이 아니다. 미초기화/default sentinel, signed/unsigned 변환, nibble 추출 실패 또는 구 status parser 혼용으로 판단하고 parser 오류로 기록한다. `0x201`의 Byte2/3에서 축별 nibble을 먼저 분리한 후 각 nibble의 bit2만 모아 moving mask를 만들고, bit1만 모아 ready mask를 만들어야 한다. position-valid mask나 `0xFF` default를 moving 값으로 재사용하지 마라.

같은 최신 Status frame에서 `state=ERROR`인데 ready mask가 `0x0F`라면 의미상 모순이다. 서로 다른 시점의 cached field를 합쳤거나 ready bit가 아닌 값을 출력했는지 확인한다. Status sequence(Byte7)별 snapshot을 원자적으로 갱신하고 한 로그에 서로 다른 frame의 필드를 섞지 마라.

State:

```text
0 INIT, 1 IDLE, 2 HOMING, 3 MOVING,
4 ERROR, 5 ESTOP, 6 DISABLED
```

완료 조건:

- 해당 goal에서 Board1/Board2 STARTED를 받음
- Board1 state IDLE, 네 축 moving=0, target reached=1
- Board2 state IDLE, axis0 moving=0, target reached=1
- 양쪽 error=0
- 양쪽 goal_slot_free=1
- 양쪽 status heartbeat 정상

펌웨어의 최대속도/가속도 안전 제한 때문에 요청 duration보다 실제 시간이 길어질 수 있다. `duration + grace`만으로 취소하지 않는다. 정상 MOVING heartbeat가 계속 오면 기다린다. Status가 1초 이상 끊기면 별도의 통신 오류로 처리한다.

## 11. Position feedback

Board1 `0x301`:

```text
Byte0~1 motor0 -> arm_joint_1/global1
Byte2~3 motor1 -> arm_joint_2/global2
Byte4~5 motor2 -> arm_joint_3/global3
Byte6~7 motor3 -> base_joint/global0
```

Board2 `0x302`:

```text
Byte0~1 motor0 -> arm_joint_4/global4
Byte2~7 reserved
```

각 값은 signed int16 little-endian, 0.01도다. CAN 슬롯을 global 순서로 그대로 복사하지 말고 역매핑한다.

## 12. 반드시 함께 제거/수정할 구 서버 경로

- MoveIt2 trajectory point callback과 주기 송신 timer
- 마지막 point 선택 또는 75개 waypoint 분할/streaming
- Queue Free 기반 preload/refill
- point sequence/trajectory ID V2 상태 머신
- Byte7 5ms duration tick
- Byte5~6 speed packing
- `arm_task_server_node.py` duration+grace 강제 취소
- `roscue_arm_pick` queue/tick validator
- Queue Full retry와 Clear Error 흐름

`send_test_trajectory`의 current→target→current는 한 queue/trajectory로 보내지 않는다. 다음 두 개의 독립 goal로 실행한다.

1. target 각도 + duration 전송, 완료 대기
2. original 각도 + duration 전송, 완료 대기

Board3 공용 `pack_position_command()`는 그대로 유지하고 Board1/Board2 V3 전용 함수를 별도로 만든다.

권장 내부 API 의미:

```text
send_board1_goal_v3(goal_id, duration_ms, local_targets[4])
send_board2_goal_v3(goal_id, duration_ms, target)
start_goal_v3(goal_id)
cancel_goal_v3(goal_id)
```

실제 함수명과 언어 스타일은 저장소 기존 구조를 따르되 의미와 wire bytes는 정확히 유지한다.

## 13. 필수 테스트

1. Board1 정확히 4 frame, Board2 정확히 1 frame 생성
2. Board1 local motor 0→1→2→3 매핑
3. 음수 int32 little-endian
4. duration 1, 5000, 65535ms packing
5. duration 0/65535 초과 거절
6. 다섯 frame 동일 goal ID와 duration
7. Byte5 goal ID, Byte6~7 duration 위치
8. 상대명령 bit가 항상 0
9. Board1 READY 전 START 금지
10. Board1/Board2 READY 후 START 한 번 송신
11. ACK result 0~7 parser
12. partial timeout 시 Board1 네 frame 전체 재전송
13. DUPLICATE full mask로 READY ACK 유실 복구
14. BUSY에서 새 goal queueing 금지
15. CANCEL 후 다른 goal ID 사용
16. capability probe 실패 시 Move 송신 차단
17. Status Byte5를 0/1 goal slot로 파싱
18. Status Byte5=32/124 수신 시 protocol mismatch
19. duration이 지나도 MOVING이면 실패 처리하지 않음
20. 양쪽 IDLE/target reached/slot free에서만 완료
21. `0x301/0x302` signed feedback 역매핑
22. current→target→current가 독립 goal 두 개로 실행
23. Board3 기존 payload snapshot 불변
24. vcan에서 예시 frame과 READY→STARTED→완료 흐름 검증
25. 여러 송신 thread가 동시에 CAN socket을 쓰지 않는지 검증
26. Board1 moving mask가 0x0F, Board2가 0x01을 넘으면 parser 오류로 거절
27. 하나의 status sequence에서 state/error/ready/moving/slot-free가 원자적으로 갱신되는지

## 14. 작업 결과 보고

완료 후 다음을 보고한다.

1. 변경 파일과 목적
2. 제거한 MoveIt/trajectory streaming 경로
3. 새 direct joint-goal 진입점
4. Board1/Board2 V3 packer 실제 함수
5. 단일 CAN writer와 송신 직렬화 위치
6. goal ID lifecycle
7. READY/STARTED/CANCELLED ACK 상태 머신
8. Status Byte5 Queue Free 제거 근거
9. duration+grace 제거 위치
10. Board3 불변 테스트 근거
11. 실행한 테스트 명령과 결과
12. 실제 CAN 장비에서 확인할 항목

기존 코드와 이 문서가 충돌하면 구 Queue/MoveIt 동작을 임의로 남기지 말고, Board1/Board2 경로를 이 V3 계약으로 완전히 교체한다.
