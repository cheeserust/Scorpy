# 서버 수동조작 Goal V3 안정화 작업 프롬프트

아래 요구사항에 따라 ROS2 중앙 서버/Raspberry Pi의 웹 수동조작 및
SocketCAN 송수신 코드를 조사하고 수정하라. 단순 분석이나 예시 코드만
제공하지 말고, 실제 저장소 코드에 구현하고 테스트까지 실행하라.

기존 사용자 변경을 보존하라. 작업 전에 `git status`와 diff를 확인하고,
관련 없는 파일을 되돌리거나 전면 포맷팅하지 마라. C/C++ 코드를 수정할
경우 숫자 상수 뒤에 `u` 또는 `U` 접미사를 새로 붙이지 마라.

## 1. 이번 작업의 핵심 목적

현재 웹 수동조작에서 다음 증상이 발생한다.

- 서버가 Board1 READY를 기다리다 최대 3회 retry 후 실패한다.
- 같은 수동조작을 계속 요청하면 가끔만 성공한다.
- 웹 요청 또는 화면 갱신 지연이 100ms 이상일 수 있다.
- READY ACK가 유실된 뒤 재전송에 대한 DUPLICATE ACK를 서버가 정상 복구로
  처리하지 못할 가능성이 있다.
- 이전 goal이 READY 또는 MOVING 상태로 남아 있는데 새 goal을 보내 BUSY가
  반복될 가능성이 있다.

서버의 목표는 다음과 같다.

1. ACK 결과를 정확히 해석하여 정상 복구 가능한 상황을 timeout 실패로
   처리하지 않는다.
2. 웹 지연과 CAN handshake timeout을 분리한다.
3. 수동조작 입력이 연속으로 들어와도 여러 goal이 동시에 CAN으로 섞이지
   않게 한다.
4. 실패 원인을 generic `retry 3 times` 하나로 숨기지 않는다.
5. timeout, BUSY 또는 CANCEL 때문에 자동으로 모터 Disable 명령을 보내지
   않는다.

## 2. 실제 펌웨어 상태

STM32 Board1 펌웨어에는 다음 안정화가 적용됐다.

- MCP2515 SPI2 SCK를 약 750kHz에서 약 3MHz로 변경했다.
- ACK 소프트웨어 큐 크기를 8에서 32로 확대했다.
- 같은 goal/result/mask/duration의 반복 DUPLICATE 또는 BUSY ACK는 큐에서
  병합한다.
- ACK 큐가 가득 찬 경우 반복 DUPLICATE/BUSY보다 READY, STARTED,
  STAGING_TIMEOUT, CANCELLED 같은 ACK를 우선 보존하도록 개선했다.
- MCP2515 RX overflow가 발생했을 때 컨트롤러 전체를 즉시 reset하지 않고
  overflow flag만 정리한다. 이미 유실된 frame은 서버가 전체 batch retry로
  복구해야 한다.

이 변경으로 CAN wire protocol은 전혀 바뀌지 않았다. 서버는 ACK가 move
frame마다 반드시 하나씩 올 것이라고 가정하면 안 된다. ACK 개수를 세지
말고 goal ID, result, mask, duration을 상태 증거로 사용하라.

## 3. 기존 V3 wire protocol 유지

Board1과 Board2는 최종 절대 관절각 하나와 duration 하나를 받는다. 구
trajectory point queue, Queue Free credit, preload/refill, point sequence,
Byte7 5ms tick 형식을 섞지 마라.

### Goal frame

Board1 Move CAN ID는 `0x101`, Board2 Move CAN ID는 `0x102`, DLC는 항상
8이다.

```text
Byte0    0x90 | local_motor_id
Byte1~4 int32 absolute target, little-endian, 단위 0.01도
Byte5    uint8 goal_id
Byte6~7 uint16 duration_ms, little-endian, 범위 1..65535
```

Byte0 의미:

```text
bit7 0x80 Execute = 1
bit6 0x40 Relative = 0
bit5 0x20 Step Mode = 0
bit4 0x10 Goal V3 = 1
bit3~0 local motor ID
```

Board1은 local motor 0, 1, 2, 3의 네 frame을 사용하고, Board2는 local
motor 0의 한 frame을 사용한다. 다섯 frame은 모두 같은 goal ID와 같은
duration을 사용해야 한다.

### Joint mapping

joint 배열 index를 추측하지 말고 joint name 기반으로 매핑하라.

| Joint | Board | CAN ID | Local motor |
|---|---:|---:|---:|
| `base_joint` | 1 | `0x101` | 3 |
| `arm_joint_1` | 1 | `0x101` | 0 |
| `arm_joint_2` | 1 | `0x101` | 1 |
| `arm_joint_3` | 1 | `0x101` | 2 |
| `arm_joint_4` | 2 | `0x102` | 0 |

Board1 송신 순서는 local motor `0 -> 1 -> 2 -> 3`으로 고정한다.

각도 변환:

```text
target_raw = nearest_integer(position_rad * 180 / pi * 100)
```

signed int32 little-endian으로 pack한다. 범위를 벗어나면 clamp하지 말고
goal을 거절한다.

| Joint | 최소 | 최대 |
|---|---:|---:|
| `arm_joint_1` | -86.50도 | 90.00도 |
| `arm_joint_2` | -78.10도 | 80.00도 |
| `arm_joint_3` | -91.50도 | 90.00도 |
| `base_joint` | -90.00도 | 180.00도 |
| `arm_joint_4` | -90.00도 | 90.00도 |

### Control frame

Control CAN ID는 공통 `0x040`, DLC는 8이다.

```text
Byte0 1=START, 2=CANCEL
Byte1 goal_id
Byte2~7 반드시 0
```

### ACK/NACK

Board1 ACK ID는 `0x401`, Board2 ACK ID는 `0x402`, DLC는 8이다.

```text
Byte0 protocol_version = 3
Byte1 result
Byte2 goal_id
Byte3 received_axis_mask
Byte4 state snapshot
Byte5 reserved = 0
Byte6~7 duration_ms echo, little-endian
```

Result 값:

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

### Status

Board1 Status는 `0x201`, Board2 Status는 `0x202`, DLC는 8이다.

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

Byte5는 Queue Free 개수가 아니다.

```text
1 = 새 goal을 받을 수 있음
0 = partial, READY 또는 MOVING goal이 있음
```

Status Byte5를 31, 32 또는 124 capacity로 해석하거나 4로 곱하고 나누는
구 코드를 제거하라. 값이 0 또는 1이 아니면 protocol mismatch로 처리하라.

State 값:

```text
0 INIT
1 IDLE
2 HOMING
3 MOVING
4 ERROR
5 ESTOP
6 DISABLED
```

## 4. 단일 CAN writer와 원자적 batch 전송

웹 callback, ROS callback, timer가 SocketCAN socket에 직접 쓰지 못하게
하라. 모든 송신은 하나의 writer/worker와 하나의 application queue를
통과해야 한다.

한 goal의 전송 단위:

1. Board1 `0x101` motor 0 frame
2. Board1 `0x101` motor 1 frame
3. Board1 `0x101` motor 2 frame
4. Board1 `0x101` motor 3 frame
5. Board2 `0x102` motor 0 frame

Board1 네 frame을 하나의 application batch로 연속 송신하라. 축마다 별도
웹 요청, 별도 callback 또는 100ms timer를 사용하지 마라. Board1 펌웨어는
첫 축을 받은 시점부터 100ms 안에 네 축을 모두 받아야 한다.

각 SocketCAN `send()` 반환값이 정확한 CAN frame 크기인지 확인하라.
`ENOBUFS`, short write 또는 송신 오류가 발생한 frame을 성공 처리하지 말고
명시적으로 기록하고 재시도하라.

## 5. timeout 측정 기준

웹사이트에서 버튼을 누른 시각부터 READY timeout을 측정하지 마라.

다음 타임스탬프를 분리하라.

```text
T0 웹 입력 생성
T1 서버가 웹 입력 수신
T2 Board1 batch 송신 시작
T3 Board1 네 frame 송신 완료
T4 Board1 READY 또는 full-mask DUPLICATE 수신
T5 Board2 READY 또는 full-mask DUPLICATE 수신
T6 START 송신
T7 양쪽 STARTED 수신
```

READY timeout은 해당 board의 CAN frame을 성공적으로 모두 송신한 직후부터
측정하라. 웹 전달 지연 `T1-T0`과 화면 표시 지연은 CAN READY timeout에
포함하지 마라.

초기 READY timeout은 250ms를 사용한다. 운영 환경에서 계측 결과가
250ms에 너무 가깝다면 500ms까지 조정할 수 있지만, 먼저 위 타임스탬프를
로그하고 원인을 확인하라. 서버 timeout만 늘려도 Board1 네 frame 사이가
100ms를 넘는 문제는 해결되지 않는다.

Board1 READY timeout 시 같은 goal ID, 같은 duration, 같은 네 payload를
전체 batch로 재전송한다. 축 하나만 골라 보내지 마라. 최대 staging attempt는
3회다.

## 6. ACK 처리 규칙

ACK는 반드시 CAN ID, protocol version, goal ID, duration 및 mask를 함께
검증한다. 다른 goal의 늦은 ACK는 현재 goal 상태를 바꾸지 말고 stale ACK로
로그한다.

### READY

다음을 board-ready로 처리한다.

```text
Board1: result=READY이고 mask=0x0F
Board1: result=DUPLICATE이고 mask=0x0F
Board2: result=READY이고 mask=0x01
Board2: result=DUPLICATE이고 mask=0x01
```

`DUPLICATE + full mask`는 최초 READY ACK가 유실됐지만 펌웨어에는 목표가
완전히 staging된 상태라는 뜻이다. 이를 retry 실패로 처리하지 마라.
DUPLICATE의 mask가 full mask보다 작으면 READY가 아니다.

### BUSY

BUSY를 READY timeout으로 바꾸거나 같은 새 goal을 3회 자동 재전송하지
마라. 다른 goal이 partial, READY 또는 MOVING 상태라는 의미다.

- 서버가 소유한 현재 goal이 실행 중이면 완료를 기다리거나 명시적인
  CANCEL 절차를 수행한다.
- 서버 재시작 후 소유자를 알 수 없는 BUSY라면 상태를 로그하고 무조건
  임의 goal ID로 CANCEL하지 마라. capability probe 및 복구 정책을 따른다.
- Status Byte5가 0인지, state가 IDLE/READY 상당 상태인지 MOVING인지 함께
  기록한다.

### STAGING_TIMEOUT

Board1 result가 STAGING_TIMEOUT이면 Byte3 mask에 실제로 수신된 local motor가
표시된다. 누락 motor ID는 다음처럼 계산해 로그한다.

```text
missing_mask = 0x0F & ~received_mask
```

partial staging은 펌웨어에서 폐기됐으므로 같은 goal ID와 duration으로
Board1 네 frame 전체를 다시 보낸다. Board2 staging 상태와 전체 goal의
일관성도 확인한다.

### INVALID와 CONFLICT

INVALID 또는 CONFLICT를 timeout retry로 덮지 마라. 즉시 최종 실패 원인으로
보고한다.

- INVALID: DLC, flags, relative, duration, target range, enable/homing/error
  상태 또는 잘못된 START 가능성
- CONFLICT: 같은 goal ID에서 duration/target 불일치, tombstone ID 재사용
  또는 goal lifecycle 충돌 가능성

동일 payload를 무한 재전송하지 마라. 양쪽 보드 중 한쪽만 staging됐을 수
있으므로 서버가 소유한 goal에 한해서 coordinated CANCEL로 정리한다.

### STARTED와 CANCELLED

STARTED와 CANCELLED도 Board1과 Board2 양쪽에서 해당 goal ID로 확인해야
한다. 한쪽 응답만 받고 전체 성공으로 처리하지 마라.

## 7. 정상 goal 상태 머신

서버에 전역 active goal은 최대 하나만 존재해야 한다.

```text
IDLE
  -> SENDING
  -> WAITING_READY
  -> STARTING
  -> MOVING
  -> COMPLETED
  -> IDLE
```

오류/취소 경로:

```text
SENDING 또는 WAITING_READY 또는 MOVING
  -> CANCELLING
  -> 양쪽 CANCELLED 확인
  -> IDLE
```

정상 실행 순서:

1. 새 uint8 goal ID 할당
2. Board1 네 frame 연속 송신
3. Board2 한 frame 송신
4. Board1 READY 또는 full-mask DUPLICATE 확인
5. Board2 READY 또는 full-mask DUPLICATE 확인
6. START `01 <goal_id> 00 00 00 00 00 00` 한 번 송신
7. Board1과 Board2 양쪽 STARTED 확인
8. Status 기반으로 완료 대기

READY 전에 START를 보내지 마라. `0x040`은 `0x101/0x102`보다 CAN arbitration
우선순위가 높으므로 ACK를 기다리지 않고 START를 enqueue하면 START가 goal
frame보다 먼저 도착할 수 있다.

goal ID는 goal마다 증가시키고 Board1/Board2에 동일하게 사용한다. 완료 또는
CANCEL 후 다음 goal은 다른 ID를 사용한다. CANCEL한 ID를 즉시 재사용하지
마라.

## 8. Software powered-hold E-stop

E-stop은 CAN ID `0x001`, DLC 8, payload
`01 00 00 00 00 00 00 00`을 사용한다. 별도 Resume 명령은 없다.

E-stop 버튼을 누르면 단일 CAN writer에서 다음 순서로 처리하라.

1. application queue에서 아직 송신하지 않은 goal/START/CANCEL 제거
2. E-stop frame을 최우선으로 broadcast
3. active goal을 `ABORTED_BY_ESTOP`으로 종료
4. E-stop 이전 goal의 늦은 READY/STARTED ACK를 stale ACK로 무시
5. 양쪽 Status가 `STATE_ESTOP`인지 확인

펌웨어는 E-stop 시 STEP과 모든 작업을 정지하지만 기존 motor enable 상태는
유지한다. enabled였다면 holding torque가 유지되고 disabled였다면 자동으로
enable되지 않는다. Status는 `state=ESTOP`, `goal_slot_free=0`, moving=0,
ready=0을 보고하며 Byte6 enabled는 실제 모터 EN 상태다.

E-stop은 다음 명령 중 하나로 해제한다.

```text
0x010 Enable=1: E-stop/error 해제 + motor enable
0x030 Clear Error: E-stop/error 해제 + 기존 motor enable 상태 유지
```

Enable 또는 Clear 후 취소된 goal을 자동 재전송하거나 START하지 마라. 새 사용자
명령으로 새 goal ID를 할당해야 한다. `0x010 Enable=0`은 관리자/사용자의 명시적
Disable 버튼에서만 보낸다. E-stop 자체, timeout, retry exhaustion, BUSY,
INVALID, exception cleanup 또는 웹 연결 종료에서는 Disable을 보내지 마라.

이 기능은 안전 인증된 STO가 아닌 팀 프로젝트용 software powered hold다.
엔코더 없는 stepper는 즉시 정지 중 탈조할 수 있으므로 position feedback이
명령 기반 추정값이라는 경고를 UI/운영 문서에 표시하라.

## 9. 웹 수동조작 정책

웹에서 연속 입력이 들어와도 입력 하나마다 동시 goal task를 생성하지 마라.
권장 정책은 `latest target wins`다.

1. active goal이 없으면 최신 입력으로 새 goal을 시작한다.
2. active goal과 완전히 같은 target이면 중복 goal을 만들지 않는다.
3. 새 target이 들어오고 기존 goal이 SENDING, WAITING_READY, STARTING 또는
   MOVING이면 기존 goal에 coordinated CANCEL을 수행한다.
4. CANCEL 진행 중 들어온 중간 입력은 CAN으로 보내지 않고 메모리의
   `pending_latest_target` 하나만 갱신한다.
5. Board1/Board2 양쪽 CANCELLED를 확인한 뒤 가장 최신 target 하나만 새로운
   goal ID로 전송한다.
6. CANCEL/START/goal batch가 서로 다른 callback에서 교차 송신되지 않게
   단일 writer에서 직렬화한다.
7. 웹 입력 빈도가 높다면 debounce 또는 rate limit을 적용하되, 마지막 입력은
   버리지 않는다.

수동조작 명령의 timeout, retry 실패, BUSY, INVALID, CONFLICT 또는 CANCEL은
모터 Disable과 다른 의미다. 이 상황에서 다음 Disable frame을 자동으로
보내지 마라.

```text
CAN ID 0x010
payload 00 00 00 00 00 00 00 00
```

Disable은 사용자 명시적 Disable 요청이나 별도로 승인된 안전 정책에서만
보낸다. 통신 실패 처리 코드, 웹 연결 종료 handler, manual mode 종료 handler,
exception cleanup, retry exhaustion 경로에서 `0x010 Byte0=0`을 보내는 코드가
있는지 조사하고 제거하거나 명시적 정책으로 분리하라.

## 10. CANCEL 규칙

서버가 소유한 현재 goal을 취소할 때 다음 순서를 지켜라.

1. application TX queue에서 아직 전송되지 않은 해당 goal frame 제거
2. 단일 writer에서 진행 중인 해당 goal batch 종료 여부 확인
3. CANCEL `02 <goal_id> 00 00 00 00 00 00` 송신
4. Board1 `0x401 CANCELLED` 확인
5. Board2 `0x402 CANCELLED` 확인
6. 다음 goal에 다른 ID 할당

CANCEL timeout을 모터 Disable로 대체하지 마라. CANCEL 응답이 한쪽에서만
오면 양쪽 상태와 마지막 ACK를 포함한 coordinated cancel failure로 보고한다.

## 11. 완료 판정

요청 duration만 지나면 완료라고 판단하거나 `duration + grace`가 지났다는
이유만으로 CANCEL하지 마라. 펌웨어의 속도/가속도 제한 때문에 실제 이동은
요청 duration보다 길어질 수 있다.

완료 조건:

- 해당 goal에서 Board1/Board2 STARTED를 받음
- Board1 state=IDLE
- Board1 네 축 moving=0, target reached=1
- Board2 state=IDLE
- Board2 axis0 moving=0, target reached=1
- 양쪽 error=0
- 양쪽 goal_slot_free=1
- 양쪽 status heartbeat 정상

Status가 정상적으로 MOVING을 보고하면 duration을 넘겨도 기다린다. Status
heartbeat가 1초 이상 끊기면 motion timeout과 구분되는 통신 오류로 처리한다.

## 12. 필수 진단 로그

`READY timeout after three staging attempts` 한 줄만 출력하지 마라. goal마다
다음 정보를 구조화해서 남겨라.

- 웹 입력 수신 시각과 request/session ID
- goal ID, duration, target joint name과 값
- T0~T7 타임스탬프와 각 구간 소요시간
- attempt 번호
- 실제 송신한 모든 `0x101/0x102/0x040` payload
- 각 `send()` 성공, short write, errno, ENOBUFS 여부
- 수신한 모든 `0x401/0x402`의 version/result/goal/mask/state/duration
- DUPLICATE full-mask를 READY로 승격했는지
- 마지막 STAGING_TIMEOUT received mask와 누락 motor ID
- 마지막 BUSY/INVALID/CONFLICT 원문
- 최신 `0x201/0x202` state/error/goal_slot_free/enabled/sequence/age
- SocketCAN interface state, TX/RX error counter 및 drop counter
- active goal 상태와 pending latest target 존재 여부
- Disable `0x010 Byte0=0`을 송신한 경우 호출 원인과 사용자 요청 ID

서버 내부 timeout 예외가 발생하더라도 이미 수신한 BUSY, INVALID, CONFLICT,
STAGING_TIMEOUT을 generic timeout으로 덮어쓰지 마라.

## 13. 저장소에서 반드시 조사할 항목

수정 전에 실제 파일과 함수 위치를 찾아 작업 보고에 포함하라.

1. 웹 수동조작 HTTP/WebSocket 진입점
2. 수동조작 debounce/throttle/timer
3. timeout 시작 지점
4. Board1 네 축 frame 생성 및 송신 경로
5. SocketCAN socket을 직접 쓰는 모든 thread/callback
6. ACK reader와 goal별 waiter/future/event
7. result=2 DUPLICATE 처리
8. BUSY/INVALID/CONFLICT를 timeout으로 변환하는 catch/fallback
9. retry 3회 구현
10. active goal과 goal ID allocator
11. CANCEL 및 pending TX 제거 구현
12. Status Byte5 parser
13. 동작 완료 판정
14. timeout/error/web disconnect 시 자동 Disable 송신 여부
15. Board3 공용 packer 사용 여부

Board3 `0x103/0x203/0x303` 프로토콜과 기존 packer는 변경하지 마라.
Board1/Board2 V3 전용 packer와 상태 머신만 수정하라.

## 14. 필수 테스트

최소한 다음 테스트를 추가하거나 갱신하고 모두 실행하라.

1. Board1 네 frame이 local motor 0,1,2,3 순서로 한 batch에서 생성됨
2. Board2 한 frame이 같은 goal ID와 duration으로 생성됨
3. 음수 각도 int32 little-endian packing
4. duration 1, 5000, 65535 packing
5. duration 0과 65535 초과 거절
6. Board1 READY mask=0x0F 성공
7. Board1 DUPLICATE mask=0x0F를 READY로 처리
8. Board1 DUPLICATE partial mask를 READY로 처리하지 않음
9. Board2 DUPLICATE mask=0x01을 READY로 처리
10. BUSY를 READY timeout retry로 변환하지 않음
11. INVALID/CONFLICT 즉시 실패
12. STAGING_TIMEOUT mask로 누락 motor ID 계산
13. STAGING_TIMEOUT 뒤 Board1 네 frame 전체 retry
14. timeout clock이 웹 입력 시점이 아니라 CAN batch 송신 완료 후 시작됨
15. READY 전 START가 송신되지 않음
16. 양쪽 READY 후 START가 정확히 한 번 송신됨
17. 여러 웹 입력이 동시에 SocketCAN write를 호출하지 않음
18. CANCEL 중 입력 여러 개가 들어오면 마지막 target 하나만 실행됨
19. CANCEL 뒤 새 goal ID 사용
20. retry/BUSY/INVALID/CANCEL 경로에서 Disable frame을 보내지 않음
21. 사용자 명시적 Disable 요청에서만 `0x010 Byte0=0` 송신
22. Status Byte5를 0/1 goal slot로 해석
23. duration을 지났어도 정상 MOVING heartbeat이면 취소하지 않음
24. Board1/Board2 양쪽 완료 조건을 만족해야 완료 처리
25. ACK 개수를 frame 개수와 동일하다고 가정하지 않음
26. stale goal ID ACK가 현재 goal 상태를 바꾸지 않음
27. vcan에서 READY 유실 후 full-mask DUPLICATE 복구 시나리오
28. vcan에서 BUSY, STAGING_TIMEOUT, CANCEL 경합 시나리오
29. Board3 기존 payload snapshot 불변
30. E-stop이 pending goal/START/CANCEL을 제거하고 active goal을 abort함
31. E-stop에서 Disable frame을 보내지 않고 enabled status를 그대로 해석함
32. Enable 또는 Clear Error가 E-stop을 해제하지만 기존 goal을 재시작하지 않음
33. Disable frame은 명시적 관리자/사용자 요청에서만 송신됨

## 15. 작업 완료 보고 형식

구현과 검증이 끝나면 다음을 보고하라.

1. 수정한 파일과 각 변경 목적
2. 기존 장애의 실제 원인
3. 웹 입력부터 CAN ACK까지의 timeout 기준 변경
4. 단일 CAN writer 위치와 batch 직렬화 방식
5. ACK result별 최종 처리 표
6. DUPLICATE full-mask 복구 구현 위치
7. 수동조작 latest-target/CANCEL 상태 머신
8. 자동 Disable 제거 또는 부재 확인 근거
9. 추가한 진단 로그 예시
10. 실행한 테스트 명령과 결과
11. 남은 위험과 실제 CAN 장비에서 확인할 항목

코드 수정 없이 설명만 제공하지 마라. 테스트가 실패하면 실패 내용을 숨기지
말고 원인과 남은 작업을 정확히 보고하라.
