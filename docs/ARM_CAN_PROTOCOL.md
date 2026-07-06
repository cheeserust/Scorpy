# VicPinky Arm CAN Protocol

이 문서는 Board1, Board2, Board3가 함께 사용하는 최종 CAN 통합 기준이다.
배포 기준은 아래 3개 보드별 프로토콜 문서다.

```text
Board1_CAN_Protocol
Board2_CAN_Protocol
Board3_CAN_Protocol
```

단, Board3 `0x103`의 Byte5~6은 오늘 추가한 파지 부하 기능을 반영해
`Target Load`로 확장한다. 이 확장은 기존 8 byte frame 구조와 CAN ID를
바꾸지 않고, 원래 `Speed`로 예약/미사용이던 자리를 재정의한다.

---

## 1. 가장 중요한 원칙

```text
서버/RPi 내부에서는 board_id로 대상 보드를 구분할 수 있다.
하지만 실제 CAN frame payload에는 Board ID를 넣지 않는다.
보드 구분은 CAN ID로 한다.
```

따라서 아래 과거 방식은 사용하지 않는다.

```bash
# 사용 금지: payload에 Target Board를 넣는 과거 방식
cansend can0 010#01FF000000000000
cansend can0 010#0102000000000000
cansend can0 010#0103000000000000
cansend can0 020#02FF000000000000
cansend can0 020#03FF000000000000
cansend can0 030#02FF000000000000
cansend can0 030#03FF000000000000
```

최종 공통 명령은 아래처럼 broadcast payload를 사용한다.

```bash
cansend can0 010#0100000000000000  # 전체 Enable
cansend can0 010#0000000000000000  # 전체 Disable
cansend can0 020#FF00000000000000  # Board1+Board2 stepper homing
cansend can0 023#FF00000000000000  # Board3 gripper home posture
cansend can0 030#FF00000000000000  # 전체 Clear Error
cansend can0 001#0100000000000000  # 전체 ESTOP
```

---

## 2. Board 역할과 CAN ID

| Board | 역할 | Move CAN ID | Status CAN ID | Position Feedback CAN ID | Payload Motor ID |
|---|---|---:|---:|---:|---|
| Board1 | 팔 2~5축 stepper 4개 | `0x101` | `0x201` | `0x301` | `0~3` |
| Board2 | base 1축 stepper 1개 | `0x102` | `0x202` | `0x302` | `0` |
| Board3 | gripper servo 9개 | `0x103` | `0x203` | `0x303` | `0~8` |

MoveIt2 기준 joint mapping은 아래와 같다.

| MoveIt Joint | 실제 축 | Board | Local Motor ID |
|---|---|---:|---:|
| `base_joint` | 베이스 1축 | Board2 | `0` |
| `arm_joint_1` | 팔 2축 | Board1 | `0` |
| `arm_joint_2` | 팔 3축 | Board1 | `1` |
| `arm_joint_3` | 팔 4축 | Board1 | `2` |
| `arm_joint_4` | 팔 5축 | Board1 | `3` |
| `finger_1_base_joint` | finger 1 base | Board3 | `0` |
| `finger_1_middle_joint` | finger 1 middle | Board3 | `1` |
| `finger_1_tip_joint` | finger 1 tip | Board3 | `2` |
| `finger_2_base_joint` | finger 2 base | Board3 | `3` |
| `finger_2_middle_joint` | finger 2 middle | Board3 | `4` |
| `finger_2_tip_joint` | finger 2 tip | Board3 | `5` |
| `finger_3_base_joint` | finger 3 base | Board3 | `6` |
| `finger_3_middle_joint` | finger 3 middle | Board3 | `7` |
| `finger_3_tip_joint` | finger 3 tip | Board3 | `8` |

---

## 3. CAN ID 전체 목록

| CAN ID | 방향 | 용도 |
|---:|---|---|
| `0x001` | 서버/RPi → 전체 보드 | Emergency Stop |
| `0x010` | 서버/RPi → 전체 보드 | Enable / Disable broadcast |
| `0x020` | 서버/RPi → Board1+Board2 | Stepper homing broadcast |
| `0x023` | 서버/RPi → Board3 | Gripper home posture |
| `0x030` | 서버/RPi → 전체 보드 | Clear Error broadcast |
| `0x101` | 서버/RPi → Board1 | Board1 4축 trajectory frame |
| `0x102` | 서버/RPi → Board2 | Board2 base trajectory frame |
| `0x103` | 서버/RPi → Board3 | Board3 gripper servo frame |
| `0x201` | Board1 → 서버/RPi | Board1 status |
| `0x202` | Board2 → 서버/RPi | Board2 status |
| `0x203` | Board3 → 서버/RPi | Board3 status |
| `0x301` | Board1 → 서버/RPi | Board1 current position feedback |
| `0x302` | Board2 → 서버/RPi | Board2 current position feedback |
| `0x303` | Board3 → 서버/RPi | Board3 current position feedback |

---

## 4. 공통 제어 명령

### 4.1 ESTOP, CAN ID `0x001`

```text
CAN ID = 0x001
DLC = 8
```

| Byte | 필드 | 값 |
|---:|---|---:|
| 0 | ESTOP request | `1` |
| 1~7 | Reserved | `0` |

```bash
cansend can0 001#0100000000000000
```

수신 시 보드는 motion을 정지하고 driver/servo torque를 disable하며 queue와
staging buffer를 clear한다. ESTOP 해제는 Clear Error가 아니라
`0x010 Enable=1`에서 처리한다.

### 4.2 Enable / Disable, CAN ID `0x010`

```text
CAN ID = 0x010
DLC = 8
payload에 Target Board를 넣지 않는다.
```

| Byte | 필드 | 값 |
|---:|---|---|
| 0 | Enable | `0`: Disable, `1`: Enable |
| 1~7 | Reserved | `0` |

```bash
cansend can0 010#0100000000000000  # 전체 Enable
cansend can0 010#0000000000000000  # 전체 Disable
```

Board1, Board2, Board3는 `0x010`을 수신하면 항상 처리한다.

### 4.3 Stepper Homing, CAN ID `0x020`

```text
CAN ID = 0x020
DLC = 8
대상 = Board1 + Board2
Board3는 기본적으로 0x020을 무시한다.
payload에 Target Board를 넣지 않는다.
```

| Byte | 필드 | 값 |
|---:|---|---|
| 0 | Target Motor | `0xFF`: 전체 stepper homing |
| 1 | Homing Mode | `0` |
| 2~7 | Reserved | `0` |

```bash
cansend can0 020#FF00000000000000
```

처리 조건:

```text
enabled == 1
state != STATE_ESTOP
Homing Mode == 0
Target Motor == 0xFF
```

Board1은 local motor `0~3`, Board2는 local motor `0`을 homing한다.
Board3 gripper home은 `0x023`을 사용한다.

### 4.4 Gripper Home Posture, CAN ID `0x023`

```text
CAN ID = 0x023
DLC = 8
대상 = Board3
payload에 Board ID를 넣지 않는다.
```

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | Target Motor | `0xFF`: 전체 gripper home posture |
| 1 | Home Mode | `0` |
| 2 | Duration | 5ms tick. `0`이면 firmware 기본값 사용 |
| 3~7 | Reserved | `0` |

```bash
cansend can0 023#FF00000000000000
```

Board3 home posture는 모든 gripper Motor ID `0~8`의 목표 각도를
`0.00도`로 설정하는 명령이다.

### 4.5 Clear Error, CAN ID `0x030`

```text
CAN ID = 0x030
DLC = 8
payload에 Target Board를 넣지 않는다.
```

| Byte | 필드 | 값 |
|---:|---|---|
| 0 | Target Motor | `0xFF`: 전체 error clear |
| 1~7 | Reserved | `0` |

```bash
cansend can0 030#FF00000000000000
```

Clear Error는 error/fault flag와 queue 관련 error를 clear한다. ESTOP 상태는
Clear Error만으로 해제하지 않고, `0x010 Enable=1`에서 해제한다.

---

## 5. 공통 단위

### 5.1 각도 raw 단위

모든 trajectory command의 target position은 `0.01도` 단위 signed integer다.

| 실제 각도 | raw 값 | little endian 예 |
|---:|---:|---|
| `30.00 deg` | `3000` | `B8 0B 00 00` |
| `-15.50 deg` | `-1550` | `F2 F9 FF FF` |
| `-90.00 deg` | `-9000` | `D8 DC FF FF` |

### 5.2 Duration

`Duration`은 5ms tick이다.

```text
duration_ms = Byte7 * 5
```

| Byte7 | 실제 시간 |
|---:|---:|
| `1` | `5ms` |
| `20` | `100ms` |
| `100` | `500ms` |

`Byte7 = 0`이면 STM32 내부에서 최소 segment로 처리한다.

---

## 6. Board1 Protocol

### 6.1 역할

Board1은 팔 2~5축 stepper 4개를 담당한다.

| Board1 Local Motor ID | MoveIt Joint | Min deg | Max deg | Home deg |
|---:|---|---:|---:|---:|
| `0` | `arm_joint_1` | `-90` | `90` | `-90` |
| `1` | `arm_joint_2` | `-80` | `80` | `-80` |
| `2` | `arm_joint_3` | `-90` | `90` | `-90` |
| `3` | `arm_joint_4` | `-170` | `170` | `-170` |

### 6.2 Move Command, CAN ID `0x101`

```text
CAN ID = 0x101
DLC = 8
payload에 Board ID 없음
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Motor ID | `uint8_t` | flags + Board1 local motor id |
| 1 | Target Pos LSB | `int32_t` 일부 | 0.01도 단위, little endian |
| 2 | Target Pos | `int32_t` 일부 |  |
| 3 | Target Pos | `int32_t` 일부 |  |
| 4 | Target Pos MSB | `int32_t` 일부 |  |
| 5 | Speed LSB | `uint16_t` 일부 | 0.01도/s, 초기 firmware에서는 미사용 가능 |
| 6 | Speed MSB | `uint16_t` 일부 |  |
| 7 | Duration | `uint8_t` | 5ms tick |

Byte0 구조:

```text
Bit7 Bit6 Bit5 Bit4 | Bit3 Bit2 Bit1 Bit0
Exec Rel  Step Rsv  | Motor ID
```

| Bit | 이름 | 의미 |
|---:|---|---|
| 7 | Execute | `1`: staging 대상 |
| 6 | Relative | `1`: 현재 위치 기준 상대 이동 |
| 5 | Step Mode | `0`: angle raw, `1`: step |
| 4 | Reserved | 반드시 `0` |
| 3~0 | Motor ID | `0~3` |

일반 절대 각도 명령의 Byte0:

| Motor ID | Byte0 |
|---:|---:|
| `0` | `0x80` |
| `1` | `0x81` |
| `2` | `0x82` |
| `3` | `0x83` |

### 6.3 Board1 4-frame Staging

Board1 `0x101`은 한 축씩 바로 실행하지 않는다. MoveIt trajectory point 하나는
반드시 아래 4개 frame이 연속으로 들어와야 한다.

```text
Motor ID 0 -> 1 -> 2 -> 3
첫 frame 이후 20ms 안에 4개 frame 수신
4개 frame의 Duration 동일
Execute=1
Reserved bit=0
안 움직이는 축도 현재 목표 위치를 다시 보내야 함
```

4번째 frame까지 정상 수신되면 하나의 4축 point로 queue에 들어가고, 네 축이
같은 tick에서 동시 시작한다.

### 6.4 Board1 Status, CAN ID `0x201`

```text
CAN ID = 0x201
DLC = 8
주기 = 100ms + 주요 이벤트 즉시 송신
```

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | State | 현재 보드 상태 |
| 1 | Error Code | 현재 error code |
| 2 | Homing Done Bits | bit0~3 = local motor 0~3 homing done |
| 3 | Moving Motor ID | 이동/homing 중인 motor id, 없으면 `255` |
| 4 | Limit Status Bits | bit0~3 = local motor 0~3 limit active |
| 5 | Queue Free | 외부 `0x101` command slot 기준, `0~32` |
| 6 | Enabled | `0`: disabled, `1`: enabled |
| 7 | Reserved | `0` |

### 6.5 Board1 Position Feedback, CAN ID `0x301`

```text
CAN ID = 0x301
DLC = 8
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Local Motor ID | `uint8_t` | `0~3` |
| 1 | Flags | `uint8_t` | position valid / homed / moving / target reached |
| 2~5 | Current Pos | `int32_t` | 0.01도 단위, little endian |
| 6 | Error / Fault Code | `uint8_t` | 없으면 `0` |
| 7 | Sequence Counter | `uint8_t` | 송신 순서 확인 |

Flags:

| Bit | 의미 |
|---:|---|
| bit0 | Position Valid |
| bit1 | Homed / Ready |
| bit2 | Moving |
| bit3 | Target Reached |
| bit4~7 | Reserved, `0` |

---

## 7. Board2 Protocol

### 7.1 역할

Board2는 base 1축 stepper를 담당한다.

| Local Motor ID | MoveIt Joint | Min deg | Max deg | Home deg |
|---:|---|---:|---:|---:|
| `0` | `base_joint` | `-90` | `180` | `-90` |

### 7.2 Move Command, CAN ID `0x102`

```text
CAN ID = 0x102
DLC = 8
payload에 Board ID 없음
대상 local motor id = 0
```

Payload는 Board1 `0x101`과 같은 구조를 사용한다.

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Motor ID | `uint8_t` | Board2에서는 일반 명령 `0x80` |
| 1~4 | Target Pos | `int32_t` | 0.01도 단위, little endian |
| 5~6 | Speed | `uint16_t` | 초기 firmware에서는 미사용 가능 |
| 7 | Duration | `uint8_t` | 5ms tick |

Board2 일반 절대 각도 명령은 항상:

```text
Byte0 = 0x80
```

예시:

```bash
# Board2 base를 30.00도로 50ms 동안 이동
cansend can0 102#80B80B0000E8030A

# Board2 base를 home 위치 -90.00도로 50ms 동안 이동
cansend can0 102#80D8DCFFFFE8030A
```

### 7.3 Board2 Status, CAN ID `0x202`

```text
CAN ID = 0x202
DLC = 8
주기 = 100ms + 주요 이벤트 즉시 송신
```

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | State |
| 1 | Error Code |
| 2 | Homing Done Bits | bit0 = base axis homing done |
| 3 | Moving Motor ID | 이동/homing 중이면 `0`, 없으면 `255` |
| 4 | Limit Status Bits | bit0 = base limit active |
| 5 | Queue Free | `0x102` command slot 기준 |
| 6 | Enabled | `0`: disabled, `1`: enabled |
| 7 | Reserved | `0` |

예시:

```text
202#010001FF00200100

state=STATE_IDLE
error=ERR_NONE
homing_done bit0=1
moving=255
queue_free=32
enabled=1
```

### 7.4 Board2 Position Feedback, CAN ID `0x302`

```text
CAN ID = 0x302
DLC = 8
대상 = Board2 local motor 0 / base_joint
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Local Motor ID | `uint8_t` | 항상 `0` |
| 1 | Flags | `uint8_t` | position valid / homed / moving / target reached |
| 2~5 | Current Pos | `int32_t` | 0.01도 단위, little endian |
| 6 | Error / Fault Code | `uint8_t` | 없으면 `0` |
| 7 | Sequence Counter | `uint8_t` | 송신 순서 확인 |

예시:

```text
302#0001D8DCFFFF00XX  # position valid, -90.00도
302#000BD8DCFFFF00XX  # valid + homed + target reached, -90.00도
```

---

## 8. Board3 Protocol

### 8.1 역할

Board3는 SCS0009 gripper servo 9개를 담당한다.

| Local Motor ID | Joint | Servo ID |
|---:|---|---:|
| `0` | `finger_1_base_joint` | `1` |
| `1` | `finger_1_middle_joint` | `2` |
| `2` | `finger_1_tip_joint` | `3` |
| `3` | `finger_2_base_joint` | `4` |
| `4` | `finger_2_middle_joint` | `5` |
| `5` | `finger_2_tip_joint` | `6` |
| `6` | `finger_3_base_joint` | `7` |
| `7` | `finger_3_middle_joint` | `8` |
| `8` | `finger_3_tip_joint` | `9` |

### 8.2 Servo Command, CAN ID `0x103`

```text
CAN ID = 0x103
DLC = 8
payload에 Board ID 없음
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Control & Motor ID | `uint8_t` | Bit7 Execute, Bit3~0 Motor ID |
| 1 | Target Position LSB | `int32_t` 일부 | 0.01도 단위, little endian |
| 2 | Target Position | `int32_t` 일부 |  |
| 3 | Target Position | `int32_t` 일부 |  |
| 4 | Target Position MSB | `int32_t` 일부 |  |
| 5 | Target Load LSB | `uint16_t` 일부 | 프로젝트 확장, little endian |
| 6 | Target Load MSB | `uint16_t` 일부 | 프로젝트 확장, little endian |
| 7 | Duration | `uint8_t` | 5ms tick |

원본 Board3 v1.1 문서에서는 Byte5~6을 `Speed`로 두고 미사용 `0`을 권장했다.
본 프로젝트에서는 같은 위치를 gripper 파지 부하 임계값으로 재정의한다.

```text
Target Load 범위 = 0~1023
권장 기본값 = 500
```

Board3 firmware는 이 값을 부하 임계값으로 해석한다. 모터가 target load에
도달하면 이동을 멈추고 hold 상태를 유지할 수 있다. load 기능이 없는
firmware와 테스트할 때는 Byte5~6을 `0`으로 보내도 기존 v1.1 구조와 호환된다.

Byte0 구조:

```text
Bit7 Bit6 Bit5 Bit4 | Bit3 Bit2 Bit1 Bit0
Exec Rsv  Rsv  Rsv  | Motor ID
```

| Motor ID | Byte0 |
|---:|---:|
| `0` | `0x80` |
| `1` | `0x81` |
| `2` | `0x82` |
| `3` | `0x83` |
| `4` | `0x84` |
| `5` | `0x85` |
| `6` | `0x86` |
| `7` | `0x87` |
| `8` | `0x88` |

처리 조건:

```text
execute == 1
reserved bits == 0
motor_id <= 8
enabled == 1
state != STATE_ESTOP
```

### 8.3 Board3 9-frame Staging

Gripper 한 번의 동작은 `0x103` frame 9개로 구성된다.

```text
Motor ID 0 -> 1 -> 2 -> 3 -> 4 -> 5 -> 6 -> 7 -> 8
9개 frame의 Duration 동일
Execute=1
Reserved bit=0
중복 Motor ID 없음
```

9개 frame이 정상 수집되면 하나의 gripper command set으로 처리한다.

예시, 모든 servo를 `0.00도`, target load `500`, duration `100ms`로 이동:

```bash
cansend can0 103#8000000000F40114
cansend can0 103#8100000000F40114
cansend can0 103#8200000000F40114
cansend can0 103#8300000000F40114
cansend can0 103#8400000000F40114
cansend can0 103#8500000000F40114
cansend can0 103#8600000000F40114
cansend can0 103#8700000000F40114
cansend can0 103#8800000000F40114
```

```text
Byte5~6 = F4 01 = 0x01F4 = 500
Byte7 = 0x14 = 20 * 5ms = 100ms
```

원본 v1.1 speed-unused firmware 테스트용 예시:

```bash
cansend can0 103#8000000000000014
```

### 8.4 Board3 Status, CAN ID `0x203`

```text
CAN ID = 0x203
DLC = 8
주기 = 100ms + 주요 이벤트 즉시 송신
```

| Byte | 필드 | 설명 |
|---:|---|---|
| 0 | State | 현재 Board3 상태 |
| 1 | Error Code | 현재 error code |
| 2 | Ready | 제어 가능 상태면 `1` |
| 3 | Staging Count | 현재 staging된 frame 개수 `0~9` |
| 4 | Fault | fault 있으면 `1` |
| 5 | Buffer Free | `9 - staging_count` |
| 6 | Enabled | `0`: disabled, `1`: enabled |
| 7 | Fault Motor ID | fault motor id, 없으면 `255` |

`0x203`에는 9개 servo 위치를 넣지 않는다. 현재 위치는 `0x303`으로 송신한다.

### 8.5 Board3 Position Feedback, CAN ID `0x303`

```text
CAN ID = 0x303
DLC = 8
주기 = 20ms
1 feedback cycle = 0x303 frame 3개
```

| Byte | 필드 | 자료형 | 설명 |
|---:|---|---|---|
| 0 | Feedback Frame Index | `uint8_t` | `0x01`, `0x02`, `0x03` |
| 1~2 | Position A | `int16_t` | 0.01도 단위, little endian |
| 3~4 | Position B | `int16_t` | 0.01도 단위, little endian |
| 5~6 | Position C | `int16_t` | 0.01도 단위, little endian |
| 7 | Reserved / Group Flag | `uint8_t` | v1.1 기본 `0x00` |

Index별 mapping:

| Byte0 Index | 포함 Motor ID | Byte1~2 | Byte3~4 | Byte5~6 |
|---:|---|---|---|---|
| `0x01` | `0,1,2` | Motor 0 | Motor 1 | Motor 2 |
| `0x02` | `3,4,5` | Motor 3 | Motor 4 | Motor 5 |
| `0x03` | `6,7,8` | Motor 6 | Motor 7 | Motor 8 |

중요:

```text
Byte7 = 0x00이어도 정상 위치 데이터로 처리해야 한다.
향후 flag 확장이 필요하면 Byte7을 bitmap으로 확장할 수 있다.
```

예시:

```bash
cansend can0 303#0100000000000000
cansend can0 303#0200000000000000
cansend can0 303#0300000000000000
```

---

## 9. State / Error 값

### 9.1 Board1 / Board2 State

| 값 | 이름 | 설명 |
|---:|---|---|
| `0` | `STATE_INIT` | 초기화 중 |
| `1` | `STATE_IDLE` | 대기 상태 |
| `2` | `STATE_HOMING` | 원점복귀 중 |
| `3` | `STATE_MOVING` | 이동 중 |
| `4` | `STATE_ERROR` | 에러 발생 |
| `5` | `STATE_ESTOP` | 비상정지 상태 |
| `6` | `STATE_DISABLED` | Disable 상태 |

### 9.2 Board1 / Board2 Error

| 값 | 이름 | 의미 |
|---:|---|---|
| `0` | `ERR_NONE` | 정상 |
| `1` | `ERR_INVALID_CMD` | 잘못된 명령 |
| `2` | `ERR_LIMIT_SWITCH_DETECTED` | limit switch 감지 또는 예약 |
| `3` | `ERR_DRIVER_FAULT` | driver fault |
| `4` | `ERR_HOMING_FAIL` | homing 실패 또는 예약 |
| `5` | `ERR_QUEUE_FULL` | trajectory queue full |
| `6` | `ERR_RESERVED` | 예약 |

### 9.3 Board3 State

| 값 | 이름 | 설명 |
|---:|---|---|
| `0` | `STATE_INIT` | 초기화 중 |
| `1` | `STATE_IDLE` | 대기 상태 |
| `2` | `STATE_STAGING` | 9개 frame 수집 중 |
| `3` | `STATE_MOVING` | 명령 처리 또는 이동 중 |
| `4` | `STATE_ERROR` | 에러 발생 |
| `5` | `STATE_ESTOP` | 비상정지 상태 |
| `6` | `STATE_DISABLED` | Disable 상태 |

### 9.4 Board3 Error

| 값 | 이름 | 의미 |
|---:|---|---|
| `0` | `ERR_NONE` | 정상 |
| `1` | `ERR_INVALID_CMD` | 잘못된 명령 |
| `2` | `ERR_INVALID_MOTOR_ID` | Motor ID 범위 오류 |
| `3` | `ERR_DUPLICATE_MOTOR_ID` | command set 안에서 Motor ID 중복 |
| `4` | `ERR_STAGING_TIMEOUT` | 9개 frame 수집 timeout |
| `5` | `ERR_DURATION_MISMATCH` | 9개 frame duration 불일치 |
| `6` | `ERR_ANGLE_RANGE` | 목표 각도 범위 초과 |
| `7` | `ERR_SERVO_COMM` | servo 통신 오류 |
| `8` | `ERR_SERVO_FAULT` | servo fault 또는 과부하 |
| `9` | `ERR_ESTOP` | ESTOP 상태 |
| `10` | `ERR_DISABLED` | Disable 상태에서 command 수신 |

---

## 10. Bring-Up 확인 순서

### 10.1 통신 확인

```bash
ip -details -statistics link show can0
candump can0 -tz
```

정상 수신 예:

```text
201  Board1 status
202  Board2 status
203  Board3 status
301  Board1 position feedback
302  Board2 position feedback
303  Board3 position feedback
```

Board2/Board3만 연결한 경우에는 `202`, `203`, `302`, `303`만 보여도
해당 보드의 송신 통신은 정상이다.

### 10.2 Enable / Homing / Clear

```bash
# 전체 enable
cansend can0 010#0100000000000000

# Board1+Board2 stepper homing
cansend can0 020#FF00000000000000

# Board3 gripper home posture
cansend can0 023#FF00000000000000

# 전체 clear error
cansend can0 030#FF00000000000000
```

### 10.3 Board2 단독 명령 예

```bash
# base_joint를 30.00도로 50ms 이동
cansend can0 102#80B80B0000E8030A
```

### 10.4 Board3 단독 명령 예

```bash
# gripper 전체 0.00도, target load 500, duration 100ms
cansend can0 103#8000000000F40114
cansend can0 103#8100000000F40114
cansend can0 103#8200000000F40114
cansend can0 103#8300000000F40114
cansend can0 103#8400000000F40114
cansend can0 103#8500000000F40114
cansend can0 103#8600000000F40114
cansend can0 103#8700000000F40114
cansend can0 103#8800000000F40114
```

---

## 11. ROS / arm_can_bridge 구현 요구사항

`arm_can_bridge`는 이 문서 기준으로 다음 frame을 생성해야 한다.

| 기능 | CAN frame |
|---|---|
| Enable | `010#0100000000000000` |
| Disable | `010#0000000000000000` |
| Stepper homing | `020#FF00000000000000` |
| Gripper home posture | `023#FF00000000000000` |
| Clear error | `030#FF00000000000000` |
| ESTOP | `001#0100000000000000` |
| Board1 move | `0x101`, payload local motor id `0~3` |
| Board2 move | `0x102`, payload local motor id `0` |
| Board3 move | `0x103`, payload local motor id `0~8`, Byte5~6 target load |

`0x303` feedback parser는 Byte7이 `0x00`이어도 위치값을 정상 처리해야 한다.

---

## 12. 최종 체크리스트

```text
1. payload에 Board ID를 넣지 않는다.
2. Board 구분은 CAN ID로 한다.
3. 0x010 Enable/Disable은 Byte0만 사용한다.
4. 0x020은 Board1+Board2 stepper homing이다.
5. Board3 home posture는 0x023이다.
6. 0x030 Clear Error는 Byte0=0xFF만 사용한다.
7. Board1 0x101은 4-frame staging이다.
8. Board3 0x103은 9-frame staging이다.
9. Board3 0x103 Byte5~6은 본 프로젝트에서 Target Load로 확장한다.
10. 0x203은 Board3 status이고 위치는 0x303으로 보낸다.
11. 0x303 Byte7=0x00도 정상 feedback으로 처리한다.
```
