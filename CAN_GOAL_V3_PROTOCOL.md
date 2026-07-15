# Board1/Board2 final-goal protocol V3

V3 sends one final absolute joint goal and one requested duration. There is no
trajectory point queue, point sequence, Queue Free credit, preload, or refill.

## Goal frames

Board1 uses CAN ID `0x101` and four local motors. Board2 uses `0x102` and local
motor 0. DLC is always 8.

```text
Byte0    0x90 | local_motor_id
Byte1-4 int32 absolute target, little-endian, 0.01 degree
Byte5   uint8 goal_id
Byte6-7 uint16 duration_ms, little-endian (1..65535)
```

Relative commands are rejected. Board1 accepts its four motor frames in any
arrival order and reports READY only after all four matching frames arrive.
Board2 reports READY after its one frame. A different goal is rejected while a
goal is staged or moving.

## Control

CAN ID `0x040`, DLC 8:

```text
Byte0 1=START, 2=CANCEL
Byte1 goal_id
Byte2-7 zero
```

Send START only after READY from both boards.

## ACK/NACK

Board1 uses `0x401`; Board2 uses `0x402`.

```text
Byte0 protocol version = 3
Byte1 result
Byte2 goal_id
Byte3 received axis mask (Board1 ready=0x0F, Board2 ready=0x01)
Byte4 state snapshot
Byte5 reserved zero
Byte6-7 duration_ms echo, little-endian
```

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

Status `0x201/0x202` Byte5 is now `goal_slot_free`: 1 means a new goal may be
staged; 0 means a goal is partial, ready, or moving. It is not Queue Free.

## Software powered-hold E-stop

E-stop uses CAN ID `0x001`, DLC 8, payload `01 00 00 00 00 00 00 00`.
It immediately cancels goal staging, motion, and homing and stops STEP pulses.
It does not change the existing motor-enable state: an enabled motor keeps
holding torque, while a disabled motor is not automatically enabled. During
E-stop, status state is `STATE_ESTOP`, axis moving/ready flags are zero, and
`goal_slot_free` is zero.

`0x010 Enable=1` or `0x030 Clear Error` releases E-stop. Enable also enables
the motor; Clear Error preserves the existing motor-enable state. Neither
command restarts the cancelled goal. `0x010 Enable=0` remains the only runtime
command that disables the motor and must be sent only by an explicit
administrator/user action. Boot/reset still starts disabled.

This is a project-level software powered hold, not a safety-rated STO or
industrial emergency-stop function. With open-loop steppers, current position
is estimated from commanded steps; an immediate stop can lose steps and make
the estimate differ from the physical position.

## Axis-local limit behavior

During normal motion, a debounced limit switch blocks only movement of that
local axis farther in its home direction. Firmware immediately changes that
axis target to its current commanded position without setting the global
`ERR_LIMIT_SWITCH_DETECTED`; other axes in the same goal continue. A later
command in the opposite direction is allowed even while the switch is still
active, so Clear Error is not required to move away from the limit. Status
Byte4 continues to report the raw active limit bits. The server may therefore
observe the coordinated goal as completed even though the blocked axis did not
reach its originally requested target.

Both boards generate a simple triangular/trapezoidal speed envelope internally.
The configured initial limits are 1000 step/s and 500 step/s². An unsafe short
duration may be extended; completion must be determined from status, not a
duration-only timeout.
