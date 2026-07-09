# Arduino TB6600 Motor Test

전원을 넣으면 모터가 바로 도는 단독 테스트 스케치입니다. CAN, MCP2515, 리미트 스위치 없이 TB6600 STEP/DIR/ENA만 확인합니다.

## 핀맵

| TB6600 | Arduino |
|---|---:|
| `PUL+` | `D6` |
| `PUL-` | `GND` |
| `DIR+` | `D7` |
| `DIR-` | `GND` |
| `ENA+` | `D8` |
| `ENA-` | `GND` |

모터 코일 2쌍은 TB6600의 `A+/A-`, `B+/B-`에 연결합니다.

## 설정

- TB6600 모터 전원은 별도 전원을 사용하세요. Arduino 5V를 모터 전원으로 쓰면 안 됩니다.
- 이 테스트는 `D8 LOW`로 ENA 입력을 꺼둡니다. 많은 TB6600 모듈은 ENA를 안 넣을 때 기본 enable 상태입니다.
- 기존 Board2 코드와 맞추려면 TB6600 microstep DIP는 `1/16`으로 맞추는 것이 좋습니다.
- 방향이 반대면 `DIR_LEVEL`을 `LOW`로 바꾸거나 모터 한 코일의 `+/-`를 바꾸면 됩니다.
- 너무 빠르거나 탈조하면 `RUN_STEP_LOW_US` 값을 키우세요. 값이 클수록 느립니다.
