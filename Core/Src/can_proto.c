#include "../Inc/can_proto.h"
#include "../Inc/stepper.h"
#include "../Inc/trajectory.h"
#include "../Inc/mcp2515.h"

static int32_t get_i32_le(const uint8_t *p)
{
    uint32_t v = ((uint32_t)p[0]) |
                 ((uint32_t)p[1] << 8) |
                 ((uint32_t)p[2] << 16) |
                 ((uint32_t)p[3] << 24);
    return (int32_t)v;  // CAN 데이터 4바이트를 little-endian int32 값으로 변환
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0]) | ((uint16_t)p[1] << 8));  // CAN 데이터 2바이트를 little-endian uint16 값으로 변환
}

void can_send_status(void)
{
    uint8_t data[8];  // [8바이트] CAN 송신할 status 데이터 

    system_update_state();  // 현재 모터 상태를 전역 상태값에 반영
    data[0] = global_motor_state;           // 현재 시스템 상태
    data[1] = global_motor_error;           // 현재 에러 코드
    data[2] = system_homing_done_bits();    // 각 축의 원점복귀 완료 비트
    data[3] = system_first_moving_axis();   // 움직이는 축 중 첫 번째 축 번호
    data[4] = stepper_limit_status_bits();  // 리미트 스위치 입력 상태 비트
    data[5] = trajectory_free_count();      // 남은 궤적 큐 슬롯 수
    data[6] = system_enabled_status();      // 모터 enable/estop 상태
    data[7] = 0;                            // 예비 바이트
    (void)mcp2515_send_std(CAN_ID_BOARD1_STAT, data, 8);  // 보드 상태 프레임 송신
}

uint8_t can_decode_frame(uint16_t id, const uint8_t *data, uint8_t len, CanCommand *cmd)
{
    if (id == CAN_ID_ESTOP) {
        cmd->type = CAN_CMD_ESTOP;
        return 1;
    }

    if (id == CAN_ID_ENABLE) {
        if (len < 1) return 0;  // enable 값이 없으면 무시
        cmd->type = CAN_CMD_ENABLE;
        cmd->enable = data[0];
        return 1;
    }

    if (id == CAN_ID_HOMING) {
        if (len < 2) return 0;  // 축 번호와 모드 값이 모두 있어야 처리
        cmd->type = CAN_CMD_HOMING;
        cmd->target_axis = data[0];  // 원점복귀 대상 축 번호
        cmd->homing_mode = data[1];  // 원점복귀 모드
        return 1;
    }

    if (id == CAN_ID_CLEAR_ERROR) {
        cmd->type = CAN_CMD_CLEAR_ERROR;
        cmd->target_axis = HOMING_ALL_AXIS;  // 기본값은 전체 축 대상
        if (len >= 1) cmd->target_axis = data[0];  // 데이터가 있으면 지정 축만 대상으로 처리
        return 1;
    }

    if (id == CAN_ID_BOARD1_MOVE) {
        if (len < 8) return 0;  // 이동 명령은 8바이트 프레임만 처리
        cmd->type = CAN_CMD_MOVE;
        cmd->move.motor_id = data[0] & 0x0F;       // 하위 4비트: 축 번호
        cmd->move.flags = data[0] >> 4;             // 상위 4비트: 실행/좌표 모드 플래그
        cmd->move.target_pos = get_i32_le(&data[1]);  // 목표 위치(raw 각도 단위)
        cmd->move.speed = get_u16_le(&data[5]);       // 목표 속도 필드
        cmd->move.duration_5ms = data[7];             // 이동 시간(5ms 단위)
        return 1;
    }

    return 0;
}
