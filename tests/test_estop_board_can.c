#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MCP2515_H
#define GPIO_H
#define STEPPER_H
#define TRAJECTORY_H

#define AXIS_COUNT 4
#define ENABLE_ESTOP_LOGIC 1

#define CAN_ID_ESTOP 0x001
#define CAN_ID_ENABLE 0x010
#define CAN_ID_HOMING 0x020
#define CAN_ID_CLEAR_ERROR 0x030
#define CAN_ID_GOAL_CONTROL 0x040
#define BOARD_MOVE_CAN_ID 0x101
#define BOARD_STATUS_CAN_ID 0x201
#define BOARD_POSITION_CAN_ID 0x301
#define BOARD_ACK_CAN_ID 0x401

#define CAN_CTRL_EXECUTE 0x80
#define CAN_CTRL_RELATIVE 0x40
#define CAN_CTRL_STEP_MODE 0x20
#define CAN_CTRL_GOAL_V3 0x10
#define CAN_CTRL_MOTOR_MASK 0x0F

#define STATE_INIT 0
#define STATE_IDLE 1
#define STATE_HOMING 2
#define STATE_MOVING 3
#define STATE_ERROR 4
#define STATE_ESTOP 5
#define STATE_DISABLED 6

#define ERR_NONE 0
#define ERR_INVALID_CMD 1
#define ERR_DRIVER_FAULT 3

#define GOAL_CONTROL_START 1
#define GOAL_CONTROL_CANCEL 2
#define GOAL_ACK_READY 0
#define GOAL_ACK_STARTED 1
#define GOAL_ACK_DUPLICATE 2
#define GOAL_ACK_BUSY 3
#define GOAL_ACK_STAGING_TIMEOUT 4
#define GOAL_ACK_CONFLICT 5
#define GOAL_ACK_CANCELLED 6
#define GOAL_ACK_INVALID 7

#define GOAL_STAGE_WAITING 0
#define GOAL_STAGE_READY 1
#define GOAL_STAGE_INVALID 2
#define GOAL_STAGE_DUPLICATE 3
#define GOAL_STAGE_BUSY 4

#define HOMING_ALL_AXIS 255
#define ESTOP_ACTIVE() (g_estop != 0)

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} CanFrame;

typedef enum {
    MCP2515_SEND_BUSY = 0,
    MCP2515_SEND_OK = 1,
    MCP2515_SEND_FAULT = 2
} Mcp2515SendResult;

volatile uint8_t g_enabled;
volatile uint8_t g_estop;
volatile uint8_t g_state;
volatile uint8_t g_error_code;
volatile uint8_t g_motion_active;
volatile uint8_t g_homing_active;
volatile uint8_t g_homing_done_bits;
volatile int32_t g_current_step[AXIS_COUNT];
volatile int32_t g_target_step[AXIS_COUNT];
volatile int32_t g_motion_start_step[AXIS_COUNT];
volatile uint32_t global_tick_ms;

static uint8_t mock_slot_free;
static uint8_t mock_stage_calls;
static uint8_t mock_start_calls;
static uint8_t mock_homing_calls;
static uint8_t mock_motor_enable_calls;
static uint8_t mock_motor_disable_calls;
static uint8_t mock_trajectory_clear_calls;
static uint8_t mock_stepper_stop_calls;
static uint8_t mock_cancel_calls;

void trajectory_clear(void);
void stepper_stop_all(void);
void motor_enable(void);
void motor_disable(void);
uint8_t system_all_homed(void);
uint8_t trajectory_take_timeout_event(uint8_t *goal_id, uint8_t *mask,
                                      uint16_t *duration_ms);
void stepper_start_homing_all(void);
uint8_t trajectory_resolve_target_step(uint8_t axis_id, int32_t target_raw,
                                       uint8_t relative, uint8_t step_mode,
                                       int32_t *target_step);
uint8_t trajectory_stage_goal_axis(uint8_t motor_id, int32_t target_step,
                                   uint8_t goal_id, uint16_t duration_ms,
                                   uint8_t *received_axis_mask);
uint16_t trajectory_goal_duration_ms(void);
uint8_t trajectory_goal_mask(void);
uint8_t trajectory_start_goal(uint8_t goal_id);
uint8_t trajectory_cancel_goal(uint8_t goal_id);
uint8_t stepper_limit_switch_status_bits(void);
uint8_t trajectory_goal_slot_free(void);
uint8_t system_reported_error_code(void);
uint8_t system_enabled_status(void);
int32_t step_to_angle(uint8_t axis_id, int32_t step);
Mcp2515SendResult mcp2515_send_frame(const CanFrame *frame);

#include "../Core/Src/board_can.c"

void trajectory_clear(void)
{
    mock_trajectory_clear_calls++;
    g_motion_active = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_target_step[i] = g_current_step[i];
        g_motion_start_step[i] = g_current_step[i];
    }
    mock_slot_free = 1;
}

void stepper_stop_all(void)
{
    mock_stepper_stop_calls++;
}

void motor_enable(void)
{
    mock_motor_enable_calls++;
}

void motor_disable(void)
{
    mock_motor_disable_calls++;
}

uint8_t system_all_homed(void)
{
    return g_homing_done_bits == 0x0F ? 1 : 0;
}

uint8_t trajectory_take_timeout_event(uint8_t *goal_id, uint8_t *mask,
                                      uint16_t *duration_ms)
{
    (void)goal_id;
    (void)mask;
    (void)duration_ms;
    return 0;
}

void stepper_start_homing_all(void)
{
    mock_homing_calls++;
}

uint8_t trajectory_resolve_target_step(uint8_t axis_id, int32_t target_raw,
                                       uint8_t relative, uint8_t step_mode,
                                       int32_t *target_step)
{
    (void)axis_id;
    (void)relative;
    (void)step_mode;
    *target_step = target_raw;
    return 1;
}

uint8_t trajectory_stage_goal_axis(uint8_t motor_id, int32_t target_step,
                                   uint8_t goal_id, uint16_t duration_ms,
                                   uint8_t *received_axis_mask)
{
    (void)motor_id;
    (void)target_step;
    (void)goal_id;
    (void)duration_ms;
    mock_stage_calls++;
    *received_axis_mask = 0x0F;
    return GOAL_STAGE_READY;
}

uint16_t trajectory_goal_duration_ms(void)
{
    return 1000;
}

uint8_t trajectory_goal_mask(void)
{
    return 0x0F;
}

uint8_t trajectory_start_goal(uint8_t goal_id)
{
    (void)goal_id;
    mock_start_calls++;
    return 1;
}

uint8_t trajectory_cancel_goal(uint8_t goal_id)
{
    (void)goal_id;
    mock_cancel_calls++;
    return 1;
}

uint8_t stepper_limit_switch_status_bits(void)
{
    return 0;
}

uint8_t trajectory_goal_slot_free(void)
{
    return mock_slot_free;
}

uint8_t system_reported_error_code(void)
{
    return g_error_code;
}

uint8_t system_enabled_status(void)
{
    return g_enabled ? 1 : 0;
}

int32_t step_to_angle(uint8_t axis_id, int32_t step)
{
    (void)axis_id;
    return step;
}

Mcp2515SendResult mcp2515_send_frame(const CanFrame *frame)
{
    (void)frame;
    return MCP2515_SEND_OK;
}

static CanFrame control_frame(uint16_t id, uint8_t value)
{
    CanFrame frame;
    memset(&frame, 0, sizeof(frame));
    frame.id = id;
    frame.dlc = 8;
    frame.data[0] = value;
    return frame;
}

static void reset_fixture(uint8_t enabled)
{
    g_enabled = enabled;
    g_estop = 0;
    g_state = enabled ? STATE_IDLE : STATE_DISABLED;
    g_error_code = ERR_NONE;
    g_motion_active = 0;
    g_homing_active = 0;
    g_homing_done_bits = 0x0F;
    global_tick_ms = 0;
    mock_slot_free = 1;
    mock_stage_calls = 0;
    mock_start_calls = 0;
    mock_homing_calls = 0;
    mock_motor_enable_calls = 0;
    mock_motor_disable_calls = 0;
    mock_trajectory_clear_calls = 0;
    mock_stepper_stop_calls = 0;
    mock_cancel_calls = 0;
    g_ack_tx_head = 0;
    g_ack_tx_tail = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_current_step[i] = 100 + i;
        g_target_step[i] = 200 + i;
        g_motion_start_step[i] = 0;
    }
}

static void assert_estop_axis_flags(uint8_t packed)
{
    assert((packed & 0x0F) == 0x09);
    assert(((packed >> 4) & 0x0F) == 0x09);
}

static void test_enabled_estop_holds_torque_and_clears_work(void)
{
    CanFrame estop = control_frame(CAN_ID_ESTOP, 1);

    reset_fixture(1);
    g_motion_active = 1;
    g_homing_active = 1;
    g_state = STATE_MOVING;
    g_error_code = ERR_DRIVER_FAULT;
    g_ack_tx_tail = 3;

    board_can_handle_frame(&estop);

    assert(g_estop == 1);
    assert(g_enabled == 1);
    assert(g_state == STATE_ESTOP);
    assert(g_error_code == ERR_DRIVER_FAULT);
    assert(g_motion_active == 0);
    assert(g_homing_active == 0);
    assert(g_homing_done_bits == 0x0F);
    assert(mock_trajectory_clear_calls == 1);
    assert(mock_stepper_stop_calls == 1);
    assert(mock_motor_enable_calls == 0);
    assert(mock_motor_disable_calls == 0);
    assert(g_ack_tx_head == g_ack_tx_tail);
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        assert(g_target_step[i] == g_current_step[i]);
    }

    board_can_queue_status();
    assert(g_pending_status_frame.data[0] == STATE_ESTOP);
    assert(g_pending_status_frame.data[1] == ERR_DRIVER_FAULT);
    assert(g_pending_status_frame.data[5] == 0);
    assert(g_pending_status_frame.data[6] == 1);
    assert_estop_axis_flags(g_pending_status_frame.data[2]);
    assert_estop_axis_flags(g_pending_status_frame.data[3]);
}

static void test_clear_releases_estop_without_changing_enable(void)
{
    CanFrame estop = control_frame(CAN_ID_ESTOP, 1);
    CanFrame clear = control_frame(CAN_ID_CLEAR_ERROR, HOMING_ALL_AXIS);

    reset_fixture(1);
    board_can_handle_frame(&estop);
    board_can_handle_frame(&clear);
    assert(g_estop == 0);
    assert(g_enabled == 1);
    assert(g_error_code == ERR_NONE);
    assert(g_state == STATE_IDLE);
    assert(mock_motor_enable_calls == 0);
    assert(mock_motor_disable_calls == 0);
    assert(mock_stage_calls == 0);
    assert(mock_start_calls == 0);

    reset_fixture(0);
    board_can_handle_frame(&estop);
    board_can_handle_frame(&clear);
    assert(g_estop == 0);
    assert(g_enabled == 0);
    assert(g_state == STATE_DISABLED);
    assert(mock_motor_enable_calls == 0);
    assert(mock_motor_disable_calls == 0);
}

static void test_enable_releases_estop_and_disable_is_explicit(void)
{
    CanFrame estop = control_frame(CAN_ID_ESTOP, 1);
    CanFrame enable = control_frame(CAN_ID_ENABLE, 1);
    CanFrame disable = control_frame(CAN_ID_ENABLE, 0);

    reset_fixture(0);
    g_error_code = ERR_DRIVER_FAULT;
    board_can_handle_frame(&estop);
    assert(g_enabled == 0);
    assert(mock_motor_enable_calls == 0);

    board_can_handle_frame(&enable);
    assert(g_estop == 0);
    assert(g_enabled == 1);
    assert(g_error_code == ERR_NONE);
    assert(g_state == STATE_IDLE);
    assert(mock_motor_enable_calls == 1);
    assert(mock_stage_calls == 0);
    assert(mock_start_calls == 0);

    board_can_handle_frame(&estop);
    board_can_handle_frame(&disable);
    assert(g_estop == 1);
    assert(g_enabled == 0);
    assert(g_state == STATE_DISABLED);
    assert(mock_motor_disable_calls == 1);
}

static void test_motion_and_homing_are_rejected_during_estop(void)
{
    CanFrame estop = control_frame(CAN_ID_ESTOP, 1);
    CanFrame move = control_frame(BOARD_MOVE_CAN_ID, 0x90);
    CanFrame start = control_frame(CAN_ID_GOAL_CONTROL, GOAL_CONTROL_START);
    CanFrame home = control_frame(CAN_ID_HOMING, HOMING_ALL_AXIS);

    reset_fixture(1);
    move.data[5] = 9;
    move.data[6] = 100;
    start.data[1] = 9;
    home.data[1] = 0;

    board_can_handle_frame(&estop);
    board_can_handle_frame(&move);
    board_can_handle_frame(&start);
    board_can_handle_frame(&home);

    assert(g_estop == 1);
    assert(g_state == STATE_ESTOP);
    assert(mock_stage_calls == 0);
    assert(mock_start_calls == 0);
    assert(mock_homing_calls == 0);
}

static void test_active_goal_ignores_every_command_except_valid_estop(void)
{
    CanFrame cancel = control_frame(CAN_ID_GOAL_CONTROL, GOAL_CONTROL_CANCEL);
    CanFrame disable = control_frame(CAN_ID_ENABLE, 0);
    CanFrame clear = control_frame(CAN_ID_CLEAR_ERROR, HOMING_ALL_AXIS);
    CanFrame home = control_frame(CAN_ID_HOMING, HOMING_ALL_AXIS);
    CanFrame move = control_frame(BOARD_MOVE_CAN_ID, 0x90);
    CanFrame malformed_estop = control_frame(CAN_ID_ESTOP, 1);
    CanFrame estop = control_frame(CAN_ID_ESTOP, 1);

    reset_fixture(1);
    g_motion_active = 1;
    g_state = STATE_MOVING;
    mock_slot_free = 0;
    cancel.data[1] = 42;
    home.data[1] = 0;
    move.data[5] = 42;
    move.data[6] = 100;
    malformed_estop.dlc = 7;

    board_can_handle_frame(&cancel);
    board_can_handle_frame(&disable);
    board_can_handle_frame(&clear);
    board_can_handle_frame(&home);
    board_can_handle_frame(&move);
    board_can_handle_frame(&malformed_estop);

    assert(g_motion_active == 1);
    assert(g_enabled == 1);
    assert(g_estop == 0);
    assert(g_error_code == ERR_NONE);
    assert(g_state == STATE_MOVING);
    assert(mock_cancel_calls == 0);
    assert(mock_trajectory_clear_calls == 0);
    assert(mock_stepper_stop_calls == 0);
    assert(mock_homing_calls == 0);
    assert(mock_motor_disable_calls == 0);
    assert(mock_stage_calls == 0);

    board_can_handle_frame(&estop);
    assert(g_estop == 1);
    assert(g_motion_active == 0);
    assert(g_state == STATE_ESTOP);
    assert(mock_trajectory_clear_calls == 1);
    assert(mock_stepper_stop_calls == 1);
}

int main(void)
{
    test_enabled_estop_holds_torque_and_clears_work();
    test_clear_releases_estop_without_changing_enable();
    test_enable_releases_estop_and_disable_is_explicit();
    test_motion_and_homing_are_rejected_during_estop();
    test_active_goal_ignores_every_command_except_valid_estop();
    puts("board_can estop tests passed");
    return 0;
}
