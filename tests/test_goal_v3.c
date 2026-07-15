#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#define TRAJECTORY_H
#define STEPPER_H
#define AXIS_COUNT 4
#define BOARD_ID 1
#define BOARD_ID_BOARD1 1
#define BOARD_ID_BOARD2 2
#define MICROSTEP 16
#define STAGING_TIMEOUT_MS 100
#define STATE_IDLE 1
#define STATE_MOVING 3
#define ERR_NONE 0
#define GOAL_STAGE_WAITING 0
#define GOAL_STAGE_READY 1
#define GOAL_STAGE_INVALID 2
#define GOAL_STAGE_DUPLICATE 3
#define GOAL_STAGE_BUSY 4
#define ESTOP_ACTIVE() 0

volatile uint8_t g_enabled = 1;
volatile uint8_t g_state = STATE_IDLE;
volatile uint8_t g_error_code = ERR_NONE;
volatile uint8_t g_motion_active;
volatile uint8_t g_homing_active;
volatile int32_t g_current_step[AXIS_COUNT];
volatile int32_t g_target_step[AXIS_COUNT];
volatile int32_t g_motion_start_step[AXIS_COUNT];
volatile uint32_t global_tick_ms;

static uint16_t prepared_duration;
void stepper_cancel_motion(void) { }
void stepper_prepare_motion(uint16_t duration_ms) { prepared_duration = duration_ms; }
uint8_t system_all_homed(void) { return 1; }

#include "../Core/Src/trajectory.c"

static uint8_t stage(uint8_t axis, int32_t target, uint8_t goal_id,
                     uint16_t duration, uint8_t *mask)
{
    return trajectory_stage_goal_axis(axis, target, goal_id, duration, mask);
}

static void test_unordered_axes_become_ready(void)
{
    uint8_t mask = 0;
    trajectory_clear();
    assert(stage(2, 300, 7, 5000, &mask) == GOAL_STAGE_WAITING);
    assert(mask == 0x04);
    assert(stage(0, 100, 7, 5000, &mask) == GOAL_STAGE_WAITING);
    assert(stage(3, 400, 7, 5000, &mask) == GOAL_STAGE_WAITING);
    assert(stage(1, 200, 7, 5000, &mask) == GOAL_STAGE_READY);
    assert(mask == 0x0F);
    assert(!trajectory_goal_slot_free());
    assert(trajectory_start_goal(7));
    assert(g_motion_active && g_state == STATE_MOVING);
    assert(prepared_duration == 5000);
}

static void test_duplicate_and_conflict(void)
{
    uint8_t mask = 0;
    trajectory_clear();
    assert(stage(1, 200, 8, 1000, &mask) == GOAL_STAGE_WAITING);
    assert(stage(1, 200, 8, 1000, &mask) == GOAL_STAGE_DUPLICATE);
    assert(stage(1, 201, 8, 1000, &mask) == GOAL_STAGE_INVALID);
    assert(trajectory_goal_slot_free());
}

static void test_other_goal_is_busy(void)
{
    uint8_t mask = 0;
    trajectory_clear();
    assert(stage(0, 100, 9, 2000, &mask) == GOAL_STAGE_WAITING);
    assert(stage(0, 100, 10, 2000, &mask) == GOAL_STAGE_BUSY);
    assert(trajectory_goal_mask() == 0x01);
}

static void test_timeout_is_nonfatal(void)
{
    uint8_t mask = 0;
    uint8_t event_goal = 0;
    uint8_t event_mask = 0;
    uint16_t event_duration = 0;
    trajectory_clear();
    global_tick_ms = 10;
    assert(stage(3, 400, 11, 3000, &mask) == GOAL_STAGE_WAITING);
    global_tick_ms = 111;
    assert(trajectory_handle_staging_timeout());
    assert(trajectory_take_timeout_event(&event_goal, &event_mask, &event_duration));
    assert(event_goal == 11 && event_mask == 0x08 && event_duration == 3000);
    assert(g_error_code == ERR_NONE && trajectory_goal_slot_free());
}

static void test_cancel_tombstones_late_goal(void)
{
    uint8_t mask = 0;
    trajectory_clear();
    assert(trajectory_cancel_goal(12));
    assert(stage(0, 100, 12, 1000, &mask) == GOAL_STAGE_INVALID);
    assert(stage(0, 100, 13, 1000, &mask) == GOAL_STAGE_WAITING);
}

static void test_zero_distance_goal_holds_duration(void)
{
    uint8_t mask = 0;
    trajectory_clear();
    global_tick_ms = 1000;
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        uint8_t expected = axis == AXIS_COUNT - 1 ? GOAL_STAGE_READY : GOAL_STAGE_WAITING;
        assert(stage(axis, g_current_step[axis], 14, 50, &mask) == expected);
    }
    assert(trajectory_start_goal(14));
    global_tick_ms = 1049;
    trajectory_1ms_interrupt();
    assert(g_motion_active);
    global_tick_ms = 1050;
    trajectory_1ms_interrupt();
    assert(!g_motion_active && g_state == STATE_IDLE && trajectory_goal_slot_free());
}

static void test_started_goal_ignores_non_estop_runtime_state_changes(void)
{
    uint8_t mask = 0;

    trajectory_clear();
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        uint8_t expected = axis == AXIS_COUNT - 1 ? GOAL_STAGE_READY : GOAL_STAGE_WAITING;
        assert(stage(axis, 1000 + axis, 15, 5000, &mask) == expected);
    }
    assert(trajectory_start_goal(15));

    g_enabled = 0;
    g_error_code = 99;
    g_homing_active = 1;
    trajectory_1ms_interrupt();

    assert(g_motion_active);
    assert(!trajectory_goal_slot_free());

    g_enabled = 1;
    g_error_code = ERR_NONE;
    g_homing_active = 0;
}

int main(void)
{
    test_unordered_axes_become_ready();
    test_duplicate_and_conflict();
    test_other_goal_is_busy();
    test_timeout_is_nonfatal();
    test_cancel_tombstones_late_goal();
    test_zero_distance_goal_holds_duration();
    test_started_goal_ignores_non_estop_runtime_state_changes();
    puts("goal_v3 tests passed");
    return 0;
}
