#include "../Inc/trajectory.h"
#include "../Inc/stepper.h"

typedef struct {
    uint8_t valid;
    uint8_t ready;
    uint8_t started;
    uint8_t goal_id;
    uint8_t received_axis_mask;
    uint16_t duration_ms;
    uint32_t staging_start_ms;
    uint32_t motion_start_ms;
    uint8_t has_motion;
    int32_t target_step[AXIS_COUNT];
} GoalSlot;

#if BOARD_ID == BOARD_ID_BOARD1
static const int32_t gear_ratio[AXIS_COUNT] = {20, 50, 30, 20};
static const int32_t motor_steps_per_rev[AXIS_COUNT] = {200, 200, 200, 200};
/* Axis0 home is intentionally valid at -86.50 degrees. */
static const int32_t min_angle[AXIS_COUNT] = {-8650, -7810, -9150, -9000};
static const int32_t max_angle[AXIS_COUNT] = {9000, 8000, 9000, 18000};
static const int32_t home_angle[AXIS_COUNT] = {-8650, -7810, -9150, -9000};
#else
static const int32_t gear_ratio[AXIS_COUNT] = {20};
static const int32_t motor_steps_per_rev[AXIS_COUNT] = {200};
static const int32_t min_angle[AXIS_COUNT] = {-9000};
static const int32_t max_angle[AXIS_COUNT] = {18000};
static const int32_t home_angle[AXIS_COUNT] = {-9000};
#endif

static GoalSlot g_goal;
static volatile int32_t g_planned_step[AXIS_COUNT];
static uint8_t g_cancelled_goal_valid;
static uint8_t g_cancelled_goal_id;
static uint8_t g_completed_goal_valid;
static uint8_t g_completed_goal_id;
static uint8_t g_timeout_event_pending;
static uint8_t g_timeout_goal_id;
static uint8_t g_timeout_mask;
static uint16_t g_timeout_duration_ms;

static void clear_goal_slot(void)
{
    g_goal.valid = 0;
    g_goal.ready = 0;
    g_goal.started = 0;
    g_goal.goal_id = 0;
    g_goal.received_axis_mask = 0;
    g_goal.duration_ms = 0;
    g_goal.staging_start_ms = 0;
    g_goal.motion_start_ms = 0;
    g_goal.has_motion = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) g_goal.target_step[i] = 0;
}

void trajectory_cancel_staging(void)
{
    if (!g_goal.started) clear_goal_slot();
}

void trajectory_stop_motion(void)
{
    g_motion_active = 0;
    stepper_cancel_motion();
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_target_step[i] = g_current_step[i];
        g_motion_start_step[i] = g_current_step[i];
    }
}

void trajectory_sync_planned_to_current(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) g_planned_step[i] = g_current_step[i];
}

int32_t trajectory_get_planned_step(uint8_t axis_id)
{
    return axis_id < AXIS_COUNT ? g_planned_step[axis_id] : 0;
}

void trajectory_clear(void)
{
    clear_goal_slot();
    trajectory_stop_motion();
    trajectory_sync_planned_to_current();
    g_timeout_event_pending = 0;
}

uint8_t trajectory_goal_slot_free(void)
{
    return (!g_goal.valid && !g_motion_active) ? 1u : 0u;
}

uint16_t trajectory_goal_duration_ms(void)
{
    return g_goal.valid ? g_goal.duration_ms : 0;
}

uint8_t trajectory_goal_mask(void)
{
    return g_goal.valid ? g_goal.received_axis_mask : 0;
}

int32_t angle_to_step(uint8_t axis_id, int32_t angle_raw)
{
    int64_t value;
    if (axis_id >= AXIS_COUNT) return 0;
    value = (int64_t)angle_raw * gear_ratio[axis_id] *
            motor_steps_per_rev[axis_id] * MICROSTEP;
    return (int32_t)(value / 36000);
}

int32_t step_to_angle(uint8_t axis_id, int32_t step)
{
    int64_t value;
    int64_t steps_per_rev;
    if (axis_id >= AXIS_COUNT) return 0;
    value = (int64_t)step * 36000;
    steps_per_rev = (int64_t)gear_ratio[axis_id] *
                    motor_steps_per_rev[axis_id] * MICROSTEP;
    value += value >= 0 ? steps_per_rev / 2 : -(steps_per_rev / 2);
    return (int32_t)(value / steps_per_rev);
}

int32_t get_home_angle(uint8_t axis_id)
{
    return axis_id < AXIS_COUNT ? home_angle[axis_id] : 0;
}

uint8_t trajectory_resolve_target_step(uint8_t axis_id, int32_t target_raw,
                                       uint8_t relative, uint8_t step_mode,
                                       int32_t *target_step)
{
    int64_t resolved;
    int32_t minimum;
    int32_t maximum;
    if (axis_id >= AXIS_COUNT || target_step == 0) return 0;

    resolved = step_mode ? target_raw : angle_to_step(axis_id, target_raw);
    if (relative) resolved += g_planned_step[axis_id];
    if (resolved < INT32_MIN || resolved > INT32_MAX) return 0;
    *target_step = (int32_t)resolved;

    minimum = angle_to_step(axis_id, min_angle[axis_id]);
    maximum = angle_to_step(axis_id, max_angle[axis_id]);
    if (minimum > maximum) {
        int32_t swap = minimum;
        minimum = maximum;
        maximum = swap;
    }
    return (*target_step >= minimum && *target_step <= maximum) ? 1u : 0u;
}

uint8_t trajectory_stage_goal_axis(uint8_t motor_id, int32_t target_step,
                                   uint8_t goal_id, uint16_t duration_ms,
                                   uint8_t *received_axis_mask)
{
    uint8_t motor_bit;
    if (received_axis_mask != 0) *received_axis_mask = 0;
    if (motor_id >= AXIS_COUNT || duration_ms == 0) return GOAL_STAGE_INVALID;
    if (g_motion_active || g_goal.started) return GOAL_STAGE_BUSY;

    if (!g_goal.valid) {
        if (g_cancelled_goal_valid && goal_id == g_cancelled_goal_id) {
            return GOAL_STAGE_INVALID;
        }
        if (g_completed_goal_valid && goal_id == g_completed_goal_id) {
            if (received_axis_mask != 0) {
                *received_axis_mask = (uint8_t)((1u << AXIS_COUNT) - 1u);
            }
            return GOAL_STAGE_DUPLICATE;
        }
        clear_goal_slot();
        g_goal.valid = 1;
        g_goal.goal_id = goal_id;
        g_goal.duration_ms = duration_ms;
        g_goal.staging_start_ms = global_tick_ms;
    } else if (g_goal.goal_id != goal_id) {
        return GOAL_STAGE_BUSY;
    } else if (g_goal.duration_ms != duration_ms) {
        clear_goal_slot();
        return GOAL_STAGE_INVALID;
    }

    motor_bit = (uint8_t)(1u << motor_id);
    if (g_goal.received_axis_mask & motor_bit) {
        if (received_axis_mask != 0) *received_axis_mask = g_goal.received_axis_mask;
        if (g_goal.target_step[motor_id] == target_step) return GOAL_STAGE_DUPLICATE;
        clear_goal_slot();
        return GOAL_STAGE_INVALID;
    }

    g_goal.target_step[motor_id] = target_step;
    g_goal.received_axis_mask |= motor_bit;
    if (received_axis_mask != 0) *received_axis_mask = g_goal.received_axis_mask;

    if (g_goal.received_axis_mask != (uint8_t)((1u << AXIS_COUNT) - 1u)) {
        return GOAL_STAGE_WAITING;
    }

    g_goal.ready = 1;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) g_planned_step[i] = g_goal.target_step[i];
    return GOAL_STAGE_READY;
}

uint8_t trajectory_start_goal(uint8_t goal_id)
{
    if (!g_goal.valid || !g_goal.ready || g_goal.started || g_goal.goal_id != goal_id) return 0;

    g_goal.has_motion = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        g_motion_start_step[i] = g_current_step[i];
        g_target_step[i] = g_goal.target_step[i];
        if (g_target_step[i] != g_current_step[i]) g_goal.has_motion = 1;
    }
    g_goal.started = 1;
    g_goal.motion_start_ms = global_tick_ms;
    stepper_prepare_motion(g_goal.duration_ms);
    g_motion_active = 1;
    g_state = STATE_MOVING;
    return 1;
}

uint8_t trajectory_cancel_goal(uint8_t goal_id)
{
    if (g_goal.valid && g_goal.goal_id != goal_id) return 0;
    trajectory_stop_motion();
    clear_goal_slot();
    trajectory_sync_planned_to_current();
    g_cancelled_goal_valid = 1;
    g_cancelled_goal_id = goal_id;
    if (g_enabled && !ESTOP_ACTIVE() && g_error_code == ERR_NONE) g_state = STATE_IDLE;
    return 1;
}

uint8_t trajectory_handle_staging_timeout(void)
{
    if (!g_goal.valid || g_goal.ready || g_goal.started) return 0;
    if ((global_tick_ms - g_goal.staging_start_ms) <= STAGING_TIMEOUT_MS) return 0;

    g_timeout_event_pending = 1;
    g_timeout_goal_id = g_goal.goal_id;
    g_timeout_mask = g_goal.received_axis_mask;
    g_timeout_duration_ms = g_goal.duration_ms;
    clear_goal_slot();
    return 1;
}

uint8_t trajectory_take_timeout_event(uint8_t *goal_id, uint8_t *mask,
                                      uint16_t *duration_ms)
{
    if (!g_timeout_event_pending) return 0;
    if (goal_id != 0) *goal_id = g_timeout_goal_id;
    if (mask != 0) *mask = g_timeout_mask;
    if (duration_ms != 0) *duration_ms = g_timeout_duration_ms;
    g_timeout_event_pending = 0;
    return 1;
}

void trajectory_1ms_interrupt(void)
{
    uint8_t reached = 1;
    if (!g_motion_active || !g_goal.started) return;

    /* START locks the goal. Runtime state/error changes must not cancel it;
     * only E-stop is allowed to interrupt active motion. */
    if (ESTOP_ACTIVE()) {
        trajectory_stop_motion();
        return;
    }

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (g_current_step[i] != g_goal.target_step[i]) {
            reached = 0;
            break;
        }
    }
    if (!reached) return;
    if (!g_goal.has_motion &&
        (global_tick_ms - g_goal.motion_start_ms) < g_goal.duration_ms) return;

    g_completed_goal_valid = 1;
    g_completed_goal_id = g_goal.goal_id;
    g_motion_active = 0;
    stepper_cancel_motion();
    clear_goal_slot();
    trajectory_sync_planned_to_current();
    g_state = STATE_IDLE;
}
