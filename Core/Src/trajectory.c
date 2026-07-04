#include "../Inc/trajectory.h"
#include "../Inc/stepper.h"

typedef struct {
    uint8_t active;
    uint8_t expected_motor_id;
    uint8_t duration_5ms;
    uint32_t start_ms;
    TrajectoryPoint point;
} StagingState;

#if BOARD_ID == 1
static const int32_t gear_ratio[AXIS_COUNT] = { 20, 50, 30, 120 };
static const int32_t motor_steps_per_rev[AXIS_COUNT] = { 200, 200, 200, 48 };
static const int32_t angle_min[AXIS_COUNT] = { -9000, -8000, -9000, -17000 };
static const int32_t angle_max[AXIS_COUNT] = { 9000, 8000, 9000, 17000 };
static const int32_t home_angle[AXIS_COUNT] = { -9000, -8000, -9000, -17000 };
#elif BOARD_ID == 2
static const int32_t gear_ratio[AXIS_COUNT] = { 20 };
static const int32_t motor_steps_per_rev[AXIS_COUNT] = { 200 };
static const int32_t angle_min[AXIS_COUNT] = { -9000 };
static const int32_t angle_max[AXIS_COUNT] = { 18000 };
static const int32_t home_angle[AXIS_COUNT] = { -9000 };
#endif

static StagingState g_staging;
static TrajectoryPoint g_queue[TRAJECTORY_POINT_QUEUE_SIZE];
static volatile uint8_t g_q_head;
static volatile uint8_t g_q_tail;
static volatile uint8_t g_q_count;

static TrajectoryPoint g_active_point;

static uint8_t queue_push(const TrajectoryPoint *point)
{
    if (g_q_count >= TRAJECTORY_POINT_QUEUE_SIZE) return 0;

    g_queue[g_q_tail] = *point;
    g_q_tail = (uint8_t)((g_q_tail + 1) % TRAJECTORY_POINT_QUEUE_SIZE);
    g_q_count++;
    return 1;
}

static uint8_t queue_pop(TrajectoryPoint *point)
{
    if (g_q_count == 0) return 0;

    *point = g_queue[g_q_head];
    g_q_head = (uint8_t)((g_q_head + 1) % TRAJECTORY_POINT_QUEUE_SIZE);
    g_q_count--;
    return 1;
}

static void point_clear(TrajectoryPoint *point)
{
    point->duration_ms = 0;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        point->target_step[i] = 0;
        point->speed[i] = 0;
    }
}

static void staging_reset(void)
{
    g_staging.active = 0;
    g_staging.expected_motor_id = 0;
    g_staging.duration_5ms = 0;
    g_staging.start_ms = 0;
    point_clear(&g_staging.point);
}

void trajectory_cancel_staging(void)
{
    staging_reset();
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

void trajectory_clear(void)
{
    g_q_head = 0;
    g_q_tail = 0;
    g_q_count = 0;
    staging_reset();
    trajectory_stop_motion();
}

uint8_t get_free_axis_command_count(void)
{
    return (uint8_t)((TRAJECTORY_POINT_QUEUE_SIZE - g_q_count) * BOARD_STAGING_FRAME_COUNT);
}

int32_t angle_to_step(uint8_t axis_id, int32_t angle_raw)
{
    int64_t step_value;

    if (axis_id >= AXIS_COUNT) return 0;
    step_value = (int64_t)angle_raw *
                 gear_ratio[axis_id] *
                 motor_steps_per_rev[axis_id] *
                 MICROSTEP;
    return (int32_t)(step_value / 36000);
}

int32_t step_to_angle(uint8_t axis_id, int32_t step)
{
    int64_t angle_value;
    int64_t steps_per_output_rev;

    if (axis_id >= AXIS_COUNT) return 0;

    angle_value = (int64_t)step * 36000;
    steps_per_output_rev = (int64_t)gear_ratio[axis_id] *
                           motor_steps_per_rev[axis_id] *
                           MICROSTEP;

    if (angle_value >= 0) angle_value += steps_per_output_rev / 2;
    else angle_value -= steps_per_output_rev / 2;

    return (int32_t)(angle_value / steps_per_output_rev);
}

int32_t get_home_angle(uint8_t axis_id)
{
    if (axis_id >= AXIS_COUNT) return 0;
    return home_angle[axis_id];
}

uint8_t trajectory_resolve_target_step(uint8_t axis_id,
                                       int32_t target_raw,
                                       uint8_t relative,
                                       uint8_t step_mode,
                                       int32_t *target_step)
{
    int64_t resolved;
    int32_t min_step;
    int32_t max_step;

    if (axis_id >= AXIS_COUNT || target_step == 0) return 0;

    resolved = step_mode ? target_raw : angle_to_step(axis_id, target_raw);
    if (relative) resolved += g_current_step[axis_id];
    if (resolved < INT32_MIN || resolved > INT32_MAX) return 0;

    *target_step = (int32_t)resolved;

    min_step = angle_to_step(axis_id, angle_min[axis_id]);
    max_step = angle_to_step(axis_id, angle_max[axis_id]);
    if (min_step > max_step) {
        int32_t tmp = min_step;
        min_step = max_step;
        max_step = tmp;
    }

    if (*target_step < min_step) return 0;
    if (*target_step > max_step) return 0;
    return 1;
}

uint8_t trajectory_handle_staging_timeout(void)
{
    if (!g_staging.active) return 0;
    if ((global_tick_ms - g_staging.start_ms) <= STAGING_TIMEOUT_MS) return 0;

    staging_reset();
    trajectory_stop_motion();
    g_error_code = ERR_INVALID_CMD;
    g_state = STATE_ERROR;
    return 1;
}

static uint16_t duration_5ms_to_ms(uint8_t duration_5ms)
{
    uint16_t duration_ms = (uint16_t)duration_5ms * 5;
    return duration_ms == 0 ? 1 : duration_ms;
}

uint8_t trajectory_stage_axis(uint8_t motor_id, int32_t target_step, uint16_t speed, uint8_t duration_5ms)
{
    if (trajectory_handle_staging_timeout()) return TRAJECTORY_STAGING_INVALID;
    if (motor_id >= AXIS_COUNT) {
        staging_reset();
        return TRAJECTORY_STAGING_INVALID;
    }

    if (BOARD_STAGING_FRAME_COUNT == 1) {
        TrajectoryPoint point;

        if (motor_id != 0) return TRAJECTORY_STAGING_INVALID;
        point_clear(&point);
        point.duration_ms = duration_5ms_to_ms(duration_5ms);
        point.target_step[0] = target_step;
        point.speed[0] = speed;
        if (!queue_push(&point)) return TRAJECTORY_STAGING_QUEUE_FULL;
        return TRAJECTORY_STAGING_COMMITTED;
    }

    if (!g_staging.active) {
        if (motor_id != 0) return TRAJECTORY_STAGING_INVALID;

        staging_reset();
        g_staging.active = 1;
        g_staging.expected_motor_id = 0;
        g_staging.duration_5ms = duration_5ms;
        g_staging.start_ms = global_tick_ms;
        g_staging.point.duration_ms = duration_5ms_to_ms(duration_5ms);
    } else {
        if (motor_id != g_staging.expected_motor_id) {
            staging_reset();
            return TRAJECTORY_STAGING_INVALID;
        }
        if (duration_5ms != g_staging.duration_5ms) {
            staging_reset();
            return TRAJECTORY_STAGING_INVALID;
        }
    }

    g_staging.point.target_step[motor_id] = target_step;
    g_staging.point.speed[motor_id] = speed;
    g_staging.expected_motor_id++;

    if (g_staging.expected_motor_id < BOARD_STAGING_FRAME_COUNT) return TRAJECTORY_STAGING_WAITING;

    if (!queue_push(&g_staging.point)) {
        staging_reset();
        return TRAJECTORY_STAGING_QUEUE_FULL;
    }

    staging_reset();
    return TRAJECTORY_STAGING_COMMITTED;
}

static uint8_t motion_allowed(void)
{
    if (!g_enabled) return 0;
    if (g_estop) return 0;
    if (g_error_code != ERR_NONE) return 0;
    if (g_homing_active) return 0;
    if (!system_all_homed()) return 0;
    return 1;
}

void trajectory_1ms_interrupt(void)
{
    TrajectoryPoint point;
    uint8_t reached;

    // 1. 모션 허용 상태 체크
    if (!motion_allowed()) {
        if (g_motion_active) {
            trajectory_stop_motion();
        }
        return;
    }

    // 2. 모션이 비활성화 상태라면 즉시 큐에서 다음 명령 인출
    if (!g_motion_active) {
        if (queue_pop(&point)) {
            g_active_point = point;

            for (uint8_t i = 0; i < AXIS_COUNT; i++) {
                g_motion_start_step[i] = g_current_step[i];
                g_target_step[i] = point.target_step[i];
            }

            stepper_prepare_motion(point.duration_ms == 0 ? 1 : point.duration_ms);
            g_motion_active = 1;
            g_state = STATE_MOVING;
        } else if (g_state == STATE_MOVING) {
            g_state = STATE_IDLE;
        }
    }

    // 큐에서 새 모션을 시작하지 못했다면(큐가 비었음) 즉시 탈출
    if (!g_motion_active) return;


    // 3. 목표 도달 여부 체크 (새 모션 시작 직후 바로 체크 가능하도록 순서 유지)
    reached = 1;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (g_current_step[i] != g_active_point.target_step[i]) {
            reached = 0;
            break; // 한 축이라도 덜 갔으면 즉시 탈출
        }
    }

    // 4. 도달 완료 처리
    if (reached) {
        for (uint8_t i = 0; i < AXIS_COUNT; i++) {
            g_target_step[i] = g_active_point.target_step[i];
        }
        g_motion_active = 0;
        stepper_cancel_motion();
        if (g_q_count == 0) {
            g_state = STATE_IDLE;
        } else if (queue_pop(&point)) {
            g_active_point = point;

            for (uint8_t i = 0; i < AXIS_COUNT; i++) {
                g_motion_start_step[i] = g_current_step[i];
                g_target_step[i] = point.target_step[i];
            }

            stepper_prepare_motion(point.duration_ms == 0 ? 1 : point.duration_ms);
            g_motion_active = 1;
            g_state = STATE_MOVING;
        }
    }
}
