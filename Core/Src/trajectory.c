#include "../Inc/trajectory.h"

typedef struct {
    MultiAxisTrajectoryPoint point;  // 조립 중인 다축 궤적 명령
    uint8_t active;                  // 스테이징 진행 여부
    uint8_t next_motor_id;           // 다음에 받아야 할 축 번호
    uint32_t start_ms;               // 스테이징 시작 시간(ms)
} StagingState;

static MultiAxisTrajectoryPoint queue[MULTI_AXIS_QUEUE_SIZE];  // 실행 대기 중인 다축 궤적 큐
static volatile uint8_t queue_head;   // 큐에 새 명령을 넣을 위치
static volatile uint8_t queue_tail;   // 큐에서 다음 명령을 꺼낼 위치
static volatile uint8_t queue_count;  // 큐에 저장된 명령 개수
static StagingState staging;          // CAN으로 나눠 받은 축별 명령 조립 상태

// Board1 local motor 0~3 controls robot arm axes 2~5.
static const int32_t gear_ratio[AXIS_COUNT] = { 20, 50, 30, 120 };  // 각 축 감속비
static const int32_t motor_steps_per_rev[AXIS_COUNT] = { 200, 200, 200, 48 };  // 5축은 7.5도 스텝모터
static const int32_t angle_min_raw[AXIS_COUNT] = {
    -9000, -8000, -9000, -17000
};  // 각 축 최소 각도, 0.01도 단위
static const int32_t angle_max_raw[AXIS_COUNT] = {
    9000, 8000, 9000, 17000
};  // 각 축 최대 각도, 0.01도 단위
static const int32_t home_angle_raw[AXIS_COUNT] = {
    -9000, -8000, -9000, -17000
};  // homing 완료 후 논리 home 각도, 0.01도 단위

static uint8_t any_axis_busy(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axis[i].homing || axis[i].moving || axis[i].seg_total_ms != 0) // 원점복귀/이동/세그먼트 실행 중
        {
            return 1;
        }
    }
    return 0;  // 모든 축이 대기 상태
}

static uint8_t queue_push(const MultiAxisTrajectoryPoint *point)
{
    if (queue_count >= MULTI_AXIS_QUEUE_SIZE) // 큐가 가득 차면 실패
    {
        return 0;
    }
    queue[queue_head] = *point;  // 현재 head 위치에 명령 저장
    queue_head = (uint8_t)((queue_head + 1) % MULTI_AXIS_QUEUE_SIZE);  // 원형 큐 head 증가
    queue_count++;  // 저장된 명령 개수 증가
    return 1;       // 큐 입력 성공
}

static uint8_t queue_pop(MultiAxisTrajectoryPoint *point)
{
    if (queue_count == 0)  // 큐가 비어 있으면 실패
    {
        return 0;
    }
    *point = queue[queue_tail];  // 현재 tail 위치의 명령 복사
    queue_tail = (uint8_t)((queue_tail + 1) % MULTI_AXIS_QUEUE_SIZE);  // 원형 큐 tail 증가
    queue_count--;  // 저장된 명령 개수 감소
    return 1;       // 큐 출력 성공
}

static void staging_clear(void)
{
    staging.active = 0;         // 스테이징 비활성화
    staging.next_motor_id = 0;  // 다음 수신 축 번호 초기화
    staging.start_ms = 0;       // 타임아웃 기준 시간 초기화
}

void trajectory_cancel_staging(void)
{
    staging_clear();  // 조립 중이던 다축 이동 명령 취소
}

void trajectory_clear(void)
{
    queue_head = 0;   // 큐 입력 위치 초기화
    queue_tail = 0;   // 큐 출력 위치 초기화
    queue_count = 0;  // 큐 명령 개수 초기화
    staging_clear();  // 조립 중인 명령도 함께 초기화

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        axis[i].seg_total_ms = 0;              // 현재 실행 중인 세그먼트 시간 초기화
        axis[i].seg_elapsed_ms = 0;            // 세그먼트 진행 시간 초기화
        axis[i].seg_delta_step = 0;            // 이동해야 할 스텝 수 초기화
        axis[i].seg_start_step = axis[i].current_step;  // 시작 위치를 현재 위치로 맞춤
        axis[i].seg_end_step = axis[i].current_step;    // 종료 위치를 현재 위치로 맞춤
        if (!axis[i].homing) axis[i].moving = 0;         // homing 중이 아니면 이동 상태 해제
        axis[i].target_step = axis[i].current_step;      // 목표 위치를 현재 위치로 맞춤
    }
}

uint8_t trajectory_free_count(void)
{
    return (uint8_t)((MULTI_AXIS_QUEUE_SIZE - queue_count) * AXIS_COUNT);  // 남은 다축 큐를 축별 명령 개수로 환산
}

int32_t trajectory_angle_raw_to_step(uint8_t axis_id, int32_t angle_raw)
{
    int64_t step_value;  // 곱셈 중 오버플로우를 줄이기 위한 64비트 중간값
    if (axis_id >= AXIS_COUNT) return 0;  // 잘못된 축 번호는 0스텝 처리
    step_value = (int64_t)angle_raw *
                 gear_ratio[axis_id] *
                 motor_steps_per_rev[axis_id] *
                 MICROSTEP;
    return (int32_t)(step_value / 36000);  // 0.01도 단위 raw 각도를 마이크로스텝 수로 변환
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

int32_t trajectory_axis_home_raw(uint8_t axis_id)
{
    if (axis_id >= AXIS_COUNT) return 0;
    return home_angle_raw[axis_id];
}

uint8_t trajectory_angle_raw_in_limit(uint8_t axis_id, int32_t angle_raw)
{
    if (axis_id >= AXIS_COUNT) return 0;
    if (angle_raw < angle_min_raw[axis_id]) return 0;
    if (angle_raw > angle_max_raw[axis_id]) return 0;
    return 1;
}

static uint8_t target_step_from_cmd(uint8_t axis_id, int32_t target_pos, uint8_t flags, int32_t *target_step)
{
    uint8_t relative = (flags & 0x04) ? 1 : 0;
    uint8_t step_mode = (flags & 0x02) ? 1 : 0;
    int64_t resolved;
    int32_t min_step;
    int32_t max_step;

    if (axis_id >= AXIS_COUNT) return 0;

    resolved = step_mode ? target_pos : trajectory_angle_raw_to_step(axis_id, target_pos);
    if (relative) resolved += axis[axis_id].current_step;
    if (resolved < INT32_MIN || resolved > INT32_MAX) return 0;

    *target_step = (int32_t)resolved;

    min_step = trajectory_angle_raw_to_step(axis_id, angle_min_raw[axis_id]);
    max_step = trajectory_angle_raw_to_step(axis_id, angle_max_raw[axis_id]);
    if (min_step > max_step) {
        int32_t tmp = min_step;
        min_step = max_step;
        max_step = tmp;
    }

    if (*target_step < min_step) return 0;
    if (*target_step > max_step) return 0;
    return 1;
}

uint8_t trajectory_check_staging_timeout(void)
{
    if (!staging.active) return 0;  // 조립 중인 명령이 없으면 정상
    if ((global_tick_ms - staging.start_ms) <= STAGING_TIMEOUT_MS) return 0;  // 제한 시간 이내면 계속 대기

    staging_clear();                    // 시간 초과된 명령 조립 취소
    global_motor_error = ERR_INVALID_CMD;  // 축별 프레임 누락을 잘못된 명령으로 처리
    return 1;                           // 타임아웃 발생
}

uint8_t trajectory_stage_command(const TrajectoryPoint *point)
{
    uint8_t execute = (point->flags & 0x08) ? 1 : 0;    // 실행 플래그
    uint8_t relative = (point->flags & 0x04) ? 1 : 0;   // 상대좌표 플래그
    uint8_t step_mode = (point->flags & 0x02) ? 1 : 0;  // 스텝 직접 명령 플래그
    uint8_t reserved = (point->flags & 0x01) ? 1 : 0;   // 예약 플래그

    if (trajectory_check_staging_timeout()) return TRAJECTORY_STAGE_INVALID;  // 이전 조립 명령이 시간 초과되면 실패

    if (!execute || reserved || point->motor_id >= AXIS_COUNT) {
        staging_clear();  // 지원하지 않는 플래그 또는 축 번호면 조립 취소
        return TRAJECTORY_STAGE_INVALID;  // 잘못된 명령
    }
    if (!relative && !step_mode && !trajectory_angle_raw_in_limit(point->motor_id, point->target_pos)) {
        staging_clear();  // 축별 동작 범위를 벗어난 목표 각도면 조립 취소
        return TRAJECTORY_STAGE_INVALID;  // 잘못된 명령
    }

    if (!staging.active) {
        if (point->motor_id != 0) return TRAJECTORY_STAGE_INVALID;  // 다축 명령은 0번 축부터 순서대로 수신
        staging_clear();  // 새 조립 시작 전 상태 초기화
        staging.active = 1;  // 스테이징 시작
        staging.start_ms = global_tick_ms;  // 타임아웃 기준 시간 저장
        staging.point.duration_5ms = point->duration_5ms;  // 모든 축에 적용할 이동 시간 저장
    } else {
        if (point->motor_id != staging.next_motor_id) {
            staging_clear();  // 예상한 축 순서가 아니면 조립 취소
            return TRAJECTORY_STAGE_INVALID;  // 잘못된 명령 순서
        }
        if (point->duration_5ms != staging.point.duration_5ms) {
            staging_clear();  // 축별 이동 시간이 다르면 동기 이동 불가
            return TRAJECTORY_STAGE_INVALID;  // 잘못된 명령
        }
    }

    staging.point.target_pos[point->motor_id] = point->target_pos;  // 해당 축 목표 위치 저장
    staging.point.speed[point->motor_id] = point->speed;            // 해당 축 속도 필드 저장
    staging.point.flags[point->motor_id] = point->flags;            // 해당 축 제어 플래그 저장
    staging.next_motor_id++;                                        // 다음에 받을 축 번호 증가

    if (staging.next_motor_id < AXIS_COUNT) return TRAJECTORY_STAGE_WAITING;  // 아직 모든 축 명령을 받지 못함

    if (!queue_push(&staging.point)) {
        staging_clear();  // 큐 입력 실패 시 조립 상태 초기화
        return TRAJECTORY_STAGE_QUEUE_FULL;  // 큐 가득 참
    }

    staging_clear();  // 큐에 넣은 뒤 스테이징 상태 초기화
    return TRAJECTORY_STAGE_COMMITTED;  // 다축 명령 조립 및 큐 입력 완료
}

static void start_axis_segment(uint8_t axis_id, const MultiAxisTrajectoryPoint *point, int32_t target_step)
{
    uint16_t duration_ms = (uint16_t)point->duration_5ms * 5;  // 5ms 단위를 실제 ms로 변환
    (void)point->speed[axis_id];  // 현재 보간은 duration 기반이라 speed 필드는 보관만 함

    if (duration_ms == 0) duration_ms = 1;  // 0ms 명령은 최소 1ms로 보정

    axis[axis_id].seg_start_step = axis[axis_id].current_step;  // 세그먼트 시작 위치
    axis[axis_id].seg_end_step = target_step;                   // 세그먼트 종료 위치
    axis[axis_id].seg_delta_step = target_step - axis[axis_id].seg_start_step;  // 총 이동 스텝
    axis[axis_id].seg_total_ms = duration_ms;    // 세그먼트 총 실행 시간
    axis[axis_id].seg_elapsed_ms = 0;            // 세그먼트 경과 시간 초기화
    axis[axis_id].target_step = axis[axis_id].seg_start_step;  // 첫 목표 위치를 시작 위치로 설정
    axis[axis_id].moving = (axis[axis_id].seg_delta_step != 0) ? 1 : 0;  // 이동량이 있으면 moving 표시
}

static void try_start_next_point(void)
{
    MultiAxisTrajectoryPoint point;  // 큐에서 꺼낸 다음 다축 궤적 명령
    int32_t target_step[AXIS_COUNT];

    if (!global_motor_enabled || global_motor_estop || global_motor_error != ERR_NONE) return;
    if (any_axis_busy()) return;  // 어느 축이라도 바쁘면 다음 명령 시작 보류
    if (!queue_pop(&point)) return;  // 대기 중인 명령이 없으면 종료

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (!target_step_from_cmd(i, point.target_pos[i], point.flags[i], &target_step[i])) {
            global_motor_error = ERR_INVALID_CMD;
            return;
        }
    }

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        start_axis_segment(i, &point, target_step[i]);  // 모든 축의 새 세그먼트 동시 시작
    }
}

static uint32_t s_curve_ratio(uint16_t elapsed_ms, uint16_t total_ms)
{
    uint64_t t;
    uint64_t t2;
    uint64_t t3;
    uint64_t s;

    if (total_ms == 0) return 32768;
    if (elapsed_ms >= total_ms) return 32768;

    t = ((uint64_t)elapsed_ms << 15) / total_ms;  // 0~32768 비율값
    t2 = (t * t) >> 15;
    t3 = (t2 * t) >> 15;

    // s(t) = 10t^3 - 15t^4 + 6t^5 = t^3 * (10 - 15t + 6t^2)
    s = t3 * ((10ULL << 15) - (15ULL * t) + (6ULL * t2));
    s >>= 15;

    if (s > 32768) return 32768;
    return (uint32_t)s;
}

void trajectory_1ms_interrupt(void)
{
    try_start_next_point();  // 실행 가능한 다음 궤적 명령 시작

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (axis[i].homing || axis[i].seg_total_ms == 0) continue;  // homing 중이거나 실행 세그먼트가 없으면 건너뜀

        if (axis[i].seg_elapsed_ms < axis[i].seg_total_ms) {
            uint32_t ratio = s_curve_ratio(axis[i].seg_elapsed_ms,
                                           axis[i].seg_total_ms);
            int64_t delta = (int64_t)axis[i].seg_delta_step *
                            (int64_t)ratio;  // 5차 S-curve 진행률에 따른 누적 이동량
            axis[i].target_step = axis[i].seg_start_step +
                                  (int32_t)(delta >> 15);  // 5차 S-curve 보간 목표 위치
            axis[i].seg_elapsed_ms++;  // 1ms 진행
            axis[i].moving = (axis[i].seg_delta_step != 0) ? 1 : 0;  // 이동량이 있으면 moving 유지
        } else {
            axis[i].target_step = axis[i].seg_end_step;  // 마지막 목표 위치를 정확히 종료 위치로 맞춤
            axis[i].seg_total_ms = 0;                    // 세그먼트 완료 표시
            axis[i].seg_elapsed_ms = 0;                  // 경과 시간 초기화
            axis[i].moving = (axis[i].current_step != axis[i].target_step) ? 1 : 0;  // 실제 위치가 남았으면 moving 유지
        }
    }
}
