#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "config.h"

void trajectory_clear(void);
void trajectory_cancel_staging(void);
uint8_t trajectory_stage_command(const TrajectoryPoint *point);
uint8_t trajectory_check_staging_timeout(void);
uint8_t trajectory_free_count(void);
void trajectory_update_1ms(void);
int32_t trajectory_angle_raw_to_step(uint8_t axis_id, int32_t angle_raw);
int32_t trajectory_axis_home_raw(uint8_t axis_id);
uint8_t trajectory_angle_raw_in_limit(uint8_t axis_id, int32_t angle_raw);

#define TRAJECTORY_STAGE_WAITING     0
#define TRAJECTORY_STAGE_COMMITTED   1
#define TRAJECTORY_STAGE_INVALID     2
#define TRAJECTORY_STAGE_QUEUE_FULL  3

#endif
