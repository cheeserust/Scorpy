#include "../Inc/state.h"

volatile uint8_t g_enabled = 0;
volatile uint8_t g_estop = 0;
volatile uint8_t g_state = STATE_INIT;
volatile uint8_t g_error_code = ERR_NONE;
volatile uint8_t g_motion_active = 0;
volatile uint8_t g_homing_active = 0;
volatile uint8_t g_homing_done_bits = 0;
volatile int32_t g_current_step[AXIS_COUNT];
volatile int32_t g_target_step[AXIS_COUNT];
volatile int32_t g_motion_start_step[AXIS_COUNT];
volatile uint32_t global_tick_ms = 0;

uint8_t system_homing_done_bits(void)
{
    return (uint8_t)(g_homing_done_bits & ((1 << AXIS_COUNT) - 1));
}

uint8_t system_enabled_status(void)
{
    return g_enabled ? 1 : 0;
}

uint8_t system_all_homed(void)
{
    return (system_homing_done_bits() == (uint8_t)((1 << AXIS_COUNT) - 1)) ? 1 : 0;
}

uint8_t system_first_moving_axis(void)
{
    if (g_homing_active) {
        for (uint8_t i = 0; i < AXIS_COUNT; i++) {
            if ((g_homing_done_bits & (uint8_t)(1 << i)) == 0) return i;
        }
        return 0;
    }

    if (g_motion_active) {
        for (uint8_t i = 0; i < AXIS_COUNT; i++) {
            if (g_current_step[i] != g_target_step[i]) return i;
        }
        return 0;
    }

    return 255;
}
