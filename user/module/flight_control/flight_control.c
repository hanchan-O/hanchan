#include "flight_control.h"
#include "motor.h"

/* ========== ?????? ========== */

Attitude_Est_t attitude = {0};

/* Q15 ?? cos9 ??0~180 ??? 22.5 ???
 * idx0=0??idx4=90??idx8=180????? */
static const int16_t COS_Q15_15[9] = {
    30784, 25133,  16384,
    11207,     0, -11207,
   -16384,-25133,-30784
};

/* ????? */
uint8_t  sm_idx = 0;           /* 0..8 ?????? */
int8_t   sm_dir = 1;           /* +1 ?? / -1 ?? */
uint32_t sm_next_tick = 0;     /* ????? [ms] */
uint8_t  thr = 0;              /* ?????? main ?? */

#define FLAP_CENTER_OFFSET 200  /* ?????????? */

/* ========== ???? ========== */

int16_t constrain(int16_t val, int16_t min, int16_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/* ??????? */
void reset_flap_state(void)
{
    sm_idx = 0;
    sm_dir = 1;
    sm_next_tick = HAL_GetTick();
}

/**
 * ??????(Wings_motor[1])?(Wings_motor[3])???????
 */
void Estimate_Attitude(void)
{
    attitude.avg_position = (Wings_Data.Wings_motor[1].Corrective_Angle +
                             Wings_Data.Wings_motor[3].Corrective_Angle) / 2;

    attitude.sync_error[1] = Wings_Data.Wings_motor[1].Corrective_Angle - attitude.avg_position;
    attitude.sync_error[3] = Wings_Data.Wings_motor[3].Corrective_Angle - attitude.avg_position;
    attitude.sync_error[0] = 0;
    attitude.sync_error[2] = 0;

    attitude.roll = Wings_Data.Wings_motor[1].Corrective_Angle -
                    Wings_Data.Wings_motor[3].Corrective_Angle;

    attitude.pitch = 0;

    int32_t variance = 0;
    variance += (int64_t)attitude.sync_error[1] * attitude.sync_error[1];
    variance += (int64_t)attitude.sync_error[3] * attitude.sync_error[3];
    attitude.lift_balance = (int16_t)(variance / 2);
}

/**
 * ????????????? Target_Angle???????
 */
void Motor_Sync_Compensate(void)
{
    int16_t max_diff = 0;
    int16_t abs_diff_1 = abs16_fast(attitude.sync_error[1]);
    int16_t abs_diff_3 = abs16_fast(attitude.sync_error[3]);
    max_diff = (abs_diff_1 > abs_diff_3) ? abs_diff_1 : abs_diff_3;

    if (max_diff > 50) {
        const int16_t SYNC_GAIN = 3;
        /* ?????(sm_idx<=1 ? >=7)??????? */
        int16_t sync_scale = (sm_idx <= 1 || sm_idx >= 7) ? 5 : 10;
        int16_t compensation_1 = (attitude.sync_error[1] * SYNC_GAIN * sync_scale) / 100;
        int16_t compensation_3 = (attitude.sync_error[3] * SYNC_GAIN * sync_scale) / 100;
        Wings_Data.Wings_motor[1].Target_Angle -= compensation_1;
        Wings_Data.Wings_motor[3].Target_Angle -= compensation_3;
    }
}

/**
 * ?????????? base_amp? -100~+100 ?? turn_ratio???? bias_angle?????? Execute_Flap_Step
 */
void Calculate_Dynamic_Turn(int16_t yaw_input, uint8_t throttle, TurnControl_t* turn_data)
{
    turn_data->base_amp = 600 + (int16_t)(throttle - FLAP_THR_MAP_MIN) * 60;

    float input_norm = (float)constrain(yaw_input, -100, 100) / 100.0f;

    float turn_curve = input_norm * fabsf(input_norm);

    turn_data->turn_ratio = (int16_t)(turn_curve * 100);

    turn_data->phase_diff = (int16_t)(fabsf(turn_curve) * 2);

    turn_data->bias_angle = (int16_t)(turn_curve * 30);
}

uint32_t Calculate_Flap_Step_Time(uint8_t throttle)
{
    if (throttle < FLAP_THR_MAP_MIN)
        throttle = FLAP_THR_MAP_MIN;
    if (throttle > FLAP_THR_MAP_MAX)
        throttle = FLAP_THR_MAP_MAX;

    uint32_t thr_span = (uint32_t)(FLAP_THR_MAP_MAX - FLAP_THR_MAP_MIN);
    if (thr_span == 0U)
        thr_span = 1U;

    /* ??? f_cHz??? [FLAP_FREQ_MIN_cHz, FLAP_FREQ_MAX_cHz] */
    uint32_t f_cHz = (uint32_t)FLAP_FREQ_MIN_cHz
        + (uint32_t)(throttle - FLAP_THR_MAP_MIN)
            * (uint32_t)(FLAP_FREQ_MAX_cHz - FLAP_FREQ_MIN_cHz) / thr_span;

    if (f_cHz == 0U)
        f_cHz = 1U;

    /* ?? ms = 1000/f_hz = 100000/f_cHz????? FLAP_STEPS_PER_CYCLE */
    uint32_t n = (uint32_t)FLAP_STEPS_PER_CYCLE * f_cHz;
    uint32_t step_ms = (100000U + n / 2U) / n;
    if (step_ms < 1U)
        step_ms = 1U;
    return step_ms;
}

/**
 * ??????????????????????? Target_Angle
 */
void Execute_Flap_Step(int16_t yaw_input,
                       TurnControl_t* turn_ctrl,
                       int16_t motor_front_L_ready,
                       int16_t motor_front_R_ready,
                       int16_t motor_back_L_ready,
                       int16_t motor_back_R_ready)
{
    int8_t phase_offset = (yaw_input < 0) ? -turn_ctrl->phase_diff : turn_ctrl->phase_diff;

    int16_t amp_diff = (turn_ctrl->turn_ratio * 25) / 10;
    int16_t ampR = turn_ctrl->base_amp - amp_diff;
    int16_t ampL = turn_ctrl->base_amp + amp_diff;

    const int16_t MIN_AMP = 500;
    const int16_t MAX_AMP = 1200;
    ampR = constrain(ampR, MIN_AMP, MAX_AMP);
    ampL = constrain(ampL, MIN_AMP, MAX_AMP);

    /* ???????????????????? */
    if (sm_idx <= 1 || sm_idx >= 7) {
        const int16_t FLAP_EDGE_Q8 = 220; /* ?? 256??0.86 */
        ampR = (int16_t)(((int32_t)ampR * FLAP_EDGE_Q8) >> 8);
        ampL = (int16_t)(((int32_t)ampL * FLAP_EDGE_Q8) >> 8);
        ampR = constrain(ampR, MIN_AMP, MAX_AMP);
        ampL = constrain(ampL, MIN_AMP, MAX_AMP);
    }

    int8_t idxR = sm_idx;
    int8_t idxL = sm_idx + phase_offset;

    if (idxL < 0) idxL = 0;
    if (idxL > 8) idxL = 8;

    int16_t cR = COS_Q15_15[idxR];
    int16_t cL = COS_Q15_15[idxL];

    int16_t biasR = -turn_ctrl->bias_angle;
    int16_t biasL = turn_ctrl->bias_angle;

    Wings_Data.Wings_motor[3].Target_Angle = (int16_t)(motor_back_R_ready + biasR + FLAP_CENTER_OFFSET - q15_mul(ampR, cR));
    Wings_Data.Wings_motor[1].Target_Angle = (int16_t)(motor_back_L_ready + biasL + FLAP_CENTER_OFFSET - q15_mul(ampL, cL));
    Wings_Data.Wings_motor[0].Target_Angle = motor_front_R_ready;
    Wings_Data.Wings_motor[2].Target_Angle = motor_front_L_ready;

    if (sm_dir > 0) {
        if (++sm_idx >= 8) { sm_idx = 8; sm_dir = -1; }
    } else {
        if (sm_idx-- == 0) { sm_idx = 0; sm_dir = +1; }
    }
}
