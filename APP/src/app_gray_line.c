/**
 * @file    app_gray_line.c
 * @brief   Gray sensor line-following control task.
 */
#include "app_gray_line.h"
#include "app_gray.h"
#include "app_imu.h"
#include "app_motor_encoder.h"
#include "app_motion.h"
#include "app_tb6612.h"
#include "app_drive_mode.h"
#include "dev_tb6612.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define GRAYLINE_TASK_PERIOD_MS      10U
#define GRAYLINE_TELEMETRY_DIVIDER   2U
#define GRAYLINE_TARGET_POS          0.0f
#define GRAYLINE_WHEEL_CIRCUM_M      (3.14159265358979323846f * 0.048f)
#define GRAYLINE_WHEEL_BASE_M        0.125f
#define GRAYLINE_RAD_TO_DEG          57.2957795130823208768f
#define GRAYLINE_GYRO_Z_SIGN         1.0f
#define GRAYLINE_RATE_LP_ALPHA       0.35f
#define GRAYLINE_SPEED_I_LIMIT       0.40f
#define GRAYLINE_POS_I_LIMIT         20.0f
#define GRAYLINE_RATE_I_LIMIT        400.0f
#define GRAYLINE_MAX_WHEEL_SPEED_MPS 0.80f
#define GRAYLINE_BASE_PWM_AT_0P12MPS 25.0f
#define GRAYLINE_REF_MPS             0.12f

QueueHandle_t g_grayLineCmdQueue = NULL;
GrayLine_PidConfig_t g_grayLinePid = {
    0.016f,
    0.0f,
    0.001f,
    0.20f,
    0.08f,
    300.0f,
    4.0f,
    0.06f,
    0.0008f,
    0.0f,
    0.0f,
    180.0f,
    0.060f
};

static GrayLine_Status_t s_grayLineStatus = {0};

typedef struct {
    bool running;
    bool path_assist;
    bool rate_loop_enabled;
    bool last_line_valid;
    bool speed_ref_ready;
    float pos_i;
    float pos_prev_error;
    float rate_i;
    float rate_prev_error;
    float target_rate_dps;
    float actual_rate_dps;
    bool rate_filter_ready;
    uint8_t telemetry_divider;
    float left_speed_i;
    float right_speed_i;
    float left_speed_prev_error;
    float right_speed_prev_error;
    float last_line_pos;
    int64_t speed_prev_left_total;
    int64_t speed_prev_right_total;
    uint32_t speed_prev_seq;
    TickType_t lost_since_tick;
    int16_t last_left_pwm;
    int16_t last_right_pwm;
} GrayLine_Runtime_t;

static float clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int16_t clamp_i16(int16_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float abs_f32(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float slew_f32(float current, float target, float max_step)
{
    float delta = target - current;

    if (delta > max_step) {
        return current + max_step;
    }
    if (delta < -max_step) {
        return current - max_step;
    }
    return target;
}

static float grayline_lowpass(float input, float previous, float alpha)
{
    return previous + (alpha * (input - previous));
}

static float grayline_turn_to_rate_dps(float turn_mps)
{
    return (2.0f * turn_mps * GRAYLINE_RAD_TO_DEG) / GRAYLINE_WHEEL_BASE_M;
}

static bool grayline_read_yaw_rate_dps(float *rate_dps)
{
    IMU_Data_t imu;

    if (rate_dps == NULL || !IMU_Data_Read(&imu) || !imu.gyro_z_valid) {
        return false;
    }

    *rate_dps = imu.gyro_z_dps * GRAYLINE_GYRO_Z_SIGN;
    return true;
}

static float grayline_rate_pid_step(GrayLine_Runtime_t *rt,
                                    float target_rate_dps,
                                    float actual_rate_dps,
                                    float dt,
                                    float *error_out)
{
    float error;
    float candidate_i;
    float d = 0.0f;
    float unsat;
    float trim;

    if (rt == NULL || dt <= 0.0f) {
        if (error_out != NULL) {
            *error_out = 0.0f;
        }
        return 0.0f;
    }

    error = target_rate_dps - actual_rate_dps;
    candidate_i = clamp_f32(rt->rate_i + (error * dt),
                            -GRAYLINE_RATE_I_LIMIT,
                            GRAYLINE_RATE_I_LIMIT);
    d = (error - rt->rate_prev_error) / dt;
    unsat = (g_grayLinePid.rate_kp * error) +
            (g_grayLinePid.rate_ki * candidate_i) +
            (g_grayLinePid.rate_kd * d);
    trim = clamp_f32(unsat,
                     -g_grayLinePid.rate_trim_mps_max,
                     g_grayLinePid.rate_trim_mps_max);

    /* Do not accumulate more integral in the direction of a saturated trim. */
    if (!((unsat > g_grayLinePid.rate_trim_mps_max && error > 0.0f) ||
          (unsat < -g_grayLinePid.rate_trim_mps_max && error < 0.0f))) {
        rt->rate_i = candidate_i;
    }
    rt->rate_prev_error = error;

    if (error_out != NULL) {
        *error_out = error;
    }
    return trim;
}

static void grayline_send_motor_cmd(MotorCmdType type, int16_t val, TickType_t wait_ticks)
{
    MotorCmd cmd;

    if (g_motorCmdQueue == NULL) {
        return;
    }

    cmd.type = type;
    cmd.val = val;
    (void)xQueueSend(g_motorCmdQueue, &cmd, wait_ticks);
}

static void grayline_stop_motors(void)
{
    if (!DriveMode_Owns(DRIVE_MODE_GRAYLINE)) {
        return;
    }
    grayline_send_motor_cmd(MOTOR_CMD_LEFT_SIGNED_SPEED, 0, pdMS_TO_TICKS(20));
    grayline_send_motor_cmd(MOTOR_CMD_RIGHT_SIGNED_SPEED, 0, pdMS_TO_TICKS(20));
    grayline_send_motor_cmd(MOTOR_CMD_ONOFF, 0, pdMS_TO_TICKS(20));
}

static void grayline_drive(int16_t left_pwm, int16_t right_pwm)
{
    if (!DriveMode_Owns(DRIVE_MODE_GRAYLINE)) {
        return;
    }
    grayline_send_motor_cmd(MOTOR_CMD_LEFT_SIGNED_SPEED, clamp_i16(left_pwm, -100, 100), pdMS_TO_TICKS(2));
    grayline_send_motor_cmd(MOTOR_CMD_RIGHT_SIGNED_SPEED, clamp_i16(right_pwm, -100, 100), pdMS_TO_TICKS(2));
    grayline_send_motor_cmd(MOTOR_CMD_ONOFF, 1, pdMS_TO_TICKS(2));
}

static float grayline_target_to_pwm(float target_mps)
{
    float pwm = (abs_f32(target_mps) / GRAYLINE_REF_MPS) *
                GRAYLINE_BASE_PWM_AT_0P12MPS;

    if (target_mps < 0.0f) {
        pwm = -pwm;
    }
    return pwm;
}

static float grayline_counts_to_mps(int32_t counts, float counts_per_rev, float dt)
{
    if (dt <= 0.0f) {
        return 0.0f;
    }
    return (((float)counts / counts_per_rev) * GRAYLINE_WHEEL_CIRCUM_M) / dt;
}

static bool grayline_read_wheel_speeds(GrayLine_Runtime_t *rt,
                                       float *actual_left_mps,
                                       float *actual_right_mps,
                                       float *dt_out)
{
    MotorEncoderSnapshot_t enc;
    uint32_t frame_count;
    int32_t left_counts;
    int32_t right_counts;
    float speed_dt;

    if (rt == NULL || actual_left_mps == NULL ||
        actual_right_mps == NULL || dt_out == NULL) {
        return false;
    }

    if (!MotorEncoder_ReadSnapshot(&enc)) {
        return false;
    }

    if (!rt->speed_ref_ready) {
        rt->speed_prev_left_total = enc.left_total_counts;
        rt->speed_prev_right_total = enc.right_total_counts;
        rt->speed_prev_seq = enc.seq;
        rt->speed_ref_ready = true;
        return false;
    }

    if (enc.seq == rt->speed_prev_seq) {
        return false;
    }

    frame_count = enc.seq - rt->speed_prev_seq;
    speed_dt = ((float)frame_count * (float)MOTOR_ENC_PERIOD_MS) / 1000.0f;
    left_counts = (int32_t)(enc.left_total_counts - rt->speed_prev_left_total);
    right_counts = (int32_t)(enc.right_total_counts - rt->speed_prev_right_total);

    *actual_left_mps = grayline_counts_to_mps(left_counts,
                                              (float)MOTOR1_COUNTS_PER_OUTPUT_REV_CAL,
                                              speed_dt);
    *actual_right_mps = grayline_counts_to_mps(right_counts,
                                               (float)MOTOR2_COUNTS_PER_OUTPUT_REV_CAL,
                                               speed_dt);

    rt->speed_prev_left_total = enc.left_total_counts;
    rt->speed_prev_right_total = enc.right_total_counts;
    rt->speed_prev_seq = enc.seq;

    /* Ignore a corrupt encoder frame instead of feeding an impossible speed to PID. */
    if (abs_f32(*actual_left_mps) > GRAYLINE_MAX_WHEEL_SPEED_MPS ||
        abs_f32(*actual_right_mps) > GRAYLINE_MAX_WHEEL_SPEED_MPS) {
        return false;
    }

    *dt_out = speed_dt;
    return true;
}

static float grayline_speed_pid_step(float target_mps,
                                     float actual_mps,
                                     float dt,
                                     float *i_accum,
                                     float *prev_error)
{
    float error = target_mps - actual_mps;
    float d = 0.0f;
    float out;

    if (dt > 0.0f) {
        if (i_accum != NULL) {
            *i_accum += error * dt;
            *i_accum = clamp_f32(*i_accum,
                                 -GRAYLINE_SPEED_I_LIMIT,
                                 GRAYLINE_SPEED_I_LIMIT);
        }
        if (prev_error != NULL) {
            d = (error - *prev_error) / dt;
            *prev_error = error;
        }
    }

    out = (g_motionPid.wheel_speed.kp * error) +
          (g_motionPid.wheel_speed.ki * ((i_accum != NULL) ? *i_accum : 0.0f)) +
          (g_motionPid.wheel_speed.kd * d);
    return clamp_f32(out,
                     -g_motionPid.wheel_speed.pwm_trim_max,
                     g_motionPid.wheel_speed.pwm_trim_max);
}

static bool grayline_calc_line_pos(const Gray_Snapshot_t *snap, float *line_pos)
{
    static const float weights[GRAY_CHANNEL_COUNT] = {
        -7.0f, -5.0f, -3.0f, -1.0f, 1.0f, 3.0f, 5.0f, 7.0f
    };
    float sum = 0.0f;
    float count = 0.0f;

    if (snap == NULL || line_pos == NULL) {
        return false;
    }

    for (uint8_t i = 0U; i < GRAY_CHANNEL_COUNT; i++) {
        if (snap->active[i]) {
            sum += weights[i];
            count += 1.0f;
        }
    }

    if (count <= 0.0f) {
        return false;
    }

    *line_pos = sum / count;
    return true;
}

static void grayline_reset_control(GrayLine_Runtime_t *rt)
{
    if (rt == NULL) {
        return;
    }

    rt->last_line_valid = false;
    rt->speed_ref_ready = false;
    rt->pos_i = 0.0f;
    rt->pos_prev_error = 0.0f;
    rt->rate_i = 0.0f;
    rt->rate_prev_error = 0.0f;
    rt->target_rate_dps = 0.0f;
    rt->actual_rate_dps = 0.0f;
    rt->rate_filter_ready = false;
    rt->telemetry_divider = 0U;
    rt->left_speed_i = 0.0f;
    rt->right_speed_i = 0.0f;
    rt->left_speed_prev_error = 0.0f;
    rt->right_speed_prev_error = 0.0f;
    rt->last_line_pos = 0.0f;
    rt->lost_since_tick = 0U;
    rt->last_left_pwm = 0;
    rt->last_right_pwm = 0;
}

static void grayline_write_status(const GrayLine_Status_t *status)
{
    if (status == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    s_grayLineStatus = *status;
    taskEXIT_CRITICAL();
}

bool GrayLine_ReadStatus(GrayLine_Status_t *out)
{
    if (out == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    *out = s_grayLineStatus;
    taskEXIT_CRITICAL();
    return true;
}

static void grayline_print_help(void)
{
    LOGI_RELIABLE(LOG_MOD_GRAYLINE, "commands:\r\n");
    LOGI_RELIABLE(LOG_MOD_GRAYLINE, "grayline start\r\n");
    LOGI_RELIABLE(LOG_MOD_GRAYLINE, "grayline stop\r\n");
    LOGI_RELIABLE(LOG_MOD_GRAYLINE, "grayline status\r\n");
    LOGI_RELIABLE(LOG_MOD_GRAYLINE, "grayline pid\r\n");
}

static void grayline_print_status(void)
{
    GrayLine_Status_t st;

    (void)GrayLine_ReadStatus(&st);
    LOGI_RELIABLE(LOG_MOD_GRAYLINE, "run=%u valid=%u rate=%u seq=%lu mask=0x%02X pos=%.2f err=%.2f turn=%.3f\r\n",
            st.running ? 1U : 0U,
            st.line_valid ? 1U : 0U,
            st.rate_loop_active ? 1U : 0U,
            (unsigned long)st.seq,
            (unsigned)st.active_mask,
            (double)st.line_pos,
            (double)st.error,
            (double)st.turn_mps);
    LOGI_RELIABLE(LOG_MOD_GRAYLINE, "rate raw=%.1f tgt=%.1f act=%.1f err=%.1f trim=%.3f ff=%.3f\r\n",
            (double)st.raw_target_rate_dps,
            (double)st.target_rate_dps,
            (double)st.actual_rate_dps,
            (double)st.rate_error_dps,
            (double)st.rate_trim_mps,
            (double)st.pos_turn_ff_mps);
    LOGI_RELIABLE(LOG_MOD_GRAYLINE, "L %.3f/%.3f pwm=%d R %.3f/%.3f pwm=%d\r\n",
            (double)st.left_target_mps,
            (double)st.left_actual_mps,
            (int)st.left_pwm,
            (double)st.right_target_mps,
            (double)st.right_actual_mps,
            (int)st.right_pwm);
}

static void grayline_print_pid(void)
{
    LOGI_RELIABLE(LOG_MOD_GRAYLINE, "Kp=%.4f Ki=%.4f Kd=%.4f TurnMax=%.3f Base=%.3f LostMs=%.0f Slew=%.1f RevMax=%.3f\r\n",
            (double)g_grayLinePid.kp,
            (double)g_grayLinePid.ki,
            (double)g_grayLinePid.kd,
            (double)g_grayLinePid.turn_mps_max,
            (double)g_grayLinePid.base_mps,
            (double)g_grayLinePid.lost_timeout_ms,
            (double)g_grayLinePid.pwm_slew,
            (double)g_grayLinePid.reverse_mps_max);
    LOGI_RELIABLE(LOG_MOD_GRAYLINE, "RateKp=%.4f RateKi=%.4f RateKd=%.4f RateMax=%.1f RateTrim=%.3f\r\n",
            (double)g_grayLinePid.rate_kp,
            (double)g_grayLinePid.rate_ki,
            (double)g_grayLinePid.rate_kd,
            (double)g_grayLinePid.rate_dps_max,
            (double)g_grayLinePid.rate_trim_mps_max);
    grayline_print_status();
}

static void grayline_send_justfloat(const GrayLine_Status_t *st)
{
    static const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7f};
    float fdata[17];

    if (st == NULL) {
        return;
    }

    fdata[0] = st->target_pos;
    fdata[1] = st->line_pos;
    fdata[2] = st->error;
    fdata[3] = st->turn_mps;
    fdata[4] = st->left_target_mps;
    fdata[5] = st->left_actual_mps;
    fdata[6] = st->right_target_mps;
    fdata[7] = st->right_actual_mps;
    fdata[8] = (float)st->left_pwm;
    fdata[9] = (float)st->right_pwm;
    fdata[10] = st->pos_turn_ff_mps;
    fdata[11] = st->raw_target_rate_dps;
    fdata[12] = st->target_rate_dps;
    fdata[13] = st->actual_rate_dps;
    fdata[14] = st->rate_error_dps;
    fdata[15] = st->rate_trim_mps;
    fdata[16] = st->rate_loop_active ? 1.0f : 0.0f;

    (void)LOG_SendTelemetryFrame(LOG_TELEMETRY_GRAYLINE, (const uint8_t *)fdata, sizeof(fdata),
                                 justfloat_tail, sizeof(justfloat_tail));
}

static void grayline_start(GrayLine_Runtime_t *rt)
{
    if (rt == NULL) {
        return;
    }

    grayline_reset_control(rt);
    rt->path_assist = false;
    rt->running = true;
    LOGI(LOG_MOD_GRAYLINE, "start\r\n");
}

static void grayline_stop(GrayLine_Runtime_t *rt, bool print_msg)
{
    GrayLine_Status_t st;

    if (rt == NULL) {
        return;
    }

    grayline_stop_motors();
    grayline_reset_control(rt);
    rt->path_assist = false;
    rt->running = false;
    memset(&st, 0, sizeof(st));
    st.rate_loop_enabled = rt->rate_loop_enabled;
    grayline_write_status(&st);
    if (print_msg) {
        LOGI(LOG_MOD_GRAYLINE, "stop\r\n");
    }
}

static void grayline_handle_command(GrayLine_Runtime_t *rt, const GrayLine_Command_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    switch (cmd->type) {
        case GRAYLINE_CMD_HELP:
            grayline_print_help();
            break;
        case GRAYLINE_CMD_STATUS:
            grayline_print_status();
            break;
        case GRAYLINE_CMD_START:
            grayline_start(rt);
            break;
        case GRAYLINE_CMD_STOP:
            grayline_stop(rt, true);
            break;
        case GRAYLINE_CMD_PID:
            grayline_print_pid();
            break;
        case GRAYLINE_CMD_SET_RATE_LOOP: {
            GrayLine_Status_t st;

            rt->rate_loop_enabled = cmd->rate_loop_enabled;
            rt->rate_i = 0.0f;
            rt->rate_prev_error = 0.0f;
            rt->target_rate_dps = 0.0f;
            rt->actual_rate_dps = 0.0f;
            rt->rate_filter_ready = false;
            (void)GrayLine_ReadStatus(&st);
            st.rate_loop_enabled = rt->rate_loop_enabled;
            st.rate_loop_active = false;
            grayline_write_status(&st);
            LOGI(LOG_MOD_GRAYLINE, "rate loop %s\r\n",
                 rt->rate_loop_enabled ? "on" : "off");
            break;
        }
        case GRAYLINE_CMD_SET_PATH_ASSIST:
            grayline_stop_motors();
            grayline_reset_control(rt);
            rt->path_assist = cmd->path_assist_enabled;
            rt->running = cmd->path_assist_enabled;
            LOGI(LOG_MOD_GRAYLINE, "path assist %s\r\n",
                 rt->path_assist ? "on" : "off");
            break;
        default:
            LOGE(LOG_MOD_GRAYLINE, "error: bad command\r\n");
            break;
    }
}

/**
 * @brief 串级 PID 控制主循环（位置环 + 速度环）
 *        每个周期：读取灰度传感器 → 位置环 PID 计算转弯速度 → 分配左右目标速度 → 速度环 PID → PWM 输出
 * @param rt  运行状态指针，包含 PID 积分量、前次误差、编码器历史等运行期状态
 */
static void grayline_update(GrayLine_Runtime_t *rt)
{
    Gray_Snapshot_t gray;                                        /* 灰度传感器原始快照 */
    GrayLine_Status_t st;                                        /* 本轮输出状态 */
    float line_pos = 0.0f;                                       /* 线中心位置 (-7~7) */
    float actual_left_mps = 0.0f;                                /* 左轮实际速度 (m/s) */
    float actual_right_mps = 0.0f;                               /* 右轮实际速度 (m/s) */
    float speed_dt = (float)GRAYLINE_TASK_PERIOD_MS / 1000.0f;   /* 速度环默认 dt (s) */
    float dt = (float)GRAYLINE_TASK_PERIOD_MS / 1000.0f;         /* 位置环 dt (s) */
    float error;                                                  /* 位置偏差 */
    float d;                                                      /* 位置微分项 */
    float pos_turn_ff_mps;                                        /* 位置环转弯前馈 (m/s) */
    float raw_target_rate_dps;                                    /* 限速前目标角速度 (deg/s) */
    float target_rate_dps = 0.0f;                                 /* 平滑后的目标角速度 (deg/s) */
    float actual_rate_dps = 0.0f;                                 /* 滤波后的实际角速度 (deg/s) */
    float rate_error_dps = 0.0f;                                  /* 角速度误差 (deg/s) */
    float rate_trim_mps = 0.0f;                                   /* 角速度环转向修正 (m/s) */
    float turn_mps;                                               /* 最终转弯速度修正 (m/s) */
    float gyro_rate_dps;                                          /* IMU 原始 Z 轴角速度 (deg/s) */
    float left_target;                                            /* 左轮目标速度 (m/s) */
    float right_target;                                           /* 右轮目标速度 (m/s) */
    float left_pwm_f;                                             /* 左轮 PWM 浮点值 */
    float right_pwm_f;                                            /* 右轮 PWM 浮点值 */
    int16_t left_pwm;                                             /* 左轮最终 PWM ([-100,100]) */
    int16_t right_pwm;                                            /* 右轮最终 PWM ([-100,100]) */
    bool line_valid;                                              /* 当前帧线位置是否有效 */
    bool speed_ok;                                                /* 速度测量是否就绪 */
    bool rate_loop_active = false;                                /* 角速度环是否可用 */
    TickType_t now;                                               /* 当前系统 tick 值 */

    if (rt == NULL || !rt->running) {                              /* 非运行态直接返回 */
        return;
    }

    now = xTaskGetTickCount();                                     /* 获取当前 tick，用于超时判断 */
    memset(&st, 0, sizeof(st));
    st.running = true;
    st.target_pos = GRAYLINE_TARGET_POS;                           /* 目标位置 = 0（线中心） */

    if (!Gray_ReadSnapshot(&gray)) {
        if (rt->path_assist) {
            st.path_assist_active = true;
            grayline_write_status(&st);
            return;
        }
        grayline_stop(rt, false);
        return;
    }

    line_valid = grayline_calc_line_pos(&gray, &line_pos);        /* 加权平均计算线位置 (-7~7) */
    if (line_valid) {
        rt->last_line_pos = line_pos;                              /* 更新有效位置缓存 */
        rt->last_line_valid = true;
        rt->lost_since_tick = 0U;                                  /* 清零丢线计时 */
    } else if (rt->path_assist) {
        st.path_assist_active = true;
        st.line_fresh = false;
        st.active_mask = gray.active_mask;
        st.sample_tick = gray.tick;
        st.seq = gray.seq;
        grayline_write_status(&st);
        return;
    } else if (rt->last_line_valid) {
        line_pos = rt->last_line_pos;                              /* 丢线 → 保持上次有效值 */
        if (rt->lost_since_tick == 0U) {
            rt->lost_since_tick = now;                             /* 记录丢线起始时间 */
        }
        if (((float)((now - rt->lost_since_tick) * portTICK_PERIOD_MS)) >=
            g_grayLinePid.lost_timeout_ms) {                       /* 丢线超时 → 停车 */
            grayline_stop(rt, false);
            return;
        }
    } else {
        grayline_stop(rt, false);                                  /* 从未有效过 → 停车 */
        return;
    }

    /* ========== 位置环 PID ========== */
    error = GRAYLINE_TARGET_POS - line_pos;                        /* 位置偏差：目标 0 - 实际线位置 */
    rt->pos_i += error * dt;                                       /* 积分累加 */
    rt->pos_i = clamp_f32(rt->pos_i, -GRAYLINE_POS_I_LIMIT, GRAYLINE_POS_I_LIMIT);  /* 积分限幅 */
    d = (error - rt->pos_prev_error) / dt;                         /* 微分：误差变化率 */
    rt->pos_prev_error = error;                                    /* 保存本次误差供下次微分使用 */

    pos_turn_ff_mps = (g_grayLinePid.kp * error) +                /* P 项 */
                      (g_grayLinePid.ki * rt->pos_i) +             /* I 项 */
                      (g_grayLinePid.kd * d);                      /* D 项 */
    pos_turn_ff_mps = clamp_f32(pos_turn_ff_mps,                   /* 转弯速度限幅 */
                                -g_grayLinePid.turn_mps_max,
                                g_grayLinePid.turn_mps_max);

    if (rt->path_assist) {
        st.line_valid = true;
        st.line_fresh = true;
        st.line_pos = line_pos;
        st.error = error;
        st.turn_mps = pos_turn_ff_mps;
        st.pos_turn_ff_mps = pos_turn_ff_mps;
        st.path_assist_active = true;
        st.rate_loop_enabled = rt->rate_loop_enabled;
        st.active_mask = gray.active_mask;
        st.sample_tick = gray.tick;
        st.seq = gray.seq;
        grayline_write_status(&st);
        return;
    }

    /* ========== IMU 角速度阻尼环 ========== */
    raw_target_rate_dps = grayline_turn_to_rate_dps(pos_turn_ff_mps);
    raw_target_rate_dps = clamp_f32(raw_target_rate_dps,
                                    -g_grayLinePid.rate_dps_max,
                                    g_grayLinePid.rate_dps_max);
    turn_mps = pos_turn_ff_mps;

    if (rt->rate_loop_enabled && grayline_read_yaw_rate_dps(&gyro_rate_dps)) {
        /* Keep line-following authority in the position loop; gyro feedback is damping only. */
        rt->target_rate_dps = raw_target_rate_dps;
        if (!rt->rate_filter_ready) {
            rt->actual_rate_dps = gyro_rate_dps;
            rt->rate_filter_ready = true;
        } else {
            rt->actual_rate_dps = grayline_lowpass(gyro_rate_dps,
                                                    rt->actual_rate_dps,
                                                    GRAYLINE_RATE_LP_ALPHA);
        }

        target_rate_dps = rt->target_rate_dps;
        actual_rate_dps = rt->actual_rate_dps;
        rate_trim_mps = grayline_rate_pid_step(rt,
                                               target_rate_dps,
                                               actual_rate_dps,
                                               dt,
                                               &rate_error_dps);
        turn_mps = clamp_f32(pos_turn_ff_mps + rate_trim_mps,
                             -g_grayLinePid.turn_mps_max,
                             g_grayLinePid.turn_mps_max);
        rate_loop_active = true;
    } else {
        /* IMU unavailable: keep the proven position-only controller active. */
        rt->rate_i = 0.0f;
        rt->rate_prev_error = 0.0f;
        rt->target_rate_dps = 0.0f;
        rt->actual_rate_dps = 0.0f;
        rt->rate_filter_ready = false;
    }

    /* ========== 速度分配 ========== */
    left_target = g_grayLinePid.base_mps - turn_mps;               /* 左轮 = 基准速度 - 转弯量（差速转向） */
    right_target = g_grayLinePid.base_mps + turn_mps;              /* 右轮 = 基准速度 + 转弯量 */
    left_target = clamp_f32(left_target,                           /* 左轮速度限幅 */
                            -g_grayLinePid.reverse_mps_max,        /*  最低可反向 */
                            0.25f);                                /*  最高 0.25 m/s */
    right_target = clamp_f32(right_target,
                             -g_grayLinePid.reverse_mps_max,
                             0.25f);

    /* ========== 速度闭环 ========== */
    speed_ok = grayline_read_wheel_speeds(rt,                      /* 读取编码器速度 */
                                          &actual_left_mps,
                                          &actual_right_mps,
                                          &speed_dt);
    left_pwm_f = grayline_target_to_pwm(left_target);              /* 目标速度 → 前馈 PWM */
    right_pwm_f = grayline_target_to_pwm(right_target);

    if (speed_ok) {
        left_pwm_f += grayline_speed_pid_step(left_target,          /* 速度环 PID 修正左轮 */
                                              actual_left_mps,
                                              speed_dt,
                                              &rt->left_speed_i,
                                              &rt->left_speed_prev_error);
        right_pwm_f += grayline_speed_pid_step(right_target,        /* 速度环 PID 修正右轮 */
                                               actual_right_mps,
                                               speed_dt,
                                               &rt->right_speed_i,
                                               &rt->right_speed_prev_error);
    }

    /* ========== PWM 输出处理 ========== */
    left_pwm_f = slew_f32((float)rt->last_left_pwm, left_pwm_f, g_grayLinePid.pwm_slew);   /* 梯形加减速限幅 */
    right_pwm_f = slew_f32((float)rt->last_right_pwm, right_pwm_f, g_grayLinePid.pwm_slew);
    left_pwm = clamp_i16((int16_t)left_pwm_f, -100, 100);          /* 限幅到 PWM 有效范围 */
    right_pwm = clamp_i16((int16_t)right_pwm_f, -100, 100);
    rt->last_left_pwm = left_pwm;                                  /* 保存本次 PWM 供下次 Slew 使用 */
    rt->last_right_pwm = right_pwm;

    /* ========== 状态记录与输出 ========== */
    st.line_valid = line_valid;
    st.line_pos = line_pos;
    st.error = error;
    st.turn_mps = turn_mps;
    st.pos_turn_ff_mps = pos_turn_ff_mps;
    st.raw_target_rate_dps = raw_target_rate_dps;
    st.target_rate_dps = target_rate_dps;
    st.actual_rate_dps = actual_rate_dps;
    st.rate_error_dps = rate_error_dps;
    st.rate_trim_mps = rate_trim_mps;
    st.left_target_mps = left_target;
    st.left_actual_mps = actual_left_mps;
    st.right_target_mps = right_target;
    st.right_actual_mps = actual_right_mps;
    st.left_pwm = left_pwm;
    st.right_pwm = right_pwm;
    st.rate_loop_enabled = rt->rate_loop_enabled;
    st.rate_loop_active = rate_loop_active;
    st.path_assist_active = false;
    st.line_fresh = line_valid;
    st.active_mask = gray.active_mask;                             /* 传感器激活位掩码 */
    st.sample_tick = gray.tick;
    st.seq = gray.seq;                                             /* 传感器帧序号 */
    grayline_write_status(&st);                                    /* 写入全局状态（临界区保护） */
    rt->telemetry_divider++;
    if (rt->telemetry_divider >= GRAYLINE_TELEMETRY_DIVIDER) {
        rt->telemetry_divider = 0U;
        grayline_send_justfloat(&st);                              /* 50 Hz JustFloat，避免阻塞 UART 扰动控制 */
    }
    grayline_drive(left_pwm, right_pwm);                           /* 发送 PWM 到电机驱动 */
}

void grayline_task(void *pvParameters)
{
    GrayLine_Runtime_t rt;
    TickType_t lastWake = xTaskGetTickCount();

    (void)pvParameters;

    memset(&rt, 0, sizeof(rt));
    memset(&s_grayLineStatus, 0, sizeof(s_grayLineStatus));
    while (1) {
        GrayLine_Command_t cmd;

        while (g_grayLineCmdQueue != NULL &&
               xQueueReceive(g_grayLineCmdQueue, &cmd, 0) == pdTRUE) {
            grayline_handle_command(&rt, &cmd);
        }

        grayline_update(&rt);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(GRAYLINE_TASK_PERIOD_MS));
    }
}
