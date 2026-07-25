/**
 * @file    app_motion.c
 * @brief   Basic INS-based straight and single-wheel turn motion control.
 */
#include "app_motion.h"
#include "app_ins.h"
#include "app_motor_encoder.h"
#include "app_tb6612.h"
#include "dev_tb6612.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define MOTION_TASK_PERIOD_MS       10U
#define MOTION_STRAIGHT_TIMEOUT_MS  8000U
#define MOTION_TURN_TIMEOUT_MS      6000U
#define MOTION_ARC_TIMEOUT_MAX_MS   20000U

#define MOTION_STRAIGHT_SPEED_PCT   25

#define MOTION_DIST_TOL_M           0.02f
#define MOTION_MIN_DIST_M           0.01f
#define MOTION_MAX_DIST_M           5.00f
#define MOTION_MIN_TURN_DEG         0.5f
#define MOTION_MAX_TURN_DEG         180.0f
#define MOTION_MIN_ARC_RADIUS_M     0.15f
#define MOTION_MAX_ARC_RADIUS_M     2.00f
#define MOTION_MIN_ARC_ANGLE_DEG    5.0f
#define MOTION_MAX_ARC_ANGLE_DEG    180.0f
#define MOTION_ARC_YAW_TOL_DEG      3.0f
#define MOTION_PID_INTEGRAL_LIMIT   50.0f

#define DEG_TO_RAD                  0.01745329252f
#define MOTION_PI_F                 3.14159265358979323846f
#define MOTION_WHEEL_BASE_M         0.125f
#define MOTION_WHEEL_DIAMETER_M     0.048f
#define MOTION_WHEEL_CIRCUM_M       (MOTION_PI_F * MOTION_WHEEL_DIAMETER_M)
#define MOTION_STRAIGHT_BASE_MPS    0.12f
#define MOTION_ARC_CENTER_MPS       0.1f
#define MOTION_YAW_TRIM_TO_MPS      0.006f
#define MOTION_SPEED_I_LIMIT        0.40f
#define MOTION_ARC_SPEED_FILTER_ALPHA       0.20f
#define MOTION_ARC_PID_TRIM_LIMIT_PCT       4.0f
#define MOTION_ARC_PWM_SLEW_PCT_PER_UPDATE  2.0f
#define MOTION_ARC_LEFT_INNER_SCALE         1.05f
#define MOTION_ARC_LEFT_OUTER_SCALE         0.98f

typedef enum {
    MOTION_STATE_IDLE = 0,
    MOTION_STATE_FWD,
    MOTION_STATE_BACK,
    MOTION_STATE_TURN,
    MOTION_STATE_ARC
} Motion_State_t;

typedef struct {
    Motion_State_t state;
    INS_Pose_t start_pose;
    INS_Pose_t last_pose;
    float target_value;
    float target_yaw_deg;
    float last_progress;
    float last_error;
    float last_yaw_error;
    float arc_radius_m;
    float arc_angle_deg;
    float arc_target_left_mps;
    float arc_target_right_mps;
    float arc_filt_left_mps;
    float arc_filt_right_mps;
    uint32_t arc_timeout_ms;
    float straight_yaw_i;
    float straight_yaw_prev_error;
    float left_speed_i;
    float right_speed_i;
    float left_speed_prev_error;
    float right_speed_prev_error;
    int64_t speed_prev_left_total;
    int64_t speed_prev_right_total;
    uint32_t speed_prev_seq;
    bool speed_ref_ready;
    bool arc_filter_ready;
    float turn_yaw_i;
    float turn_yaw_prev_error;
    int16_t last_left_pwm;
    int16_t last_right_pwm;
    bool turn_acquired;
    TickType_t start_tick;
    uint32_t active_cmd_id;
} Motion_Runtime_t;

QueueHandle_t g_motionCmdQueue = NULL;
Motion_PidConfig_t g_motionPid = {
    {4.50f, 0.02f, 0.08f, 15.0f},
    {1.10f, 0.00f, 0.08f, 1.0f, 11.0f, 22.0f},
    {80.0f, 0.0f, 0.0f, 10.0f}
};
Motion_RuntimeStatus_t g_motionRtStatus = {
    MOTION_RT_IDLE,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0U,
    0U,
    0U,
    MOTION_RESULT_NONE
};
static float wrap_180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg <= -180.0f) {
        deg += 360.0f;
    }
    return deg;
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

static float motion_counts_to_mps(int32_t counts, float counts_per_rev, float dt)
{
    if (dt <= 0.0f) {
        return 0.0f;
    }
    return (((float)counts / counts_per_rev) * MOTION_WHEEL_CIRCUM_M) / dt;
}

static void motion_reset_speed_pid(Motion_Runtime_t *rt)
{
    if (rt == NULL) {
        return;
    }

    rt->left_speed_i = 0.0f;
    rt->right_speed_i = 0.0f;
    rt->left_speed_prev_error = 0.0f;
    rt->right_speed_prev_error = 0.0f;
    rt->speed_ref_ready = false;
    {
        MotorEncoderSnapshot_t enc;
        if (MotorEncoder_ReadSnapshot(&enc)) {
            rt->speed_prev_left_total = enc.left_total_counts;
            rt->speed_prev_right_total = enc.right_total_counts;
            rt->speed_prev_seq = enc.seq;
            rt->speed_ref_ready = true;
        }
    }
}

static bool motion_read_wheel_speeds(Motion_Runtime_t *rt,
                                     float *actual_left_mps,
                                     float *actual_right_mps,
                                     float *dt_out)
{
    MotorEncoderSnapshot_t enc;
    uint32_t frame_count;
    float speed_dt;
    int32_t left_counts;
    int32_t right_counts;

    if (rt == NULL ||
        actual_left_mps == NULL ||
        actual_right_mps == NULL ||
        dt_out == NULL) {
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

    *actual_left_mps = motion_counts_to_mps(left_counts,
                                            (float)MOTOR1_COUNTS_PER_OUTPUT_REV_CAL,
                                            speed_dt);
    *actual_right_mps = motion_counts_to_mps(right_counts,
                                             (float)MOTOR2_COUNTS_PER_OUTPUT_REV_CAL,
                                             speed_dt);
    *dt_out = speed_dt;

    rt->speed_prev_left_total = enc.left_total_counts;
    rt->speed_prev_right_total = enc.right_total_counts;
    rt->speed_prev_seq = enc.seq;
    return true;
}

static float motion_speed_pid_step(float target_mps,
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
            *i_accum = clamp_f32(*i_accum, -MOTION_SPEED_I_LIMIT, MOTION_SPEED_I_LIMIT);
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

static bool pose_ready(const INS_Pose_t *pose)
{
    uint32_t required = INS_FLAG_IMU_VALID | INS_FLAG_YAW_ZERO_READY;
    return pose != NULL && ((pose->flags & required) == required);
}

static bool get_latest_pose(INS_Pose_t *pose)
{
    if (pose == NULL) {
        return false;
    }

    return INS_Pose_Read(pose);
}

static void send_motor_cmd_wait(MotorCmdType type, int16_t val, TickType_t wait_ticks)
{
    MotorCmd cmd;

    if (g_motorCmdQueue == NULL) {
        return;
    }

    cmd.type = type;
    cmd.val = val;
    (void)xQueueSend(g_motorCmdQueue, &cmd, wait_ticks);
}

static void send_motor_cmd(MotorCmdType type, int16_t val)
{
    send_motor_cmd_wait(type, val, pdMS_TO_TICKS(2));
}

static void motion_stop_motors(void)
{
    send_motor_cmd_wait(MOTOR_CMD_LEFT_SPEED, 0, pdMS_TO_TICKS(20));
    send_motor_cmd_wait(MOTOR_CMD_RIGHT_SPEED, 0, pdMS_TO_TICKS(20));
    send_motor_cmd_wait(MOTOR_CMD_ONOFF, 0, pdMS_TO_TICKS(20));
}

static void motion_set_pwm_record(Motion_Runtime_t *rt, int16_t left_pct, int16_t right_pct)
{
    if (rt != NULL) {
        rt->last_left_pwm = clamp_i16(left_pct, 0, 100);
        rt->last_right_pwm = clamp_i16(right_pct, 0, 100);
    }
}

static void motion_drive_straight(bool forward, int16_t left_pct, int16_t right_pct)
{
    send_motor_cmd(MOTOR_CMD_DIR, forward ? (int16_t)TB6612_DIR_FORWARD : (int16_t)TB6612_DIR_REVERSE);
    send_motor_cmd(MOTOR_CMD_LEFT_SPEED, clamp_i16(left_pct, 0, 100));
    send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, clamp_i16(right_pct, 0, 100));
    send_motor_cmd(MOTOR_CMD_ONOFF, 1);
}

static void motion_drive_arc(int16_t left_pct, int16_t right_pct)
{
    send_motor_cmd(MOTOR_CMD_DIR, (int16_t)TB6612_DIR_FORWARD);
    send_motor_cmd(MOTOR_CMD_LEFT_SPEED, clamp_i16(left_pct, 0, 100));
    send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, clamp_i16(right_pct, 0, 100));
    send_motor_cmd(MOTOR_CMD_ONOFF, 1);
}

static void motion_drive_single_wheel_cmd(float turn_cmd)
{
    int16_t pwm = (int16_t)clamp_f32(fabsf(turn_cmd),
                                     g_motionPid.turn_yaw.pwm_min,
                                     g_motionPid.turn_yaw.pwm_max);

    send_motor_cmd(MOTOR_CMD_DIR, (int16_t)TB6612_DIR_FORWARD);
    if (turn_cmd > 0.0f) {
        send_motor_cmd(MOTOR_CMD_LEFT_SPEED, 0);
        send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, pwm);
    } else {
        send_motor_cmd(MOTOR_CMD_LEFT_SPEED, pwm);
        send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, 0);
    }
    send_motor_cmd(MOTOR_CMD_ONOFF, 1);
}

static void motion_send_yaw_justfloat(float target_yaw_deg, float actual_yaw_deg)
{
    static const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7f};
    float fdata[2];

    fdata[0] = target_yaw_deg;
    fdata[1] = actual_yaw_deg;
    (void)LOG_SendTelemetryFrame(LOG_TELEMETRY_MOTION, (const uint8_t *)fdata, sizeof(fdata),
                                 justfloat_tail, sizeof(justfloat_tail));
}

static void motion_send_arc_justfloat(float target_left_mps,
                                      float actual_left_mps,
                                      float target_right_mps,
                                      float actual_right_mps,
                                      float target_yaw_deg,
                                      float actual_yaw_deg,
                                      float left_pwm,
                                      float right_pwm)
{
    static const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7f};
    float fdata[8];

    fdata[0] = target_left_mps;
    fdata[1] = actual_left_mps;
    fdata[2] = target_right_mps;
    fdata[3] = actual_right_mps;
    fdata[4] = target_yaw_deg;
    fdata[5] = actual_yaw_deg;
    fdata[6] = left_pwm;
    fdata[7] = right_pwm;
    (void)LOG_SendTelemetryFrame(LOG_TELEMETRY_MOTION, (const uint8_t *)fdata, sizeof(fdata),
                                 justfloat_tail, sizeof(justfloat_tail));
}

static const char *motion_state_name(Motion_State_t state)
{
    switch (state) {
        case MOTION_STATE_FWD:
            return "FWD";
        case MOTION_STATE_BACK:
            return "BACK";
        case MOTION_STATE_TURN:
            return "TURN";
        case MOTION_STATE_ARC:
            return "ARC";
        case MOTION_STATE_IDLE:
        default:
            return "IDLE";
    }
}

static const char *motion_result_name(Motion_Result_t result)
{
    switch (result) {
        case MOTION_RESULT_ACCEPTED: return "ACCEPTED";
        case MOTION_RESULT_DONE:     return "DONE";
        case MOTION_RESULT_STOPPED:  return "STOPPED";
        case MOTION_RESULT_REJECTED: return "REJECTED";
        case MOTION_RESULT_TIMEOUT:  return "TIMEOUT";
        case MOTION_RESULT_ABORTED:  return "ABORTED";
        case MOTION_RESULT_NONE:
        default:                     return "NONE";
    }
}

static const char *motion_cmd_type_name(Motion_CommandType_t type)
{
    switch (type) {
        case MOTION_CMD_HELP:   return "HELP";
        case MOTION_CMD_STATUS: return "STATUS";
        case MOTION_CMD_STOP:   return "STOP";
        case MOTION_CMD_FWD:    return "FWD";
        case MOTION_CMD_BACK:   return "BACK";
        case MOTION_CMD_TURN:   return "TURN";
        case MOTION_CMD_ARC:    return "ARC";
        default:                return "?";
    }
}

static void motion_mark_accepted(Motion_Runtime_t *rt, uint32_t cmd_id)
{
    if (rt != NULL) {
        rt->active_cmd_id = cmd_id;
    }
    g_motionRtStatus.active_cmd_id = cmd_id;
    g_motionRtStatus.last_result = MOTION_RESULT_ACCEPTED;
}

static void motion_mark_rejected(uint32_t cmd_id)
{
    g_motionRtStatus.rejected_cmd_id = cmd_id;
    g_motionRtStatus.last_result = MOTION_RESULT_REJECTED;
}

static void motion_ack_accept(uint32_t cmd_id, Motion_CommandType_t type, float value)
{
    LOGD(LOG_MOD_MOTION, "accept cmd_id=0x%08lX type=%s value=%+.3f\r\n",
                        (unsigned long)cmd_id,
                        motion_cmd_type_name(type),
                        value);
}

static void motion_ack_reject(uint32_t cmd_id, Motion_CommandType_t type, float value, const char *reason)
{
    LOGD(LOG_MOD_MOTION, "reject cmd_id=0x%08lX type=%s value=%+.3f reason=%s state=%s\r\n",
                        (unsigned long)cmd_id,
                        motion_cmd_type_name(type),
                        value,
                        reason,
                        motion_state_name((Motion_State_t)g_motionRtStatus.state));
}

static void motion_ack_done(uint32_t cmd_id, Motion_Result_t result)
{
    LOGD(LOG_MOD_MOTION, "done cmd_id=0x%08lX result=%s\r\n",
                        (unsigned long)cmd_id,
                        motion_result_name(result));
}

static void motion_print_help(void)
{
    LOGI_RELIABLE(LOG_MOD_MOTION, "commands:\r\n");
    LOGI_RELIABLE(LOG_MOD_MOTION, "motion help\r\n");
    LOGI_RELIABLE(LOG_MOD_MOTION, "motion status\r\n");
    LOGI_RELIABLE(LOG_MOD_MOTION, "motion stop\r\n");
    LOGI_RELIABLE(LOG_MOD_MOTION, "motion fwd <meters>\r\n");
    LOGI_RELIABLE(LOG_MOD_MOTION, "motion back <meters>\r\n");
    LOGI_RELIABLE(LOG_MOD_MOTION, "motion turn <-180..180 deg>\r\n");
    LOGI_RELIABLE(LOG_MOD_MOTION, "motion arc <radius_m> <-180..180 deg>\r\n");
}

static void motion_print_status(const Motion_Runtime_t *rt)
{
    LOGI_RELIABLE(LOG_MOD_MOTION, "state=%s target=%.3f target_yaw=%+.2f progress=%.3f err=%.3f yaw_err=%+.2f pwm L=%d R=%d X=%+.3f Y=%+.3f YAW=%+.2f W=%+.2f flags=0x%08lX active=%lu done=%lu rejected=%lu result=%s\r\n",
            motion_state_name(rt->state),
            rt->target_value,
            rt->target_yaw_deg,
            rt->last_progress,
            rt->last_error,
            rt->last_yaw_error,
            (int)rt->last_left_pwm,
            (int)rt->last_right_pwm,
            rt->last_pose.x_m,
            rt->last_pose.y_m,
            rt->last_pose.yaw_deg,
            rt->last_pose.w_dps,
            (unsigned long)rt->last_pose.flags,
            (unsigned long)g_motionRtStatus.active_cmd_id,
            (unsigned long)g_motionRtStatus.done_cmd_id,
            (unsigned long)g_motionRtStatus.rejected_cmd_id,
            motion_result_name(g_motionRtStatus.last_result));
    LOGI_RELIABLE(LOG_MOD_MOTION, "StraightYaw Kp=%.2f Ki=%.2f Kd=%.2f TrimMax=%.1f\r\n",
            g_motionPid.straight_yaw.kp,
            g_motionPid.straight_yaw.ki,
            g_motionPid.straight_yaw.kd,
            g_motionPid.straight_yaw.trim_max);
    LOGI_RELIABLE(LOG_MOD_MOTION, "TurnYaw Kp=%.2f Ki=%.2f Kd=%.2f Dead=%.1f PwmMin=%.1f PwmMax=%.1f\r\n",
            g_motionPid.turn_yaw.kp,
            g_motionPid.turn_yaw.ki,
            g_motionPid.turn_yaw.kd,
            g_motionPid.turn_yaw.deadband_deg,
            g_motionPid.turn_yaw.pwm_min,
            g_motionPid.turn_yaw.pwm_max);
    LOGI_RELIABLE(LOG_MOD_MOTION, "WheelSpd Kp=%.2f Ki=%.2f Kd=%.2f PwmTrim=%.1f\r\n",
            g_motionPid.wheel_speed.kp,
            g_motionPid.wheel_speed.ki,
            g_motionPid.wheel_speed.kd,
            g_motionPid.wheel_speed.pwm_trim_max);
}

static bool motion_start_straight(Motion_Runtime_t *rt, Motion_State_t state, float distance_m, uint32_t cmd_id)
{
    INS_Pose_t pose;
    Motion_CommandType_t cmd_type = (state == MOTION_STATE_BACK) ? MOTION_CMD_BACK : MOTION_CMD_FWD;

    if (rt->state != MOTION_STATE_IDLE) {
        if (rt->state == MOTION_STATE_TURN && rt->turn_acquired) {
            motion_stop_motors();
            rt->state = MOTION_STATE_IDLE;
        } else {
            LOGE(LOG_MOD_MOTION, "error: busy, use motion stop first\r\n");
            motion_mark_rejected(cmd_id);
            motion_ack_reject(cmd_id, cmd_type, distance_m, "busy");
            return false;
        }
    }
    if (rt->state != MOTION_STATE_IDLE) {
        LOGE(LOG_MOD_MOTION, "error: busy, use motion stop first\r\n");
        motion_mark_rejected(cmd_id);
        motion_ack_reject(cmd_id, cmd_type, distance_m, "busy");
        return false;
    }
    if (distance_m < MOTION_MIN_DIST_M || distance_m > MOTION_MAX_DIST_M) {
        LOGE(LOG_MOD_MOTION, "error: distance range %.2f..%.2fm\r\n",
                MOTION_MIN_DIST_M, MOTION_MAX_DIST_M);
        motion_mark_rejected(cmd_id);
        motion_ack_reject(cmd_id, cmd_type, distance_m, "range");
        return false;
    }
    if (!get_latest_pose(&pose) || !pose_ready(&pose)) {
        LOGE(LOG_MOD_MOTION, "error: INS not ready\r\n");
        motion_mark_rejected(cmd_id);
        motion_ack_reject(cmd_id, cmd_type, distance_m, "ins_not_ready");
        return false;
    }

    motion_mark_accepted(rt, cmd_id);
    motion_ack_accept(cmd_id, cmd_type, distance_m);
    rt->state = state;
    rt->start_pose = pose;
    rt->last_pose = pose;
    rt->target_value = distance_m;
    rt->target_yaw_deg = pose.yaw_deg;
    rt->last_progress = 0.0f;
    rt->last_error = distance_m;
    rt->last_yaw_error = 0.0f;
    rt->straight_yaw_i = 0.0f;
    rt->straight_yaw_prev_error = 0.0f;
    rt->left_speed_i = 0.0f;
    rt->right_speed_i = 0.0f;
    rt->left_speed_prev_error = 0.0f;
    rt->right_speed_prev_error = 0.0f;
    rt->speed_ref_ready = false;
    {
        MotorEncoderSnapshot_t enc;
        if (MotorEncoder_ReadSnapshot(&enc)) {
            rt->speed_prev_left_total = enc.left_total_counts;
            rt->speed_prev_right_total = enc.right_total_counts;
            rt->speed_prev_seq = enc.seq;
            rt->speed_ref_ready = true;
        }
    }
    rt->last_left_pwm = MOTION_STRAIGHT_SPEED_PCT;
    rt->last_right_pwm = MOTION_STRAIGHT_SPEED_PCT;
    rt->start_tick = xTaskGetTickCount();

    motion_drive_straight(state == MOTION_STATE_FWD,
                          MOTION_STRAIGHT_SPEED_PCT,
                          MOTION_STRAIGHT_SPEED_PCT);

    LOGI(LOG_MOD_MOTION, "start %s dist=%.3fm yaw=%+.2f\r\n",
            motion_state_name(state), distance_m, pose.yaw_deg);
    return true;
}

static bool motion_start_turn(Motion_Runtime_t *rt, float angle_deg, uint32_t cmd_id)
{
    INS_Pose_t pose;

    if (rt->state != MOTION_STATE_IDLE) {
        LOGE(LOG_MOD_MOTION, "error: busy, use motion stop first\r\n");
        motion_mark_rejected(cmd_id);
        motion_ack_reject(cmd_id, MOTION_CMD_TURN, angle_deg, "busy");
        return false;
    }
    if (fabsf(angle_deg) < MOTION_MIN_TURN_DEG || fabsf(angle_deg) > MOTION_MAX_TURN_DEG) {
        LOGE(LOG_MOD_MOTION, "error: turn range -180..180 deg, zero rejected\r\n");
        motion_mark_rejected(cmd_id);
        motion_ack_reject(cmd_id, MOTION_CMD_TURN, angle_deg, "range");
        return false;
    }
    if (!get_latest_pose(&pose) || !pose_ready(&pose)) {
        LOGE(LOG_MOD_MOTION, "error: INS not ready\r\n");
        motion_mark_rejected(cmd_id);
        motion_ack_reject(cmd_id, MOTION_CMD_TURN, angle_deg, "ins_not_ready");
        return false;
    }

    motion_mark_accepted(rt, cmd_id);
    motion_ack_accept(cmd_id, MOTION_CMD_TURN, angle_deg);
    rt->state = MOTION_STATE_TURN;
    rt->start_pose = pose;
    rt->last_pose = pose;
    rt->target_value = angle_deg;
    rt->target_yaw_deg = wrap_180(pose.yaw_deg + angle_deg);
    rt->last_progress = 0.0f;
    rt->last_error = angle_deg;
    rt->last_yaw_error = angle_deg;
    rt->turn_yaw_i = 0.0f;
    rt->turn_yaw_prev_error = angle_deg;
    rt->turn_acquired = false;
    rt->last_left_pwm = 0;
    rt->last_right_pwm = 0;
    rt->start_tick = xTaskGetTickCount();

    motion_drive_single_wheel_cmd(angle_deg);

    LOGI(LOG_MOD_MOTION, "start TURN angle=%+.1f yaw=%+.2f target_yaw=%+.2f mode=%s\r\n",
            angle_deg,
            pose.yaw_deg,
            rt->target_yaw_deg,
            (angle_deg > 0.0f) ? "left_stop/right_move" : "right_stop/left_move");
    return true;
}

static bool motion_start_arc(Motion_Runtime_t *rt, float radius_m, float angle_deg, uint32_t cmd_id)
{
    INS_Pose_t pose;
    float inner_mps;
    float outer_mps;
    float arc_len_m;
    uint32_t estimated_ms;
    int16_t left_pwm;
    int16_t right_pwm;

    if (rt->state != MOTION_STATE_IDLE) {
        LOGE(LOG_MOD_MOTION, "error: busy, use motion stop first\r\n");
        motion_mark_rejected(cmd_id);
        motion_ack_reject(cmd_id, MOTION_CMD_ARC, radius_m, "busy");
        return false;
    }
    if (radius_m < MOTION_MIN_ARC_RADIUS_M || radius_m > MOTION_MAX_ARC_RADIUS_M) {
        LOGE(LOG_MOD_MOTION, "error: arc radius range %.2f..%.2fm\r\n",
                MOTION_MIN_ARC_RADIUS_M,
                MOTION_MAX_ARC_RADIUS_M);
        motion_mark_rejected(cmd_id);
        motion_ack_reject(cmd_id, MOTION_CMD_ARC, radius_m, "radius");
        return false;
    }
    if (fabsf(angle_deg) < MOTION_MIN_ARC_ANGLE_DEG ||
        fabsf(angle_deg) > MOTION_MAX_ARC_ANGLE_DEG) {
        LOGE(LOG_MOD_MOTION, "error: arc angle range +/-%.1f..%.1f deg\r\n",
                MOTION_MIN_ARC_ANGLE_DEG,
                MOTION_MAX_ARC_ANGLE_DEG);
        motion_mark_rejected(cmd_id);
        motion_ack_reject(cmd_id, MOTION_CMD_ARC, radius_m, "angle");
        return false;
    }
    if (!get_latest_pose(&pose) || !pose_ready(&pose)) {
        LOGE(LOG_MOD_MOTION, "error: INS not ready\r\n");
        motion_mark_rejected(cmd_id);
        motion_ack_reject(cmd_id, MOTION_CMD_ARC, radius_m, "ins_not_ready");
        return false;
    }

    inner_mps = MOTION_ARC_CENTER_MPS *
                ((radius_m - (MOTION_WHEEL_BASE_M * 0.5f)) / radius_m);
    outer_mps = MOTION_ARC_CENTER_MPS *
                ((radius_m + (MOTION_WHEEL_BASE_M * 0.5f)) / radius_m);

    motion_mark_accepted(rt, cmd_id);
    motion_ack_accept(cmd_id, MOTION_CMD_ARC, radius_m);
    rt->state = MOTION_STATE_ARC;
    rt->start_pose = pose;
    rt->last_pose = pose;
    rt->target_value = angle_deg;
    rt->target_yaw_deg = wrap_180(pose.yaw_deg + angle_deg);
    rt->last_progress = 0.0f;
    rt->last_error = angle_deg;
    rt->last_yaw_error = angle_deg;
    rt->arc_radius_m = radius_m;
    rt->arc_angle_deg = angle_deg;
    if (angle_deg > 0.0f) {
        rt->arc_target_left_mps = inner_mps * MOTION_ARC_LEFT_INNER_SCALE;
        rt->arc_target_right_mps = outer_mps * MOTION_ARC_LEFT_OUTER_SCALE;
    } else {
        rt->arc_target_left_mps = outer_mps;
        rt->arc_target_right_mps = inner_mps;
    }
    motion_reset_speed_pid(rt);
    rt->arc_filter_ready = false;
    rt->arc_filt_left_mps = 0.0f;
    rt->arc_filt_right_mps = 0.0f;
    rt->last_left_pwm = (int16_t)clamp_f32((rt->arc_target_left_mps / MOTION_STRAIGHT_BASE_MPS) *
                                           (float)MOTION_STRAIGHT_SPEED_PCT,
                                           0.0f,
                                           100.0f);
    rt->last_right_pwm = (int16_t)clamp_f32((rt->arc_target_right_mps / MOTION_STRAIGHT_BASE_MPS) *
                                            (float)MOTION_STRAIGHT_SPEED_PCT,
                                            0.0f,
                                            100.0f);
    rt->start_tick = xTaskGetTickCount();

    arc_len_m = radius_m * fabsf(angle_deg) * DEG_TO_RAD;
    estimated_ms = (uint32_t)((arc_len_m / MOTION_ARC_CENTER_MPS) * 1000.0f) + 2000U;
    if (estimated_ms < 3000U) {
        estimated_ms = 3000U;
    }
    if (estimated_ms > MOTION_ARC_TIMEOUT_MAX_MS) {
        estimated_ms = MOTION_ARC_TIMEOUT_MAX_MS;
    }
    rt->arc_timeout_ms = estimated_ms;

    left_pwm = rt->last_left_pwm;
    right_pwm = rt->last_right_pwm;
    motion_drive_arc(left_pwm, right_pwm);

    LOGI(LOG_MOD_MOTION, "start ARC radius=%.3fm angle=%+.1f yaw=%+.2f target_yaw=%+.2f Ltar=%.3f Rtar=%.3f timeout=%lums\r\n",
            radius_m,
            angle_deg,
            pose.yaw_deg,
            rt->target_yaw_deg,
            rt->arc_target_left_mps,
            rt->arc_target_right_mps,
            (unsigned long)rt->arc_timeout_ms);
    return true;
}

static void motion_finish(Motion_Runtime_t *rt, const char *reason, Motion_Result_t result)
{
    motion_stop_motors();
    rt->last_left_pwm = 0;
    rt->last_right_pwm = 0;
    g_motionRtStatus.done_cmd_id = rt->active_cmd_id;
    g_motionRtStatus.last_result = result;
    motion_ack_done(rt->active_cmd_id, result);
    LOGI(LOG_MOD_MOTION, "%s state=%s cmd_id=%lu result=%s progress=%.3f err=%.3f yaw_err=%+.2f X=%+.3f Y=%+.3f YAW=%+.2f\r\n",
            reason,
            motion_state_name(rt->state),
            (unsigned long)rt->active_cmd_id,
            motion_result_name(result),
            rt->last_progress,
            rt->last_error,
            rt->last_yaw_error,
            rt->last_pose.x_m,
            rt->last_pose.y_m,
            rt->last_pose.yaw_deg);
    rt->state = MOTION_STATE_IDLE;
}

static void motion_handle_command(Motion_Runtime_t *rt, const Motion_Command_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    switch (cmd->type) {
        case MOTION_CMD_HELP:
            motion_print_help();
            break;
        case MOTION_CMD_STATUS:
            motion_print_status(rt);
            break;
        case MOTION_CMD_STOP:
            if (rt->state == MOTION_STATE_IDLE) {
                motion_stop_motors();
                LOGI(LOG_MOD_MOTION, "stop ok, already idle\r\n");
            } else {
                motion_finish(rt, "stopped", MOTION_RESULT_STOPPED);
            }
            break;
        case MOTION_CMD_FWD:
            (void)motion_start_straight(rt, MOTION_STATE_FWD, cmd->value, cmd->cmd_id);
            break;
        case MOTION_CMD_BACK:
            (void)motion_start_straight(rt, MOTION_STATE_BACK, cmd->value, cmd->cmd_id);
            break;
        case MOTION_CMD_TURN:
            (void)motion_start_turn(rt, cmd->value, cmd->cmd_id);
            break;
        case MOTION_CMD_ARC:
            (void)motion_start_arc(rt, cmd->value, cmd->value2, cmd->cmd_id);
            break;
        default:
            LOGE(LOG_MOD_MOTION, "error: bad command\r\n");
            motion_mark_rejected(cmd->cmd_id);
            break;
    }
}

static void motion_update_straight(Motion_Runtime_t *rt, const INS_Pose_t *pose)
{
    const float dt = (float)MOTION_TASK_PERIOD_MS / 1000.0f;
    float dx = pose->x_m - rt->start_pose.x_m;
    float dy = pose->y_m - rt->start_pose.y_m;
    float heading_rad = rt->start_pose.yaw_deg * DEG_TO_RAD;
    float projection = dx * cosf(heading_rad) + dy * sinf(heading_rad);
    bool forward = (rt->state == MOTION_STATE_FWD);
    float progress = forward ? projection : -projection;
    float yaw_error = wrap_180(rt->start_pose.yaw_deg - pose->yaw_deg);
    float yaw_d = wrap_180(yaw_error - rt->straight_yaw_prev_error) / dt;
    float trim_f;
    float target_left_mps;
    float target_right_mps;
    int16_t left = MOTION_STRAIGHT_SPEED_PCT;
    int16_t right = MOTION_STRAIGHT_SPEED_PCT;
    uint32_t elapsed_ms = (uint32_t)((xTaskGetTickCount() - rt->start_tick) * portTICK_PERIOD_MS);

    rt->straight_yaw_i += yaw_error * dt;
    rt->straight_yaw_i = clamp_f32(rt->straight_yaw_i,
                                   -MOTION_PID_INTEGRAL_LIMIT,
                                   MOTION_PID_INTEGRAL_LIMIT);
    rt->straight_yaw_prev_error = yaw_error;

    trim_f = (g_motionPid.straight_yaw.kp * yaw_error) +
             (g_motionPid.straight_yaw.ki * rt->straight_yaw_i) +
             (g_motionPid.straight_yaw.kd * yaw_d);
    trim_f = clamp_f32(trim_f,
                       -g_motionPid.straight_yaw.trim_max,
                       g_motionPid.straight_yaw.trim_max);

    if (forward) {
        target_left_mps = MOTION_STRAIGHT_BASE_MPS - (trim_f * MOTION_YAW_TRIM_TO_MPS);
        target_right_mps = MOTION_STRAIGHT_BASE_MPS + (trim_f * MOTION_YAW_TRIM_TO_MPS);
    } else {
        target_left_mps = -(MOTION_STRAIGHT_BASE_MPS + (trim_f * MOTION_YAW_TRIM_TO_MPS));
        target_right_mps = -(MOTION_STRAIGHT_BASE_MPS - (trim_f * MOTION_YAW_TRIM_TO_MPS));
    }

    {
        MotorEncoderSnapshot_t enc;
        bool speedOk = false;
        if (MotorEncoder_ReadSnapshot(&enc)) {
            if (!rt->speed_ref_ready) {
                rt->speed_prev_left_total = enc.left_total_counts;
                rt->speed_prev_right_total = enc.right_total_counts;
                rt->speed_prev_seq = enc.seq;
                rt->speed_ref_ready = true;
            } else if (enc.seq != rt->speed_prev_seq) {
                uint32_t frame_count = enc.seq - rt->speed_prev_seq;
                float speed_dt = ((float)frame_count * (float)MOTOR_ENC_PERIOD_MS) / 1000.0f;
                int32_t left_counts = (int32_t)(enc.left_total_counts - rt->speed_prev_left_total);
                int32_t right_counts = (int32_t)(enc.right_total_counts - rt->speed_prev_right_total);
                float actual_left_mps = motion_counts_to_mps(left_counts,
                                                             (float)MOTOR1_COUNTS_PER_OUTPUT_REV_CAL,
                                                             speed_dt);
                float actual_right_mps = motion_counts_to_mps(right_counts,
                                                              (float)MOTOR2_COUNTS_PER_OUTPUT_REV_CAL,
                                                              speed_dt);
                float left_pwm = (float)MOTION_STRAIGHT_SPEED_PCT +
                                 motion_speed_pid_step(target_left_mps,
                                                       actual_left_mps,
                                                       speed_dt,
                                                       &rt->left_speed_i,
                                                       &rt->left_speed_prev_error);
                float right_pwm = (float)MOTION_STRAIGHT_SPEED_PCT +
                                  motion_speed_pid_step(target_right_mps,
                                                        actual_right_mps,
                                                        speed_dt,
                                                        &rt->right_speed_i,
                                                        &rt->right_speed_prev_error);
                left = (int16_t)clamp_f32(left_pwm, 0.0f, 100.0f);
                right = (int16_t)clamp_f32(right_pwm, 0.0f, 100.0f);
                rt->speed_prev_left_total = enc.left_total_counts;
                rt->speed_prev_right_total = enc.right_total_counts;
                rt->speed_prev_seq = enc.seq;
                speedOk = true;
            }
        }

        if (!speedOk) {
            int16_t trim = (int16_t)trim_f;
            if (forward) {
                left = MOTION_STRAIGHT_SPEED_PCT - trim;
                right = MOTION_STRAIGHT_SPEED_PCT + trim;
            } else {
                left = MOTION_STRAIGHT_SPEED_PCT + trim;
                right = MOTION_STRAIGHT_SPEED_PCT - trim;
            }
        }
    }

    rt->last_progress = progress;
    rt->last_error = rt->target_value - progress;
    rt->last_yaw_error = yaw_error;
    motion_set_pwm_record(rt, left, right);

    /* VOFA+ JustFloat: 发送 2 通道浮点数据 + 帧尾 */
    {
        static const uint8_t justfloat_tail[4] = {0x00, 0x00, 0x80, 0x7f};
        float fdata[2];
        fdata[0] = rt->start_pose.yaw_deg;  /* Ch0: 目标角度 */
        fdata[1] = pose->yaw_deg;           /* Ch1: 实际角度 */
        (void)LOG_SendTelemetryFrame(LOG_TELEMETRY_MOTION, (const uint8_t *)fdata, sizeof(fdata),
                                     justfloat_tail, sizeof(justfloat_tail));
    }

    if (progress >= (rt->target_value - MOTION_DIST_TOL_M)) {
        motion_finish(rt, "done", MOTION_RESULT_DONE);
        return;
    }
    if (elapsed_ms >= MOTION_STRAIGHT_TIMEOUT_MS) {
        motion_finish(rt, "timeout", MOTION_RESULT_TIMEOUT);
        return;
    }

    motion_drive_straight(forward, left, right);
}

static void motion_update_turn(Motion_Runtime_t *rt, const INS_Pose_t *pose)
{
    const float dt = (float)MOTION_TASK_PERIOD_MS / 1000.0f;
    float progress = wrap_180(pose->yaw_deg - rt->start_pose.yaw_deg);
    float error = wrap_180(rt->target_yaw_deg - pose->yaw_deg);
    float turn_cmd;
    uint32_t elapsed_ms = (uint32_t)((xTaskGetTickCount() - rt->start_tick) * portTICK_PERIOD_MS);

    rt->last_progress = progress;
    rt->last_error = error;
    rt->last_yaw_error = error;

    if (fabsf(error) <= g_motionPid.turn_yaw.deadband_deg) {
        motion_finish(rt, "done", MOTION_RESULT_DONE);
        return;
    }

    if (!rt->turn_acquired && elapsed_ms >= MOTION_TURN_TIMEOUT_MS) {
        motion_finish(rt, "timeout", MOTION_RESULT_TIMEOUT);
        return;
    }

    motion_send_yaw_justfloat(rt->target_yaw_deg, pose->yaw_deg);

    rt->turn_yaw_i += error * dt;
    rt->turn_yaw_i = clamp_f32(rt->turn_yaw_i,
                               -MOTION_PID_INTEGRAL_LIMIT,
                               MOTION_PID_INTEGRAL_LIMIT);
    rt->turn_yaw_prev_error = error;

    turn_cmd = (g_motionPid.turn_yaw.kp * error) +
               (g_motionPid.turn_yaw.ki * rt->turn_yaw_i) -
               (g_motionPid.turn_yaw.kd * pose->w_dps);

    if (turn_cmd > 0.0f) {
        rt->last_left_pwm = 0;
        rt->last_right_pwm = (int16_t)clamp_f32(fabsf(turn_cmd),
                                                g_motionPid.turn_yaw.pwm_min,
                                                g_motionPid.turn_yaw.pwm_max);
    } else {
        rt->last_left_pwm = (int16_t)clamp_f32(fabsf(turn_cmd),
                                               g_motionPid.turn_yaw.pwm_min,
                                               g_motionPid.turn_yaw.pwm_max);
        rt->last_right_pwm = 0;
    }
    motion_drive_single_wheel_cmd(turn_cmd);
}

static void motion_update_arc(Motion_Runtime_t *rt, const INS_Pose_t *pose)
{
    float actual_left_mps = 0.0f;
    float actual_right_mps = 0.0f;
    float speed_dt = (float)MOTION_TASK_PERIOD_MS / 1000.0f;
    float left_ff_pwm;
    float right_ff_pwm;
    float left_pwm_f;
    float right_pwm_f;
    float left_trim;
    float right_trim;
    int16_t left_pwm;
    int16_t right_pwm;
    float progress = wrap_180(pose->yaw_deg - rt->start_pose.yaw_deg);
    float error = wrap_180(rt->target_yaw_deg - pose->yaw_deg);
    uint32_t elapsed_ms = (uint32_t)((xTaskGetTickCount() - rt->start_tick) * portTICK_PERIOD_MS);
    bool speed_ok;

    rt->last_progress = progress;
    rt->last_error = error;
    rt->last_yaw_error = error;

    if (fabsf(error) <= MOTION_ARC_YAW_TOL_DEG) {
        motion_finish(rt, "done", MOTION_RESULT_DONE);
        return;
    }
    if (elapsed_ms >= rt->arc_timeout_ms) {
        motion_finish(rt, "timeout", MOTION_RESULT_TIMEOUT);
        return;
    }

    left_ff_pwm = (rt->arc_target_left_mps / MOTION_STRAIGHT_BASE_MPS) *
                  (float)MOTION_STRAIGHT_SPEED_PCT;
    right_ff_pwm = (rt->arc_target_right_mps / MOTION_STRAIGHT_BASE_MPS) *
                   (float)MOTION_STRAIGHT_SPEED_PCT;
    left_pwm_f = left_ff_pwm;
    right_pwm_f = right_ff_pwm;

    speed_ok = motion_read_wheel_speeds(rt, &actual_left_mps, &actual_right_mps, &speed_dt);
    if (speed_ok) {
        if (!rt->arc_filter_ready) {
            rt->arc_filt_left_mps = actual_left_mps;
            rt->arc_filt_right_mps = actual_right_mps;
            rt->arc_filter_ready = true;
        } else {
            rt->arc_filt_left_mps += MOTION_ARC_SPEED_FILTER_ALPHA *
                                     (actual_left_mps - rt->arc_filt_left_mps);
            rt->arc_filt_right_mps += MOTION_ARC_SPEED_FILTER_ALPHA *
                                      (actual_right_mps - rt->arc_filt_right_mps);
        }

        actual_left_mps = rt->arc_filt_left_mps;
        actual_right_mps = rt->arc_filt_right_mps;

        left_trim = motion_speed_pid_step(rt->arc_target_left_mps,
                                          actual_left_mps,
                                          speed_dt,
                                          &rt->left_speed_i,
                                          &rt->left_speed_prev_error);
        right_trim = motion_speed_pid_step(rt->arc_target_right_mps,
                                           actual_right_mps,
                                           speed_dt,
                                           &rt->right_speed_i,
                                           &rt->right_speed_prev_error);
        left_pwm_f += clamp_f32(left_trim,
                                -MOTION_ARC_PID_TRIM_LIMIT_PCT,
                                MOTION_ARC_PID_TRIM_LIMIT_PCT);
        right_pwm_f += clamp_f32(right_trim,
                                 -MOTION_ARC_PID_TRIM_LIMIT_PCT,
                                 MOTION_ARC_PID_TRIM_LIMIT_PCT);
    } else if (rt->arc_filter_ready) {
        actual_left_mps = rt->arc_filt_left_mps;
        actual_right_mps = rt->arc_filt_right_mps;
    }

    left_pwm_f = slew_f32((float)rt->last_left_pwm,
                          left_pwm_f,
                          MOTION_ARC_PWM_SLEW_PCT_PER_UPDATE);
    right_pwm_f = slew_f32((float)rt->last_right_pwm,
                           right_pwm_f,
                           MOTION_ARC_PWM_SLEW_PCT_PER_UPDATE);
    left_pwm = (int16_t)clamp_f32(left_pwm_f, 0.0f, 100.0f);
    right_pwm = (int16_t)clamp_f32(right_pwm_f, 0.0f, 100.0f);
    motion_set_pwm_record(rt, left_pwm, right_pwm);
    g_motionRtStatus.target_left_mps = rt->arc_target_left_mps;
    g_motionRtStatus.target_right_mps = rt->arc_target_right_mps;
    g_motionRtStatus.actual_left_mps = actual_left_mps;
    g_motionRtStatus.actual_right_mps = actual_right_mps;

    motion_send_arc_justfloat(rt->arc_target_left_mps,
                              actual_left_mps,
                              rt->arc_target_right_mps,
                              actual_right_mps,
                              rt->target_yaw_deg,
                              pose->yaw_deg,
                              (float)left_pwm,
                              (float)right_pwm);

    motion_drive_arc(left_pwm, right_pwm);
}

static void motion_update_runtime_status(const Motion_Runtime_t *rt)
{
    switch (rt->state) {
        case MOTION_STATE_FWD:  g_motionRtStatus.state = MOTION_RT_FWD;  break;
        case MOTION_STATE_BACK: g_motionRtStatus.state = MOTION_RT_BACK; break;
        case MOTION_STATE_TURN: g_motionRtStatus.state = MOTION_RT_TURN; break;
        case MOTION_STATE_ARC:  g_motionRtStatus.state = MOTION_RT_ARC;  break;
        default:                g_motionRtStatus.state = MOTION_RT_IDLE; break;
    }
    g_motionRtStatus.target_yaw_deg = rt->target_yaw_deg;
    g_motionRtStatus.actual_yaw_deg = rt->last_pose.yaw_deg;
}

static void motion_update(Motion_Runtime_t *rt)
{
    INS_Pose_t pose;

    if (rt->state == MOTION_STATE_IDLE) {
        (void)get_latest_pose(&rt->last_pose);
        motion_update_runtime_status(rt);
        return;
    }

    if (!get_latest_pose(&pose) || !pose_ready(&pose)) {
        motion_finish(rt, "abort_ins_lost", MOTION_RESULT_ABORTED);
        motion_update_runtime_status(rt);
        return;
    }

    rt->last_pose = pose;

    if (rt->state == MOTION_STATE_FWD || rt->state == MOTION_STATE_BACK) {
        motion_update_straight(rt, &pose);
    } else if (rt->state == MOTION_STATE_TURN) {
        motion_update_turn(rt, &pose);
    } else if (rt->state == MOTION_STATE_ARC) {
        motion_update_arc(rt, &pose);
    }
    motion_update_runtime_status(rt);
}

void motion_task(void *pvParameters)
{
    Motion_Runtime_t rt;
    TickType_t lastWake;

    (void)pvParameters;

    memset(&rt, 0, sizeof(rt));
    rt.state = MOTION_STATE_IDLE;
    (void)get_latest_pose(&rt.start_pose);
    rt.last_pose = rt.start_pose;
    lastWake = xTaskGetTickCount();

    while (1) {
        Motion_Command_t cmd;

        while (g_motionCmdQueue != NULL &&
               xQueueReceive(g_motionCmdQueue, &cmd, 0) == pdTRUE) {
            motion_handle_command(&rt, &cmd);
        }

        motion_update(&rt);
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTION_TASK_PERIOD_MS));
    }
}
