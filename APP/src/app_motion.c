/**
 * @file    app_motion.c
 * @brief   Basic INS-based straight and single-wheel turn motion control.
 */
#include "app_motion.h"
#include "app_ins.h"
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

#define MOTION_TASK_PERIOD_MS       20U
#define MOTION_STRAIGHT_TIMEOUT_MS  8000U
#define MOTION_TURN_TIMEOUT_MS      6000U

#define MOTION_STRAIGHT_SPEED_PCT   25

#define MOTION_DIST_TOL_M           0.02f
#define MOTION_MIN_DIST_M           0.01f
#define MOTION_MAX_DIST_M           5.00f
#define MOTION_MIN_TURN_DEG         0.5f
#define MOTION_MAX_TURN_DEG         180.0f
#define MOTION_PID_INTEGRAL_LIMIT   50.0f

#define DEG_TO_RAD                  0.01745329252f

typedef enum {
    MOTION_STATE_IDLE = 0,
    MOTION_STATE_FWD,
    MOTION_STATE_BACK,
    MOTION_STATE_TURN
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
    float straight_yaw_i;
    float straight_yaw_prev_error;
    float turn_yaw_i;
    float turn_yaw_prev_error;
    int16_t last_left_pwm;
    int16_t last_right_pwm;
    bool turn_acquired;
    TickType_t start_tick;
} Motion_Runtime_t;

QueueHandle_t g_motionCmdQueue = NULL;
Motion_PidConfig_t g_motionPid = {
    {0.60f, 0.00f, 0.04f, 15.0f},
    {0.90f, 0.00f, 0.08f, 1.5f, 12.0f, 22.0f}
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

static bool pose_ready(const INS_Pose_t *pose)
{
    uint32_t required = INS_FLAG_IMU_VALID | INS_FLAG_YAW_ZERO_READY;
    return pose != NULL && ((pose->flags & required) == required);
}

static bool get_latest_pose(INS_Pose_t *pose)
{
    if (pose == NULL || g_insPoseQueue == NULL) {
        return false;
    }

    return xQueuePeek(g_insPoseQueue, pose, 0) == pdTRUE;
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

static void motion_drive_single_wheel_cmd(float turn_cmd)
{
    int16_t pwm = (int16_t)clamp_f32(fabsf(turn_cmd),
                                     g_motionPid.turn_yaw.pwm_min,
                                     g_motionPid.turn_yaw.pwm_max);

    if (turn_cmd > 0.0f) {
        send_motor_cmd(MOTOR_CMD_RIGHT_ONLY, 0);
        send_motor_cmd(MOTOR_CMD_LEFT_SPEED, 0);
        send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, pwm);
    } else {
        send_motor_cmd(MOTOR_CMD_LEFT_ONLY, 0);
        send_motor_cmd(MOTOR_CMD_LEFT_SPEED, pwm);
        send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, 0);
    }
    send_motor_cmd(MOTOR_CMD_ONOFF, 1);
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
        case MOTION_STATE_IDLE:
        default:
            return "IDLE";
    }
}

static void motion_print_help(void)
{
    LOG_RAW("[MOTION_CMD] commands:\r\n");
    LOG_RAW("[MOTION_CMD]   motion help\r\n");
    LOG_RAW("[MOTION_CMD]   motion status\r\n");
    LOG_RAW("[MOTION_CMD]   motion stop\r\n");
    LOG_RAW("[MOTION_CMD]   motion fwd <meters>\r\n");
    LOG_RAW("[MOTION_CMD]   motion back <meters>\r\n");
    LOG_RAW("[MOTION_CMD]   motion turn <-180..180 deg>\r\n");
}

static void motion_print_status(const Motion_Runtime_t *rt)
{
    LOG_RAW("[MOTION_CMD] state=%s target=%.3f target_yaw=%+.2f progress=%.3f err=%.3f yaw_err=%+.2f pwm L=%d R=%d X=%+.3f Y=%+.3f YAW=%+.2f W=%+.2f flags=0x%08lX\r\n",
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
            (unsigned long)rt->last_pose.flags);
    LOG_RAW("[MOTION_CMD] StraightYaw Kp=%.2f Ki=%.2f Kd=%.2f TrimMax=%.1f\r\n",
            g_motionPid.straight_yaw.kp,
            g_motionPid.straight_yaw.ki,
            g_motionPid.straight_yaw.kd,
            g_motionPid.straight_yaw.trim_max);
    LOG_RAW("[MOTION_CMD] TurnYaw Kp=%.2f Ki=%.2f Kd=%.2f Dead=%.1f PwmMin=%.1f PwmMax=%.1f\r\n",
            g_motionPid.turn_yaw.kp,
            g_motionPid.turn_yaw.ki,
            g_motionPid.turn_yaw.kd,
            g_motionPid.turn_yaw.deadband_deg,
            g_motionPid.turn_yaw.pwm_min,
            g_motionPid.turn_yaw.pwm_max);
}

static bool motion_start_straight(Motion_Runtime_t *rt, Motion_State_t state, float distance_m)
{
    INS_Pose_t pose;

    if (rt->state != MOTION_STATE_IDLE) {
        if (rt->state == MOTION_STATE_TURN && rt->turn_acquired) {
            motion_stop_motors();
            rt->state = MOTION_STATE_IDLE;
        } else {
            LOG_RAW("[MOTION_CMD] error: busy, use motion stop first\r\n");
            return false;
        }
    }
    if (rt->state != MOTION_STATE_IDLE) {
        LOG_RAW("[MOTION_CMD] error: busy, use motion stop first\r\n");
        return false;
    }
    if (distance_m < MOTION_MIN_DIST_M || distance_m > MOTION_MAX_DIST_M) {
        LOG_RAW("[MOTION_CMD] error: distance range %.2f..%.2fm\r\n",
                MOTION_MIN_DIST_M, MOTION_MAX_DIST_M);
        return false;
    }
    if (!get_latest_pose(&pose) || !pose_ready(&pose)) {
        LOG_RAW("[MOTION_CMD] error: INS not ready\r\n");
        return false;
    }

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
    rt->last_left_pwm = MOTION_STRAIGHT_SPEED_PCT;
    rt->last_right_pwm = MOTION_STRAIGHT_SPEED_PCT;
    rt->start_tick = xTaskGetTickCount();

    motion_drive_straight(state == MOTION_STATE_FWD,
                          MOTION_STRAIGHT_SPEED_PCT,
                          MOTION_STRAIGHT_SPEED_PCT);

    LOG_RAW("[MOTION] start %s dist=%.3fm yaw=%+.2f\r\n",
            motion_state_name(state), distance_m, pose.yaw_deg);
    return true;
}

static bool motion_start_turn(Motion_Runtime_t *rt, float angle_deg)
{
    INS_Pose_t pose;

    if (rt->state != MOTION_STATE_IDLE && rt->state != MOTION_STATE_TURN) {
        LOG_RAW("[MOTION_CMD] error: busy, use motion stop first\r\n");
        return false;
    }
    if (fabsf(angle_deg) < MOTION_MIN_TURN_DEG || fabsf(angle_deg) > MOTION_MAX_TURN_DEG) {
        LOG_RAW("[MOTION_CMD] error: turn range -180..180 deg, zero rejected\r\n");
        return false;
    }
    if (!get_latest_pose(&pose) || !pose_ready(&pose)) {
        LOG_RAW("[MOTION_CMD] error: INS not ready\r\n");
        return false;
    }

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

    LOG_RAW("[MOTION] start TURN angle=%+.1f yaw=%+.2f target_yaw=%+.2f mode=%s\r\n",
            angle_deg,
            pose.yaw_deg,
            rt->target_yaw_deg,
            (angle_deg > 0.0f) ? "left_stop/right_move" : "right_stop/left_move");
    return true;
}

static void motion_finish(Motion_Runtime_t *rt, const char *reason)
{
    motion_stop_motors();
    rt->last_left_pwm = 0;
    rt->last_right_pwm = 0;
    LOG_RAW("[MOTION] %s state=%s progress=%.3f err=%.3f yaw_err=%+.2f X=%+.3f Y=%+.3f YAW=%+.2f\r\n",
            reason,
            motion_state_name(rt->state),
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
                LOG_RAW("[MOTION] stop ok, already idle\r\n");
            } else {
                motion_finish(rt, "stopped");
            }
            break;
        case MOTION_CMD_FWD:
            (void)motion_start_straight(rt, MOTION_STATE_FWD, cmd->value);
            break;
        case MOTION_CMD_BACK:
            (void)motion_start_straight(rt, MOTION_STATE_BACK, cmd->value);
            break;
        case MOTION_CMD_TURN:
            (void)motion_start_turn(rt, cmd->value);
            break;
        default:
            LOG_RAW("[MOTION_CMD] error: bad command\r\n");
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
    int16_t trim;
    int16_t left;
    int16_t right;
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
    trim = (int16_t)trim_f;

    if (forward) {
        left = MOTION_STRAIGHT_SPEED_PCT - trim;
        right = MOTION_STRAIGHT_SPEED_PCT + trim;
    } else {
        left = MOTION_STRAIGHT_SPEED_PCT + trim;
        right = MOTION_STRAIGHT_SPEED_PCT - trim;
    }

    rt->last_progress = progress;
    rt->last_error = rt->target_value - progress;
    rt->last_yaw_error = yaw_error;

    if (progress >= (rt->target_value - MOTION_DIST_TOL_M)) {
        motion_finish(rt, "done");
        return;
    }
    if (elapsed_ms >= MOTION_STRAIGHT_TIMEOUT_MS) {
        motion_finish(rt, "timeout");
        return;
    }

    motion_set_pwm_record(rt, left, right);
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
        rt->turn_acquired = true;
        rt->turn_yaw_i = 0.0f;
        rt->turn_yaw_prev_error = error;
        motion_stop_motors();
        rt->last_left_pwm = 0;
        rt->last_right_pwm = 0;
        return;
    }

    if (!rt->turn_acquired && elapsed_ms >= MOTION_TURN_TIMEOUT_MS) {
        motion_finish(rt, "timeout");
        return;
    }

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

static void motion_update(Motion_Runtime_t *rt)
{
    INS_Pose_t pose;

    if (rt->state == MOTION_STATE_IDLE) {
        (void)get_latest_pose(&rt->last_pose);
        return;
    }

    if (!get_latest_pose(&pose) || !pose_ready(&pose)) {
        motion_finish(rt, "abort_ins_lost");
        return;
    }

    rt->last_pose = pose;

    if (rt->state == MOTION_STATE_FWD || rt->state == MOTION_STATE_BACK) {
        motion_update_straight(rt, &pose);
    } else if (rt->state == MOTION_STATE_TURN) {
        motion_update_turn(rt, &pose);
    }
}

void motion_task(void *pvParameters)
{
    Motion_Runtime_t rt;

    (void)pvParameters;

    memset(&rt, 0, sizeof(rt));
    rt.state = MOTION_STATE_IDLE;
    (void)get_latest_pose(&rt.start_pose);
    rt.last_pose = rt.start_pose;

    LOG_INFO("[MOTION] task ready: motion help\r\n");

    while (1) {
        Motion_Command_t cmd;

        while (g_motionCmdQueue != NULL &&
               xQueueReceive(g_motionCmdQueue, &cmd, 0) == pdTRUE) {
            motion_handle_command(&rt, &cmd);
        }

        motion_update(&rt);
        vTaskDelay(pdMS_TO_TICKS(MOTION_TASK_PERIOD_MS));
    }
}
