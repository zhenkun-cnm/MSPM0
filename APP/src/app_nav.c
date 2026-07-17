/**
 * @file    app_nav.c
 * @brief   INS pose navigation layer built on motion primitives.
 */
#include "app_nav.h"
#include "app_ins.h"
#include "app_motion.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define NAV_TASK_PERIOD_MS          50U
#define NAV_POS_TOL_M               0.05f
#define NAV_YAW_TOL_DEG             3.0f
#define NAV_MIN_DRIVE_M             0.03f
#define NAV_MAX_LEG_M               5.0f
#define NAV_MOTION_START_TIMEOUT_MS 500U
#define NAV_SQUARE_TURN_DEG         90.0f
#define NAV_SQUARE_LEGS             4U

#define NAV_RAD_TO_DEG              57.295779513082320876f

QueueHandle_t g_navCmdQueue = NULL;

typedef struct {
    Nav_State_t state;
    bool square_active;
    uint8_t square_leg;
    float square_side_m;
    float target_x_m;
    float target_y_m;
    float target_yaw_deg;
    float target_heading_deg;
    float drive_distance_m;
    float dist_error_m;
    float yaw_error_deg;
    uint32_t next_motion_cmd_id;
    uint32_t waiting_motion_cmd_id;
    bool motion_cmd_sent;
    bool motion_accepted;
    TickType_t motion_cmd_tick;
} Nav_Runtime_t;

static float nav_wrap_180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static const char *nav_state_name(Nav_State_t state)
{
    switch (state) {
        case NAV_STATE_IDLE:              return "IDLE";
        case NAV_STATE_TURN_TO_TARGET:    return "TURN_TO_TARGET";
        case NAV_STATE_DRIVE_TO_TARGET:   return "DRIVE_TO_TARGET";
        case NAV_STATE_TURN_TO_FINAL_YAW: return "TURN_TO_FINAL_YAW";
        case NAV_STATE_SQUARE_DRIVE:      return "SQUARE_DRIVE";
        case NAV_STATE_SQUARE_TURN:       return "SQUARE_TURN";
        case NAV_STATE_DONE:              return "DONE";
        case NAV_STATE_ERROR:             return "ERROR";
        default:                          return "?";
    }
}

static const char *nav_motion_result_name(Motion_Result_t result)
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

static bool nav_pose_ready(const INS_Pose_t *pose)
{
    uint32_t required = INS_FLAG_IMU_VALID | INS_FLAG_YAW_ZERO_READY;
    return pose != NULL && ((pose->flags & required) == required);
}

static bool nav_get_pose(INS_Pose_t *pose)
{
    return (pose != NULL) && INS_Pose_Read(pose) && nav_pose_ready(pose);
}

static bool nav_motion_idle(void)
{
    return g_motionRtStatus.state == MOTION_RT_IDLE;
}

static float nav_distance_to_target(Nav_Runtime_t *rt, const INS_Pose_t *pose)
{
    float dx = rt->target_x_m - pose->x_m;
    float dy = rt->target_y_m - pose->y_m;
    float dist = sqrtf((dx * dx) + (dy * dy));
    rt->dist_error_m = dist;
    return dist;
}

static void nav_update_errors(Nav_Runtime_t *rt, const INS_Pose_t *pose)
{
    if (pose == NULL) {
        return;
    }
    (void)nav_distance_to_target(rt, pose);
    rt->yaw_error_deg = nav_wrap_180(rt->target_yaw_deg - pose->yaw_deg);
}

static bool nav_send_motion_raw(Motion_CommandType_t type, float value, uint32_t cmd_id)
{
    Motion_Command_t cmd;
    cmd.type = type;
    cmd.value = value;
    cmd.cmd_id = cmd_id;

    if (g_motionCmdQueue == NULL) {
        LOG_RAW("[NAV] error: motion queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_motionCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOG_RAW("[NAV] error: motion queue full\r\n");
        return false;
    }

    return true;
}

static void nav_reset_motion_wait(Nav_Runtime_t *rt)
{
    rt->waiting_motion_cmd_id = 0U;
    rt->motion_cmd_sent = false;
    rt->motion_accepted = false;
    rt->motion_cmd_tick = 0U;
}

static bool nav_start_motion_step(Nav_Runtime_t *rt, Motion_CommandType_t type, float value)
{
    uint32_t cmd_id;

    if (!nav_motion_idle()) {
        LOG_RAW("[NAV] error: motion busy\r\n");
        return false;
    }

    cmd_id = ++rt->next_motion_cmd_id;
    if (cmd_id == 0U) {
        cmd_id = ++rt->next_motion_cmd_id;
    }

    if (!nav_send_motion_raw(type, value, cmd_id)) {
        return false;
    }

    rt->waiting_motion_cmd_id = cmd_id;
    rt->motion_cmd_sent = true;
    rt->motion_accepted = false;
    rt->motion_cmd_tick = xTaskGetTickCount();
    return true;
}

static int8_t nav_motion_step_result(Nav_Runtime_t *rt)
{
    uint32_t elapsed_ms;

    if (!rt->motion_cmd_sent) {
        return 0;
    }

    if (g_motionRtStatus.rejected_cmd_id == rt->waiting_motion_cmd_id) {
        LOG_RAW("[NAV] error: motion rejected cmd_id=%lu\r\n",
                (unsigned long)rt->waiting_motion_cmd_id);
        return -1;
    }

    if (g_motionRtStatus.active_cmd_id == rt->waiting_motion_cmd_id) {
        rt->motion_accepted = true;
    }

    if (g_motionRtStatus.done_cmd_id == rt->waiting_motion_cmd_id) {
        if (g_motionRtStatus.last_result == MOTION_RESULT_DONE) {
            return 1;
        }
        LOG_RAW("[NAV] error: motion finished cmd_id=%lu result=%s\r\n",
                (unsigned long)rt->waiting_motion_cmd_id,
                nav_motion_result_name(g_motionRtStatus.last_result));
        return -1;
    }

    elapsed_ms = (uint32_t)((xTaskGetTickCount() - rt->motion_cmd_tick) * portTICK_PERIOD_MS);
    if (!rt->motion_accepted && elapsed_ms >= NAV_MOTION_START_TIMEOUT_MS) {
        LOG_RAW("[NAV] error: motion did not start cmd_id=%lu\r\n",
                (unsigned long)rt->waiting_motion_cmd_id);
        return -1;
    }

    return 0;
}

static void nav_enter_error(Nav_Runtime_t *rt, const char *reason)
{
    (void)nav_send_motion_raw(MOTION_CMD_STOP, 0.0f, 0U);
    rt->state = NAV_STATE_ERROR;
    rt->square_active = false;
    nav_reset_motion_wait(rt);
    LOG_RAW("[NAV] error: %s\r\n", reason);
}

static void nav_print_help(void)
{
    LOG_RAW("[NAV_CMD] commands:\r\n");
    LOG_RAW("  nav help\r\n");
    LOG_RAW("  nav status\r\n");
    LOG_RAW("  nav stop\r\n");
    LOG_RAW("  nav goto <x_m> <y_m> <yaw_deg>\r\n");
    LOG_RAW("  nav square <side_m>\r\n");
}

static void nav_print_status(Nav_Runtime_t *rt)
{
    INS_Pose_t pose;
    bool ok = nav_get_pose(&pose);

    if (ok) {
        nav_update_errors(rt, &pose);
        LOG_RAW("[NAV_CMD] state=%s target=(%+.3f,%+.3f,%+.1f) pose=(%+.3f,%+.3f,%+.1f) dist_err=%.3f yaw_err=%+.2f motion=%d wait=%lu active=%lu done=%lu rejected=%lu result=%s square=%u/%u\r\n",
                nav_state_name(rt->state),
                rt->target_x_m,
                rt->target_y_m,
                rt->target_yaw_deg,
                pose.x_m,
                pose.y_m,
                pose.yaw_deg,
                rt->dist_error_m,
                rt->yaw_error_deg,
                (int)g_motionRtStatus.state,
                (unsigned long)rt->waiting_motion_cmd_id,
                (unsigned long)g_motionRtStatus.active_cmd_id,
                (unsigned long)g_motionRtStatus.done_cmd_id,
                (unsigned long)g_motionRtStatus.rejected_cmd_id,
                nav_motion_result_name(g_motionRtStatus.last_result),
                rt->square_active ? (unsigned)(rt->square_leg + 1U) : 0U,
                rt->square_active ? NAV_SQUARE_LEGS : 0U);
    } else {
        LOG_RAW("[NAV_CMD] state=%s pose=not_ready motion=%d wait=%lu active=%lu done=%lu rejected=%lu result=%s\r\n",
                nav_state_name(rt->state),
                (int)g_motionRtStatus.state,
                (unsigned long)rt->waiting_motion_cmd_id,
                (unsigned long)g_motionRtStatus.active_cmd_id,
                (unsigned long)g_motionRtStatus.done_cmd_id,
                (unsigned long)g_motionRtStatus.rejected_cmd_id,
                nav_motion_result_name(g_motionRtStatus.last_result));
    }
}

static bool nav_prepare_goto(Nav_Runtime_t *rt,
                             float x_m,
                             float y_m,
                             float yaw_deg,
                             bool internal_call)
{
    INS_Pose_t pose;
    float dx;
    float dy;
    float dist;

    if (rt->state != NAV_STATE_IDLE &&
        rt->state != NAV_STATE_DONE &&
        rt->state != NAV_STATE_ERROR &&
        !internal_call) {
        LOG_RAW("[NAV_CMD] error: busy, use nav stop first\r\n");
        return false;
    }

    if (!nav_motion_idle()) {
        LOG_RAW("[NAV_CMD] error: motion busy\r\n");
        return false;
    }

    if (!nav_get_pose(&pose)) {
        LOG_RAW("[NAV_CMD] error: INS not ready\r\n");
        return false;
    }

    dx = x_m - pose.x_m;
    dy = y_m - pose.y_m;
    dist = sqrtf((dx * dx) + (dy * dy));
    if (dist > NAV_MAX_LEG_M) {
        LOG_RAW("[NAV_CMD] error: leg too long, max %.1fm\r\n", NAV_MAX_LEG_M);
        return false;
    }

    rt->square_active = false;
    rt->target_x_m = x_m;
    rt->target_y_m = y_m;
    rt->target_yaw_deg = nav_wrap_180(yaw_deg);
    rt->drive_distance_m = dist;
    rt->dist_error_m = dist;
    rt->target_heading_deg = (dist <= NAV_POS_TOL_M)
                                 ? pose.yaw_deg
                                 : (atan2f(dy, dx) * NAV_RAD_TO_DEG);
    rt->yaw_error_deg = nav_wrap_180(rt->target_heading_deg - pose.yaw_deg);
    rt->state = (dist <= NAV_POS_TOL_M) ? NAV_STATE_TURN_TO_FINAL_YAW
                                        : NAV_STATE_TURN_TO_TARGET;
    nav_reset_motion_wait(rt);

    LOG_RAW("[NAV] goto target=(%+.3f,%+.3f,%+.1f) heading=%+.1f dist=%.3f\r\n",
            rt->target_x_m,
            rt->target_y_m,
            rt->target_yaw_deg,
            rt->target_heading_deg,
            rt->drive_distance_m);
    return true;
}

static void nav_start_square(Nav_Runtime_t *rt, float side_m)
{
    INS_Pose_t pose;

    if (side_m <= 0.0f || side_m > NAV_MAX_LEG_M) {
        LOG_RAW("[NAV_CMD] error: usage nav square <0..%.1fm>\r\n", NAV_MAX_LEG_M);
        return;
    }

    if (rt->state != NAV_STATE_IDLE &&
        rt->state != NAV_STATE_DONE &&
        rt->state != NAV_STATE_ERROR) {
        LOG_RAW("[NAV_CMD] error: busy, use nav stop first\r\n");
        return;
    }

    if (!nav_get_pose(&pose)) {
        LOG_RAW("[NAV_CMD] error: INS not ready\r\n");
        return;
    }

    rt->square_active = true;
    rt->square_leg = 0U;
    rt->square_side_m = side_m;
    rt->target_x_m = pose.x_m;
    rt->target_y_m = pose.y_m;
    rt->target_yaw_deg = pose.yaw_deg;
    rt->target_heading_deg = pose.yaw_deg;
    rt->dist_error_m = 0.0f;
    rt->yaw_error_deg = 0.0f;
    rt->state = NAV_STATE_SQUARE_DRIVE;
    nav_reset_motion_wait(rt);

    LOG_RAW("[NAV] square start side=%.3fm pose=(%+.3f,%+.3f,%+.1f)\r\n",
            side_m,
            pose.x_m,
            pose.y_m,
            pose.yaw_deg);
}

static void nav_handle_command(Nav_Runtime_t *rt, const Nav_Command_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    switch (cmd->type) {
        case NAV_CMD_HELP:
            nav_print_help();
            break;
        case NAV_CMD_STATUS:
            nav_print_status(rt);
            break;
        case NAV_CMD_STOP:
            (void)nav_send_motion_raw(MOTION_CMD_STOP, 0.0f, 0U);
            rt->state = NAV_STATE_IDLE;
            rt->square_active = false;
            nav_reset_motion_wait(rt);
            LOG_RAW("[NAV] stop ok\r\n");
            break;
        case NAV_CMD_GOTO:
            (void)nav_prepare_goto(rt, cmd->x_m, cmd->y_m, cmd->yaw_deg, false);
            break;
        case NAV_CMD_SQUARE:
            nav_start_square(rt, cmd->side_m);
            break;
        default:
            LOG_RAW("[NAV_CMD] error: bad command\r\n");
            break;
    }
}

static void nav_finish_goto(Nav_Runtime_t *rt)
{
    INS_Pose_t pose;

    if (nav_get_pose(&pose)) {
        nav_update_errors(rt, &pose);
    }

    rt->state = NAV_STATE_DONE;
    nav_reset_motion_wait(rt);
    LOG_RAW("[NAV] done target=(%+.3f,%+.3f,%+.1f) dist_err=%.3f yaw_err=%+.2f\r\n",
            rt->target_x_m,
            rt->target_y_m,
            rt->target_yaw_deg,
            rt->dist_error_m,
            rt->yaw_error_deg);
}

static void nav_finish_square(Nav_Runtime_t *rt)
{
    INS_Pose_t pose;

    rt->state = NAV_STATE_DONE;
    rt->square_active = false;
    nav_reset_motion_wait(rt);

    if (nav_get_pose(&pose)) {
        LOG_RAW("[NAV] square done pose=(%+.3f,%+.3f,%+.1f)\r\n",
                pose.x_m,
                pose.y_m,
                pose.yaw_deg);
    } else {
        LOG_RAW("[NAV] square done pose=not_ready\r\n");
    }
}

static void nav_update_goto(Nav_Runtime_t *rt, const INS_Pose_t *pose)
{
    float dist;
    float turn_delta;
    int8_t motion_result;

    switch (rt->state) {
        case NAV_STATE_TURN_TO_TARGET:
            dist = nav_distance_to_target(rt, pose);
            if (dist <= NAV_POS_TOL_M) {
                rt->state = NAV_STATE_TURN_TO_FINAL_YAW;
                nav_reset_motion_wait(rt);
                break;
            }

            rt->target_heading_deg = atan2f(rt->target_y_m - pose->y_m,
                                            rt->target_x_m - pose->x_m) * NAV_RAD_TO_DEG;
            turn_delta = nav_wrap_180(rt->target_heading_deg - pose->yaw_deg);
            rt->yaw_error_deg = turn_delta;

            if (fabsf(turn_delta) <= NAV_YAW_TOL_DEG) {
                rt->state = NAV_STATE_DRIVE_TO_TARGET;
                nav_reset_motion_wait(rt);
                break;
            }

            if (!rt->motion_cmd_sent) {
                LOG_RAW("[NAV] turn_to_target delta=%+.1f target_heading=%+.1f\r\n",
                        turn_delta,
                        rt->target_heading_deg);
                if (!nav_start_motion_step(rt, MOTION_CMD_TURN, turn_delta)) {
                    nav_enter_error(rt, "motion command failed");
                }
            } else {
                motion_result = nav_motion_step_result(rt);
                if (motion_result > 0) {
                    rt->state = NAV_STATE_DRIVE_TO_TARGET;
                    nav_reset_motion_wait(rt);
                } else if (motion_result < 0) {
                    nav_enter_error(rt, "motion step failed");
                }
            }
            break;

        case NAV_STATE_DRIVE_TO_TARGET:
            dist = nav_distance_to_target(rt, pose);
            if (dist <= NAV_POS_TOL_M || dist <= NAV_MIN_DRIVE_M) {
                rt->state = NAV_STATE_TURN_TO_FINAL_YAW;
                nav_reset_motion_wait(rt);
                break;
            }
            if (dist > NAV_MAX_LEG_M) {
                nav_enter_error(rt, "remaining distance too long");
                break;
            }

            if (!rt->motion_cmd_sent) {
                rt->drive_distance_m = dist;
                LOG_RAW("[NAV] drive_to_target dist=%.3f\r\n", dist);
                if (!nav_start_motion_step(rt, MOTION_CMD_FWD, dist)) {
                    nav_enter_error(rt, "motion command failed");
                }
            } else {
                motion_result = nav_motion_step_result(rt);
                if (motion_result > 0) {
                    rt->state = NAV_STATE_TURN_TO_FINAL_YAW;
                    nav_reset_motion_wait(rt);
                } else if (motion_result < 0) {
                    nav_enter_error(rt, "motion step failed");
                }
            }
            break;

        case NAV_STATE_TURN_TO_FINAL_YAW:
            turn_delta = nav_wrap_180(rt->target_yaw_deg - pose->yaw_deg);
            rt->yaw_error_deg = turn_delta;

            if (fabsf(turn_delta) <= NAV_YAW_TOL_DEG) {
                nav_finish_goto(rt);
                break;
            }

            if (!rt->motion_cmd_sent) {
                LOG_RAW("[NAV] turn_to_final delta=%+.1f final_yaw=%+.1f\r\n",
                        turn_delta,
                        rt->target_yaw_deg);
                if (!nav_start_motion_step(rt, MOTION_CMD_TURN, turn_delta)) {
                    nav_enter_error(rt, "motion command failed");
                }
            } else {
                motion_result = nav_motion_step_result(rt);
                if (motion_result > 0) {
                    nav_finish_goto(rt);
                } else if (motion_result < 0) {
                    nav_enter_error(rt, "motion step failed");
                }
            }
            break;

        default:
            break;
    }
}

static void nav_update_square(Nav_Runtime_t *rt)
{
    int8_t motion_result;

    if (rt->state == NAV_STATE_SQUARE_DRIVE) {
        if (!rt->motion_cmd_sent) {
            LOG_RAW("[NAV] square drive %u/%u dist=%.3f\r\n",
                    (unsigned)(rt->square_leg + 1U),
                    NAV_SQUARE_LEGS,
                    rt->square_side_m);
            if (!nav_start_motion_step(rt, MOTION_CMD_FWD, rt->square_side_m)) {
                nav_enter_error(rt, "motion command failed");
            }
            return;
        }

        motion_result = nav_motion_step_result(rt);
        if (motion_result > 0) {
            rt->state = NAV_STATE_SQUARE_TURN;
            nav_reset_motion_wait(rt);
        } else if (motion_result < 0) {
            nav_enter_error(rt, "motion step failed");
        }
        return;
    }

    if (rt->state == NAV_STATE_SQUARE_TURN) {
        if (!rt->motion_cmd_sent) {
            LOG_RAW("[NAV] square turn %u/%u yaw_delta=%+.1f\r\n",
                    (unsigned)(rt->square_leg + 1U),
                    NAV_SQUARE_LEGS,
                    NAV_SQUARE_TURN_DEG);
            if (!nav_start_motion_step(rt, MOTION_CMD_TURN, NAV_SQUARE_TURN_DEG)) {
                nav_enter_error(rt, "motion command failed");
            }
            return;
        }

        motion_result = nav_motion_step_result(rt);
        if (motion_result > 0) {
            rt->square_leg++;
            nav_reset_motion_wait(rt);
            if (rt->square_leg >= NAV_SQUARE_LEGS) {
                nav_finish_square(rt);
            } else {
                rt->state = NAV_STATE_SQUARE_DRIVE;
            }
        } else if (motion_result < 0) {
            nav_enter_error(rt, "motion step failed");
        }
    }
}

static void nav_update(Nav_Runtime_t *rt)
{
    INS_Pose_t pose;

    if (rt->state == NAV_STATE_IDLE ||
        rt->state == NAV_STATE_DONE ||
        rt->state == NAV_STATE_ERROR) {
        return;
    }

    if (rt->state == NAV_STATE_SQUARE_DRIVE ||
        rt->state == NAV_STATE_SQUARE_TURN) {
        nav_update_square(rt);
        return;
    }

    if (!nav_get_pose(&pose)) {
        nav_enter_error(rt, "INS lost");
        return;
    }

    nav_update_errors(rt, &pose);
    nav_update_goto(rt, &pose);
}

void nav_task(void *pvParameters)
{
    Nav_Runtime_t rt;
    Nav_Command_t cmd;

    (void)pvParameters;
    memset(&rt, 0, sizeof(rt));
    rt.state = NAV_STATE_IDLE;

    LOG_INFO("[NAV] task ready: nav help\r\n");

    while (1) {
        while (g_navCmdQueue != NULL &&
               xQueueReceive(g_navCmdQueue, &cmd, 0) == pdTRUE) {
            nav_handle_command(&rt, &cmd);
        }

        nav_update(&rt);
        vTaskDelay(pdMS_TO_TICKS(NAV_TASK_PERIOD_MS));
    }
}
