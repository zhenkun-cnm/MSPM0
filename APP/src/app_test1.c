/**
 * @file    app_test1.c
 * @brief   Fixed Test1 competition route.
 */
#include "app_test1.h"

#if APP_TEST1_ENABLE

#include "app_ins.h"
#include "app_motion.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define TEST1_TASK_PERIOD_MS          50U
#define TEST1_POINT_COUNT             4U
#define TEST1_POS_TOL_M               0.05f
#define TEST1_YAW_TOL_DEG             3.0f
#define TEST1_MIN_DRIVE_M             0.03f
#define TEST1_MAX_LEG_M               5.0f
#define TEST1_RIGHT_TURN_DEG          (-90.0f)
#define TEST1_CMD_ID_BASE             0x71000000UL
#define TEST1_MOTION_START_TIMEOUT_MS 500U
#define TEST1_RAD_TO_DEG              57.295779513082320876f

QueueHandle_t g_test1CmdQueue = NULL;

typedef enum {
    TEST1_STATE_IDLE = 0,
    TEST1_STATE_TURN_TO_TARGET,
    TEST1_STATE_DRIVE_TO_TARGET,
    TEST1_STATE_RIGHT_TURN,
    TEST1_STATE_DONE,
    TEST1_STATE_ERROR
} Test1_State_t;

typedef struct {
    float x_m;
    float y_m;
} Test1_Point_t;

typedef struct {
    Test1_State_t state;
    uint8_t point_index;
    float target_x_m;
    float target_y_m;
    float target_heading_deg;
    float dist_error_m;
    float yaw_error_deg;
    uint32_t next_motion_cmd_id;
    uint32_t waiting_motion_cmd_id;
    bool motion_cmd_sent;
    bool motion_accepted;
    TickType_t motion_cmd_tick;
} Test1_Runtime_t;

static const Test1_Point_t s_test1Points[TEST1_POINT_COUNT] = {
    {0.90f,  0.00f},
    {0.96f, -0.90f},
    {0.04f, -0.96f},
    {0.00f,  0.00f}
};

static float test1_wrap_180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static const char *test1_state_name(Test1_State_t state)
{
    switch (state) {
        case TEST1_STATE_IDLE:           return "IDLE";
        case TEST1_STATE_TURN_TO_TARGET: return "TURN_TO_TARGET";
        case TEST1_STATE_DRIVE_TO_TARGET:return "DRIVE_TO_TARGET";
        case TEST1_STATE_RIGHT_TURN:     return "RIGHT_TURN";
        case TEST1_STATE_DONE:           return "DONE";
        case TEST1_STATE_ERROR:          return "ERROR";
        default:                         return "?";
    }
}

static const char *test1_motion_result_name(Motion_Result_t result)
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

static bool test1_pose_ready(const INS_Pose_t *pose)
{
    uint32_t required = INS_FLAG_IMU_VALID | INS_FLAG_YAW_ZERO_READY;
    return pose != NULL && ((pose->flags & required) == required);
}

static bool test1_get_pose(INS_Pose_t *pose)
{
    return (pose != NULL) && INS_Pose_Read(pose) && test1_pose_ready(pose);
}

static bool test1_motion_idle(void)
{
    return g_motionRtStatus.state == MOTION_RT_IDLE;
}

static float test1_distance_to_target(Test1_Runtime_t *rt, const INS_Pose_t *pose)
{
    float dx = rt->target_x_m - pose->x_m;
    float dy = rt->target_y_m - pose->y_m;
    float dist = sqrtf((dx * dx) + (dy * dy));
    rt->dist_error_m = dist;
    return dist;
}

static bool test1_send_motion_raw(Motion_CommandType_t type, float value, uint32_t cmd_id)
{
    Motion_Command_t cmd;
    cmd.type = type;
    cmd.value = value;
    cmd.value2 = 0.0f;
    cmd.cmd_id = cmd_id;

    if (g_motionCmdQueue == NULL) {
        LOG_RAW("[TEST1] error: motion queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_motionCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOG_RAW("[TEST1] error: motion queue full\r\n");
        return false;
    }

    return true;
}

static void test1_reset_motion_wait(Test1_Runtime_t *rt)
{
    rt->waiting_motion_cmd_id = 0U;
    rt->motion_cmd_sent = false;
    rt->motion_accepted = false;
    rt->motion_cmd_tick = 0U;
}

static bool test1_start_motion_step(Test1_Runtime_t *rt, Motion_CommandType_t type, float value)
{
    uint32_t seq;
    uint32_t cmd_id;

    if (!test1_motion_idle()) {
        LOG_RAW("[TEST1] error: motion busy\r\n");
        return false;
    }

    seq = (++rt->next_motion_cmd_id) & 0x0000FFFFUL;
    if (seq == 0U) {
        seq = (++rt->next_motion_cmd_id) & 0x0000FFFFUL;
    }
    cmd_id = TEST1_CMD_ID_BASE | seq;

    if (!test1_send_motion_raw(type, value, cmd_id)) {
        return false;
    }

    rt->waiting_motion_cmd_id = cmd_id;
    rt->motion_cmd_sent = true;
    rt->motion_accepted = false;
    rt->motion_cmd_tick = xTaskGetTickCount();
    return true;
}

static const char *test1_motion_type_name(Motion_CommandType_t type)
{
    switch (type) {
        case MOTION_CMD_FWD:  return "FWD";
        case MOTION_CMD_TURN: return "TURN";
        case MOTION_CMD_BACK: return "BACK";
        case MOTION_CMD_STOP: return "STOP";
        default:              return "?";
    }
}

static void test1_step_send_motion(const Test1_Runtime_t *rt, Motion_CommandType_t type, float value)
{
    log_printf_internal("[TEST1_STEP] send motion cmd_id=0x%08lX type=%s value=%+.3f state=%s\r\n",
                        (unsigned long)rt->waiting_motion_cmd_id,
                        test1_motion_type_name(type),
                        value,
                        test1_state_name(rt->state));
}

static int8_t test1_motion_step_result(Test1_Runtime_t *rt)
{
    uint32_t elapsed_ms;

    if (!rt->motion_cmd_sent) {
        return 0;
    }

    if (g_motionRtStatus.rejected_cmd_id == rt->waiting_motion_cmd_id) {
        log_printf_internal("[TEST1_STEP] motion rejected cmd_id=0x%08lX\r\n",
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
        log_printf_internal("[TEST1_STEP] motion finished cmd_id=0x%08lX result=%s\r\n",
                            (unsigned long)rt->waiting_motion_cmd_id,
                            test1_motion_result_name(g_motionRtStatus.last_result));
        return -1;
    }

    elapsed_ms = (uint32_t)((xTaskGetTickCount() - rt->motion_cmd_tick) * portTICK_PERIOD_MS);
    if (!rt->motion_accepted && elapsed_ms >= TEST1_MOTION_START_TIMEOUT_MS) {
        log_printf_internal("[TEST1_STEP] motion did not start cmd_id=0x%08lX state=%d active=0x%08lX done=0x%08lX result=%s\r\n",
                            (unsigned long)rt->waiting_motion_cmd_id,
                            (int)g_motionRtStatus.state,
                            (unsigned long)g_motionRtStatus.active_cmd_id,
                            (unsigned long)g_motionRtStatus.done_cmd_id,
                            test1_motion_result_name(g_motionRtStatus.last_result));
        return -1;
    }

    return 0;
}

static void test1_enter_error(Test1_Runtime_t *rt, const char *reason)
{
    (void)test1_send_motion_raw(MOTION_CMD_STOP, 0.0f, 0U);
    rt->state = TEST1_STATE_ERROR;
    test1_reset_motion_wait(rt);
    LOG_RAW("[TEST1] error: %s\r\n", reason);
}

static void test1_load_point(Test1_Runtime_t *rt, uint8_t index, const INS_Pose_t *pose)
{
    float dx;
    float dy;
    float dist;

    rt->point_index = index;
    rt->target_x_m = s_test1Points[index].x_m;
    rt->target_y_m = s_test1Points[index].y_m;
    dx = rt->target_x_m - pose->x_m;
    dy = rt->target_y_m - pose->y_m;
    dist = sqrtf((dx * dx) + (dy * dy));
    rt->dist_error_m = dist;
    rt->target_heading_deg = (dist <= TEST1_POS_TOL_M)
                                 ? pose->yaw_deg
                                 : (atan2f(dy, dx) * TEST1_RAD_TO_DEG);
    rt->yaw_error_deg = test1_wrap_180(rt->target_heading_deg - pose->yaw_deg);
    rt->state = (dist <= TEST1_POS_TOL_M) ? TEST1_STATE_RIGHT_TURN
                                          : TEST1_STATE_TURN_TO_TARGET;
    test1_reset_motion_wait(rt);

    LOG_RAW("[TEST1] goto %u/%u target=(%+.3f,%+.3f) heading=%+.1f dist=%.3f\r\n",
            (unsigned)(index + 1U),
            TEST1_POINT_COUNT,
            rt->target_x_m,
            rt->target_y_m,
            rt->target_heading_deg,
            rt->dist_error_m);
}

static void test1_print_help(void)
{
    LOG_RAW("[TEST1] commands:\r\n");
    LOG_RAW("  test1 help\r\n");
    LOG_RAW("  test1 start\r\n");
    LOG_RAW("  test1 status\r\n");
    LOG_RAW("  test1 stop\r\n");
}

static void test1_print_status(Test1_Runtime_t *rt)
{
    INS_Pose_t pose;

    if (test1_get_pose(&pose)) {
        (void)test1_distance_to_target(rt, &pose);
        rt->yaw_error_deg = test1_wrap_180(rt->target_heading_deg - pose.yaw_deg);
        LOG_RAW("[TEST1] state=%s point=%u/%u pose=(%+.3f,%+.3f,%+.1f) target=(%+.3f,%+.3f) dist_err=%.3f yaw_err=%+.2f wait=0x%08lX active=0x%08lX done=0x%08lX rejected=0x%08lX result=%s\r\n",
                test1_state_name(rt->state),
                (unsigned)(rt->point_index + 1U),
                TEST1_POINT_COUNT,
                pose.x_m,
                pose.y_m,
                pose.yaw_deg,
                rt->target_x_m,
                rt->target_y_m,
                rt->dist_error_m,
                rt->yaw_error_deg,
                (unsigned long)rt->waiting_motion_cmd_id,
                (unsigned long)g_motionRtStatus.active_cmd_id,
                (unsigned long)g_motionRtStatus.done_cmd_id,
                (unsigned long)g_motionRtStatus.rejected_cmd_id,
                test1_motion_result_name(g_motionRtStatus.last_result));
    } else {
        LOG_RAW("[TEST1] state=%s pose=not_ready wait=0x%08lX active=0x%08lX done=0x%08lX rejected=0x%08lX result=%s\r\n",
                test1_state_name(rt->state),
                (unsigned long)rt->waiting_motion_cmd_id,
                (unsigned long)g_motionRtStatus.active_cmd_id,
                (unsigned long)g_motionRtStatus.done_cmd_id,
                (unsigned long)g_motionRtStatus.rejected_cmd_id,
                test1_motion_result_name(g_motionRtStatus.last_result));
    }
}

static void test1_start(Test1_Runtime_t *rt)
{
    INS_Pose_t pose;

    if (!test1_get_pose(&pose)) {
        LOG_RAW("[TEST1] error: INS not ready\r\n");
        return;
    }
    if (!test1_motion_idle()) {
        LOG_RAW("[TEST1] error: motion busy, use test1 stop first\r\n");
        return;
    }

    memset(rt, 0, sizeof(*rt));
    LOG_RAW("[TEST1] start\r\n");
    test1_load_point(rt, 0U, &pose);
}

static void test1_handle_command(Test1_Runtime_t *rt, const Test1_Command_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    switch (cmd->type) {
        case TEST1_CMD_HELP:
            test1_print_help();
            break;
        case TEST1_CMD_START:
            if (rt->state != TEST1_STATE_IDLE &&
                rt->state != TEST1_STATE_DONE &&
                rt->state != TEST1_STATE_ERROR) {
                LOG_RAW("[TEST1] error: busy, use test1 stop first\r\n");
            } else {
                test1_start(rt);
            }
            break;
        case TEST1_CMD_STATUS:
            test1_print_status(rt);
            break;
        case TEST1_CMD_STOP:
            (void)test1_send_motion_raw(MOTION_CMD_STOP, 0.0f, 0U);
            rt->state = TEST1_STATE_IDLE;
            test1_reset_motion_wait(rt);
            LOG_RAW("[TEST1] stop ok\r\n");
            break;
        default:
            LOG_RAW("[TEST1] error: bad command\r\n");
            break;
    }
}

static void test1_finish_route(Test1_Runtime_t *rt)
{
    INS_Pose_t pose;

    rt->state = TEST1_STATE_DONE;
    test1_reset_motion_wait(rt);
    if (test1_get_pose(&pose)) {
        LOG_RAW("[TEST1] done pose=(%+.3f,%+.3f,%+.1f)\r\n",
                pose.x_m,
                pose.y_m,
                pose.yaw_deg);
    } else {
        LOG_RAW("[TEST1] done pose=not_ready\r\n");
    }
}

static void test1_update(Test1_Runtime_t *rt)
{
    INS_Pose_t pose;
    float dist;
    float turn_delta;
    int8_t motion_result;

    if (rt->state == TEST1_STATE_IDLE ||
        rt->state == TEST1_STATE_DONE ||
        rt->state == TEST1_STATE_ERROR) {
        return;
    }

    if (!test1_get_pose(&pose)) {
        test1_enter_error(rt, "INS lost");
        return;
    }

    switch (rt->state) {
        case TEST1_STATE_TURN_TO_TARGET:
            dist = test1_distance_to_target(rt, &pose);
            if (dist <= TEST1_POS_TOL_M) {
                log_printf_internal("[TEST1_STEP] already at point, enter RIGHT_TURN point=%u/%u\r\n",
                                    (unsigned)(rt->point_index + 1U),
                                    TEST1_POINT_COUNT);
                rt->state = TEST1_STATE_RIGHT_TURN;
                test1_reset_motion_wait(rt);
                break;
            }

            rt->target_heading_deg = atan2f(rt->target_y_m - pose.y_m,
                                            rt->target_x_m - pose.x_m) * TEST1_RAD_TO_DEG;
            turn_delta = test1_wrap_180(rt->target_heading_deg - pose.yaw_deg);
            rt->yaw_error_deg = turn_delta;

            if (!rt->motion_cmd_sent) {
                if (fabsf(turn_delta) <= TEST1_YAW_TOL_DEG) {
                    log_printf_internal("[TEST1_STEP] heading ok, enter DRIVE point=%u/%u\r\n",
                                        (unsigned)(rt->point_index + 1U),
                                        TEST1_POINT_COUNT);
                    rt->state = TEST1_STATE_DRIVE_TO_TARGET;
                    test1_reset_motion_wait(rt);
                    break;
                }

                LOG_RAW("[TEST1] turn_to_target delta=%+.1f target_heading=%+.1f\r\n",
                        turn_delta,
                        rt->target_heading_deg);
                if (!test1_start_motion_step(rt, MOTION_CMD_TURN, turn_delta)) {
                    test1_enter_error(rt, "motion command failed");
                } else {
                    test1_step_send_motion(rt, MOTION_CMD_TURN, turn_delta);
                }
            } else {
                motion_result = test1_motion_step_result(rt);
                if (motion_result > 0) {
                    log_printf_internal("[TEST1_STEP] turn_to_target done, enter DRIVE point=%u/%u\r\n",
                                        (unsigned)(rt->point_index + 1U),
                                        TEST1_POINT_COUNT);
                    rt->state = TEST1_STATE_DRIVE_TO_TARGET;
                    test1_reset_motion_wait(rt);
                } else if (motion_result < 0) {
                    test1_enter_error(rt, "motion step failed");
                }
            }
            break;

        case TEST1_STATE_DRIVE_TO_TARGET:
            dist = test1_distance_to_target(rt, &pose);
            if (dist <= TEST1_POS_TOL_M || dist <= TEST1_MIN_DRIVE_M) {
                if (rt->motion_cmd_sent) {
                    log_printf_internal("[TEST1_STEP] drive arrived, stopping motion point=%u/%u dist=%.3f\r\n",
                                        (unsigned)(rt->point_index + 1U),
                                        TEST1_POINT_COUNT,
                                        dist);
                    (void)test1_send_motion_raw(MOTION_CMD_STOP, 0.0f, 0U);
                    test1_reset_motion_wait(rt);
                    break;
                }
                log_printf_internal("[TEST1_STEP] drive stopped, enter RIGHT_TURN point=%u/%u dist=%.3f\r\n",
                                    (unsigned)(rt->point_index + 1U),
                                    TEST1_POINT_COUNT,
                                    dist);
                rt->state = TEST1_STATE_RIGHT_TURN;
                test1_reset_motion_wait(rt);
                break;
            }
            if (dist > TEST1_MAX_LEG_M) {
                test1_enter_error(rt, "remaining distance too long");
                break;
            }

            if (!rt->motion_cmd_sent) {
                LOG_RAW("[TEST1] drive dist=%.3f\r\n", dist);
                if (!test1_start_motion_step(rt, MOTION_CMD_FWD, dist)) {
                    test1_enter_error(rt, "motion command failed");
                } else {
                    test1_step_send_motion(rt, MOTION_CMD_FWD, dist);
                }
            } else {
                motion_result = test1_motion_step_result(rt);
                if (motion_result > 0) {
                    log_printf_internal("[TEST1_STEP] drive done, enter RIGHT_TURN point=%u/%u\r\n",
                                        (unsigned)(rt->point_index + 1U),
                                        TEST1_POINT_COUNT);
                    rt->state = TEST1_STATE_RIGHT_TURN;
                    test1_reset_motion_wait(rt);
                } else if (motion_result < 0) {
                    if (dist <= TEST1_POS_TOL_M &&
                        g_motionRtStatus.last_result == MOTION_RESULT_STOPPED) {
                        log_printf_internal("[TEST1_STEP] drive stopped by STOP, enter RIGHT_TURN point=%u/%u\r\n",
                                            (unsigned)(rt->point_index + 1U),
                                            TEST1_POINT_COUNT);
                        rt->state = TEST1_STATE_RIGHT_TURN;
                        test1_reset_motion_wait(rt);
                    } else {
                        test1_enter_error(rt, "motion step failed");
                    }
                }
            }
            break;

        case TEST1_STATE_RIGHT_TURN:
            if (!rt->motion_cmd_sent) {
                LOG_RAW("[TEST1] right_turn %u/%u yaw_delta=%+.1f\r\n",
                        (unsigned)(rt->point_index + 1U),
                        TEST1_POINT_COUNT,
                        TEST1_RIGHT_TURN_DEG);
                if (!test1_start_motion_step(rt, MOTION_CMD_TURN, TEST1_RIGHT_TURN_DEG)) {
                    test1_enter_error(rt, "motion command failed");
                } else {
                    log_printf_internal("[TEST1_STEP] send right turn cmd_id=0x%08lX yaw_delta=%+.1f point=%u/%u\r\n",
                                        (unsigned long)rt->waiting_motion_cmd_id,
                                        TEST1_RIGHT_TURN_DEG,
                                        (unsigned)(rt->point_index + 1U),
                                        TEST1_POINT_COUNT);
                }
            } else {
                motion_result = test1_motion_step_result(rt);
                if (motion_result > 0) {
                    uint8_t next = (uint8_t)(rt->point_index + 1U);
                    log_printf_internal("[TEST1_STEP] right turn done point=%u/%u\r\n",
                                        (unsigned)(rt->point_index + 1U),
                                        TEST1_POINT_COUNT);
                    test1_reset_motion_wait(rt);
                    if (next >= TEST1_POINT_COUNT) {
                        test1_finish_route(rt);
                    } else {
                        test1_load_point(rt, next, &pose);
                    }
                } else if (motion_result < 0) {
                    test1_enter_error(rt, "motion step failed");
                }
            }
            break;

        default:
            break;
    }
}

void test1_task(void *pvParameters)
{
    Test1_Runtime_t rt;
    Test1_Command_t cmd;

    (void)pvParameters;
    memset(&rt, 0, sizeof(rt));
    rt.state = TEST1_STATE_IDLE;

    LOG_INFO("[TEST1] task ready: test1 help\r\n");

    while (1) {
        while (g_test1CmdQueue != NULL &&
               xQueueReceive(g_test1CmdQueue, &cmd, 0) == pdTRUE) {
            test1_handle_command(&rt, &cmd);
        }

        test1_update(&rt);
        vTaskDelay(pdMS_TO_TICKS(TEST1_TASK_PERIOD_MS));
    }
}

#endif /* APP_TEST1_ENABLE */
