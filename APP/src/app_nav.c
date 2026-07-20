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
    Nav_State_t state;                  /**< 当前导航状态机状态 */
    bool square_active;                 /**< 是否正在走正方形路径 */
    uint8_t square_leg;                 /**< 当前走到正方形的第几条边(0~3) */
    float square_side_m;                /**< 正方形边长，单位 m */
    float target_x_m;                   /**< goto 目标点 X 坐标，单位 m */
    float target_y_m;                   /**< goto 目标点 Y 坐标，单位 m */
    float target_yaw_deg;               /**< 到达目标后最终期望航向角，单位 ° */
    float target_heading_deg;           /**< 当前段的目标行进方向角，单位 ° */
    float drive_distance_m;             /**< 当前直行段总距离，单位 m */
    float dist_error_m;                 /**< 当前位置到目标点的距离误差，单位 m */
    float yaw_error_deg;                /**< 当前航向角偏差，单位 ° */
    uint32_t next_motion_cmd_id;        /**< 下一条运动命令的 ID(单调递增) */
    uint32_t waiting_motion_cmd_id;     /**< 正在等待完成的运动命令 ID */
    bool motion_cmd_sent;               /**< 是否已向下层发送运动命令 */
    bool motion_accepted;               /**< 运动命令是否被下层接受 */
    TickType_t motion_cmd_tick;         /**< 运动命令发送时的系统 tick */
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

/**
  * @brief  从 INS 读取当前位姿并检查是否已就绪（IMU 有效 + 航向已归零）
  * @param  pose  输出参数，存放读取到的位姿数据
  * @retval true  读取成功且 INS 已就绪
  * @retval false 读取失败或 INS 未就绪
  * @note   该函数是获取导航系统有效位姿的唯一入口，
  *         只有 IMU 数据有效且 yaw 已置零后才返回 true。
  */
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

/**
  * @brief  更新导航距离误差和航向角偏差
  * @param  rt  导航运行时状态指针
  * @param  pose  当前 INS 位姿数据
  * @retval 无
  * @note   计算当前位置到目标点的直线距离（dist_error_m）
  *         以及当前航向与目标最终航向的偏差（yaw_error_deg）。
  */
static void nav_update_errors(Nav_Runtime_t *rt, const INS_Pose_t *pose)
{
    if (pose == NULL) {
        return;
    }
    (void)nav_distance_to_target(rt, pose);
    rt->yaw_error_deg = nav_wrap_180(rt->target_yaw_deg - pose->yaw_deg);
}
/**
  * @brief  向下层运动控制队列发送一条原始命令
  * @param  type  命令类型（FWD / TURN / BACK / STOP）
  * @param  value 命令参数（距离/角度值）
  * @param  cmd_id 命令ID
  * @retval true  发送成功
  * @retval false 队列未就绪或队列满
  */
static bool nav_send_motion_raw(Motion_CommandType_t type, float value, uint32_t cmd_id)
{
    Motion_Command_t cmd;
    cmd.type = type;
    cmd.value = value;
    cmd.value2 = 0.0f;
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

/**
  * @brief  重置运动命令等待状态，清除所有等待标志
  * @param  rt  导航运行时状态指针
  * @retval 无
  */
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

static const char *nav_motion_type_name(Motion_CommandType_t type)
{
    switch (type) {
        case MOTION_CMD_FWD:  return "FWD";
        case MOTION_CMD_TURN: return "TURN";
        case MOTION_CMD_BACK: return "BACK";
        case MOTION_CMD_STOP: return "STOP";
        default:              return "?";
    }
}

static void nav_step_send_motion(const Nav_Runtime_t *rt, Motion_CommandType_t type, float value)
{
    log_printf_internal("[NAV_STEP] send motion cmd_id=0x%08lX type=%s value=%+.3f state=%s\r\n",
                        (unsigned long)rt->waiting_motion_cmd_id,
                        nav_motion_type_name(type),
                        value,
                        nav_state_name(rt->state));
}

static int8_t nav_motion_step_result(Nav_Runtime_t *rt)
{
    uint32_t elapsed_ms;

    if (!rt->motion_cmd_sent) {
        return 0;
    }

    if (g_motionRtStatus.rejected_cmd_id == rt->waiting_motion_cmd_id) {
        log_printf_internal("[NAV_STEP] motion rejected cmd_id=0x%08lX\r\n",
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
        log_printf_internal("[NAV_STEP] motion finished cmd_id=0x%08lX result=%s\r\n",
                            (unsigned long)rt->waiting_motion_cmd_id,
                            nav_motion_result_name(g_motionRtStatus.last_result));
        return -1;
    }

    elapsed_ms = (uint32_t)((xTaskGetTickCount() - rt->motion_cmd_tick) * portTICK_PERIOD_MS);
    if (!rt->motion_accepted && elapsed_ms >= NAV_MOTION_START_TIMEOUT_MS) {
        log_printf_internal("[NAV_STEP] motion did not start cmd_id=0x%08lX state=%d active=0x%08lX done=0x%08lX result=%s\r\n",
                            (unsigned long)rt->waiting_motion_cmd_id,
                            (int)g_motionRtStatus.state,
                            (unsigned long)g_motionRtStatus.active_cmd_id,
                            (unsigned long)g_motionRtStatus.done_cmd_id,
                            nav_motion_result_name(g_motionRtStatus.last_result));
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

/**
  * @brief  准备 goto 点到点导航，计算目标航向和距离并进入对应状态
  * @param  rt  导航运行时状态指针
  * @param  x_m  目标点 X 坐标，单位 m
  * @param  y_m  目标点 Y 坐标，单位 m
  * @param  yaw_deg  到达目标后的最终期望航向角，单位 °
  * @param  internal_call  是否为内部调用（true 跳过忙检查）
  * @retval true  准备成功，已进入 TURN_TO_TARGET 或 TURN_TO_FINAL_YAW 状态
  * @retval false 准备失败（导航忙 / 运动忙 / INS 未就绪 / 距离超限）
  */
static bool nav_prepare_goto(Nav_Runtime_t *rt,
                             float x_m,
                             float y_m,
                             float yaw_deg,
                             bool internal_call)
{
    INS_Pose_t pose;
    float dx;
    float dy;
    float dist;//计算当前位置与目标位置的距离

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

    rt->square_active = false;                                          /**< 取消正方形模式 */
    rt->target_x_m = x_m;                                               /**< 保存目标点 X 坐标 */
    rt->target_y_m = y_m;                                               /**< 保存目标点 Y 坐标 */
    rt->target_yaw_deg = nav_wrap_180(yaw_deg);                         /**< 保存最终期望航向角（归一化到 ±180°） */
    rt->drive_distance_m = dist;                                        /**< 保存当前段总行驶距离 */
    rt->dist_error_m = dist;                                            /**< 初始化距离误差为总距离 */
    rt->target_heading_deg = (dist <= NAV_POS_TOL_M)                    /**< 计算目标行进方向角： */
                                 ? pose.yaw_deg                         /**<   如果已经到达目标点 → 保持当前航向 */
                                 : (atan2f(dy, dx) * NAV_RAD_TO_DEG);   /**<   否则 → atan2(dy,dx) 算朝向目标的方向角 */
    rt->yaw_error_deg = nav_wrap_180(rt->target_heading_deg - pose.yaw_deg); /**< 计算航向偏差（需要转多少度） */
    rt->state = (dist <= NAV_POS_TOL_M) ? NAV_STATE_TURN_TO_FINAL_YAW   /**< 已到达 → 直接进入最终航向调整 */
                                        : NAV_STATE_TURN_TO_TARGET;     /**< 未到达 → 先转向目标方向 */
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

/**
  * @brief  处理导航命令（仅更新运行时数据和状态机，不直接控制电机）
  * @param  rt  导航运行时状态指针
  * @param  cmd  收到的导航命令
  * @retval 无
  * @note   GOTO 和 SQUARE 命令只设置目标参数 + 切换状态，
  *         实际运动命令由 nav_task 中的 nav_update() 在后续周期中发出。
  *         STOP 命令会直接向底层发送电机停止指令。
  */
static void nav_handle_command(Nav_Runtime_t *rt, const Nav_Command_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    switch (cmd->type) {
        case NAV_CMD_HELP:
            nav_print_help();//帮助命令
            break;
        case NAV_CMD_STATUS:
            nav_print_status(rt);//打印车辆当前状态命令
            break;
        case NAV_CMD_STOP:
            (void)nav_send_motion_raw(MOTION_CMD_STOP, 0.0f, 0U);//想电机底层发送停止指令
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

/**
  * @brief  更新 goto 点到点导航状态机（转向目标 → 直行 → 调整最终航向）
  * @param  rt  导航运行时状态指针
  * @param  pose  当前 INS 位姿数据
  * @retval 无
  * @note   状态机流程：
  *         TURN_TO_TARGET    → 先转向目标方向
  *         DRIVE_TO_TARGET   → 直行到目标点附近
  *         TURN_TO_FINAL_YAW → 到达后调整最终航向 → DONE
  *         每一步通过 nav_start_motion_step 发送运动命令到 motion 层，
  *         并通过 nav_motion_step_result 等待运动完成。
  */
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
                } else {
                    nav_step_send_motion(rt, MOTION_CMD_TURN, turn_delta);
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
                } else {
                    nav_step_send_motion(rt, MOTION_CMD_FWD, dist);
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
                } else {
                    nav_step_send_motion(rt, MOTION_CMD_TURN, turn_delta);
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

/**
  * @brief  更新正方形路径行走状态机（直行 → 转弯 → 直行 → 转弯，共 4 次）
  * @param  rt  导航运行时状态指针
  * @retval 无
  * @note   正方形路径由 4 条边组成，每条边先直行再右转 90°，
  *         最后一轮转弯完成后进入 DONE 状态。
  */
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
            } else {
                nav_step_send_motion(rt, MOTION_CMD_FWD, rt->square_side_m);
            }
            return;
        }

        motion_result = nav_motion_step_result(rt);
        if (motion_result > 0) {
            log_printf_internal("[NAV_STEP] square drive done, enter square turn leg=%u/%u\r\n",
                                (unsigned)(rt->square_leg + 1U),
                                NAV_SQUARE_LEGS);
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
            } else {
                log_printf_internal("[NAV_STEP] send square turn cmd_id=0x%08lX yaw_delta=%+.1f leg=%u/%u\r\n",
                                    (unsigned long)rt->waiting_motion_cmd_id,
                                    NAV_SQUARE_TURN_DEG,
                                    (unsigned)(rt->square_leg + 1U),
                                    NAV_SQUARE_LEGS);
            }
            return;
        }

        motion_result = nav_motion_step_result(rt);
        if (motion_result > 0) {
            log_printf_internal("[NAV_STEP] square turn done leg=%u/%u\r\n",
                                (unsigned)(rt->square_leg + 1U),
                                NAV_SQUARE_LEGS);
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

/**
  * @brief  导航状态机主更新函数（每 50ms 由 nav_task 周期调用，持续运行）
  * @param  rt  导航运行时状态指针
  * @retval 无
  * @note   该函数在 nav_task 主循环中每 50ms 被调用一次。
  *         状态机持续运行，根据当前 rt->state 分发到对应的子状态机：
  *         - IDLE / DONE / ERROR：直接返回，不执行动作
  *         - SQUARE_DRIVE / SQUARE_TURN：调用 nav_update_square
  *         - 其他 goto 状态：读取 INS 位姿 → 更新误差 → 调用 nav_update_goto
  *         当 INS 丢失时自动进入 ERROR 状态并停止电机。
  */
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
