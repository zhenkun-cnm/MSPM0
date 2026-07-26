/**
 * @file    app_path.c
 * @brief   Teach-and-replay path test based on INS pose and motion commands.
 */
#include "app_path.h"
#include "app_ins.h"
#include "app_motion.h"
#include "app_tb6612.h"
#include "app_gray_line.h"
#include "app_drive_mode.h"
#include "dev_flash.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* ===== 任务周期 ===== */
#define PATH_TASK_PERIOD_MS          50U    /* 任务调度周期 (ms) */

/* ===== 路径录制 ===== */
#define PATH_MAX_POINTS              292U   /* 日志 DMA 静态 RAM 后仍净释放至少 2 KiB */
#define PATH_RECORD_MIN_DIST_M       0.05f  /* 触发新录点的最小位置变化 (m) */
#define PATH_RECORD_MIN_YAW_DEG      5.0f   /* 触发新录点的最小朝向变化 (deg) */

/* ===== 回放运动命令 ===== */
#define PATH_POS_TOL_M               0.05f  /* 位置容差 (m) */
#define PATH_YAW_TOL_DEG             3.0f   /* 朝向容差 (deg) */
#define PATH_MIN_DRIVE_M             0.03f  /* 最小直驱距离 (m)，小于此值视为原地转向 */
#define PATH_MAX_LEG_M               1.00f  /* 单段最大驱动距离 (m)，超过报错 */
#define PATH_REPLAY_ARC_MIN_YAW_DEG  5.0f   /* 弧线拟合的最小朝向变化 (deg) */
#define PATH_REPLAY_MIN_ARC_RADIUS_M 0.15f  /* 弧线拟合的最小半径 (m) */
#define PATH_REPLAY_MAX_ARC_RADIUS_M 2.00f  /* 弧线拟合的最大半径 (m) */
#define PATH_MOTION_START_TIMEOUT_MS 500U   /* 等待运动命令被接受的距离(ms) */
#define PATH_CMD_ID_BASE             0x72000000UL  /* 运动命令 ID 基址 (防止与其他模块冲突) */

/* ===== 数学常量 ===== */
#define PATH_RAD_TO_DEG              57.295779513082320876f  /* 弧度转角度 (180/pi) */
#define PATH_DEG_TO_RAD              0.017453292519943295f   /* 角度转弧度 (pi/180) */

/* ===== 连续轨迹跟踪 (REPLAY_TRACK 模式) ===== */
#define PATH_TRACK_LOOKAHEAD_M       0.10f  /* 前视距离 (m)：寻找此距离外的路点作为目标 */
#define PATH_TRACK_COMPLETE_DIST_M   0.06f  /* 到达终点判定阈值 (m) */
#define PATH_TRACK_DONE_INDEX_BACKOFF 3U    /* 终点附近防提前完成：距终点 N 个索引内才允许 done */
#define PATH_TRACK_BASE_PWM          16.0f  /* 基速 PWM */
#define PATH_TRACK_YAW_KP            0.35f  /* 航向误差比例增益 */
#define PATH_TRACK_TRIM_MAX          8.0f   /* 差速修正最大限幅 (PWM) */
#define PATH_TRACK_PWM_MIN           6.0f   /* 最小允许 PWM */
#define PATH_TRACK_PWM_MAX           30.0f  /* 最大允许 PWM */
#define PATH_TRACK_PWM_SLEW          2.0f   /* PWM 缓变率：每 50ms 最大变化量 */
#define PATH_TRACK_LOG_PERIOD_MS     500U   /* 轨迹跟踪日志输出周期 (ms) */
#define PATH_FUSION_GRAY_MAX_AGE_MS  30U
#define PATH_FUSION_MODE_PATH_ONLY   0U
#define PATH_FUSION_MODE_BLEND       1U
#define PATH_FUSION_MODE_STALE       2U
#define PATH_FUSION_MODE_REVERSE     3U

/* ===== Flash 存储 ===== */
#define PATH_FLASH_START_ADDR        0x000F0000UL  /* Flash 存储起始地址 */
#define PATH_FLASH_SECTOR_SIZE       4096UL        /* Flash 扇区大小 (字节) */
#define PATH_FLASH_SECTOR_COUNT      2U            /* 占用扇区数 */
#define PATH_FLASH_BYTES             (PATH_FLASH_SECTOR_SIZE * PATH_FLASH_SECTOR_COUNT)  /* 总存储空间 (字节) */
#define PATH_FLASH_MAGIC             0x48545052UL  /* "RPTH" little-endian 魔数 */
#define PATH_FLASH_VERSION           1U            /* 存储格式版本号 */

/* 全局路径命令队列，由 UART 命令处理模块发送命令到此队列 */
QueueHandle_t g_pathCmdQueue = NULL;
PathFusion_Config_t g_pathFusionConfig = {
    0.70f,
    40.0f,
    8.0f
};

/**
 * @brief 路径点数据结构
 *        存储单个路点的位置 (x, y) 和朝向角
 */
typedef struct {
    float x_m;       /* X 坐标 (m) */
    float y_m;       /* Y 坐标 (m) */
    float yaw_deg;   /* 朝向角 (deg) */
} Path_Point_t;

/**
 * @brief 路径模块运行时状态
 *        存储当前状态机的全部运行时变量
 */
typedef struct {
    Path_State_t state;         /* 当前状态机状态 */
    uint16_t replay_index;      /* 回放当前指向的路点索引 */

    /* ---- 运动命令等待/完成状态 ---- */
    uint32_t next_motion_cmd_id;       /* 下一个运动命令 ID 种子 */
    uint32_t waiting_motion_cmd_id;    /* 当前等待完成的运动命令 ID */
    bool motion_cmd_sent;              /* 是否已发送运动命令 */
    bool motion_accepted;              /* 运动命令是否被 motion 模块接受 */
    TickType_t motion_cmd_tick;        /* 运动命令发送时的系统 tick */

    /* ---- 连续轨迹跟踪运行数据 ---- */
    uint16_t track_target_index;       /* 当前跟踪目标点索引 */
    float track_last_dist_m;           /* 到目标点的距离 (m) */
    float track_last_final_dist_m;     /* 到终点的距离 (m) */
    float track_last_heading_error_deg;/* 航向偏差 (deg)，正=偏左 */
    int16_t track_last_left_pwm;       /* 左轮当前 PWM */
    int16_t track_last_right_pwm;      /* 右轮当前 PWM */
    TickType_t track_last_log_tick;    /* 上次输出轨迹日志的时刻 */
    TickType_t track_near_final_log_tick; /* 上次输出"near final"日志的时刻 */
    bool track_motor_started;          /* 电机是否已启动 */
    bool track_reverse;
    bool fusion_active;
    float fusion_gray_trim_pwm;
    float fusion_path_trim_pwm;
    float fusion_final_trim_pwm;
    uint8_t fusion_mode;
} Path_Runtime_t;

/**
 * @brief Flash 存储文件头结构 (packed 对齐)
 *        存储在 Flash 的起始地址，用于校验和版本管理
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;         /* 魔数 "RPTH"，用于快速识别有效数据 */
    uint16_t version;       /* 存储格式版本号 */
    uint16_t point_size;    /* 单个路径点结构体大小 (用于兼容性检查) */
    uint16_t max_points;    /* 最大支持的点数 */
    uint16_t count;         /* 实际存储的路径点数 */
    uint32_t data_bytes;    /* 点数据总字节数 (count * sizeof(Path_Point_t)) */
    uint32_t checksum;      /* FNV-1a 校验和，校验点数据区域 */
} Path_FlashHeader_t;

static Path_Point_t s_pathPoints[PATH_MAX_POINTS];
static uint16_t s_pathCount = 0U;
static bool s_pathOperationOk = false;
static Path_RuntimeStatus_t s_pathStatus = {
    PATH_STATE_IDLE, PATH_CMD_STATUS, PATH_RESULT_NONE, PATH_ERROR_NONE,
    0U, 0U, 0U, 0U
};

static void path_status_publish(const Path_Runtime_t *rt,
                                Path_CommandType_t command,
                                Path_Result_t result,
                                Path_Error_t error,
                                bool newResult)
{
    taskENTER_CRITICAL();
    s_pathStatus.state = rt->state;
    s_pathStatus.point_count = s_pathCount;
    s_pathStatus.replay_index = rt->replay_index;
    s_pathStatus.replay_total = s_pathCount;
    s_pathStatus.fusion_active = rt->fusion_active;
    s_pathStatus.fusion_gray_ready = rt->fusion_active &&
                                        (rt->fusion_mode != PATH_FUSION_MODE_STALE);
    s_pathStatus.fusion_gray_trim_pwm = rt->fusion_gray_trim_pwm;
    s_pathStatus.fusion_path_trim_pwm = rt->fusion_path_trim_pwm;
    s_pathStatus.fusion_final_trim_pwm = rt->fusion_final_trim_pwm;
    s_pathStatus.fusion_mode = rt->fusion_mode;
    if (newResult) {
        s_pathStatus.last_command = command;
        s_pathStatus.result = result;
        s_pathStatus.error = error;
        s_pathStatus.sequence++;
    }
    taskEXIT_CRITICAL();
}

bool Path_Status_Read(Path_RuntimeStatus_t *out)
{
    if (out == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    *out = s_pathStatus;
    taskEXIT_CRITICAL();
    return true;
}

static float path_wrap_180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static const char *path_state_name(Path_State_t state)
{
    switch (state) {
        case PATH_STATE_IDLE:         return "IDLE";
        case PATH_STATE_RECORDING:    return "RECORDING";
        case PATH_STATE_REPLAY_TURN:  return "REPLAY_TURN";
        case PATH_STATE_REPLAY_DRIVE: return "REPLAY_DRIVE";
        case PATH_STATE_REPLAY_SEGMENT: return "REPLAY_SEG";
        case PATH_STATE_REPLAY_TRACK: return "REPLAY_TRACK";
        case PATH_STATE_DONE:         return "DONE";
        case PATH_STATE_ERROR:        return "ERROR";
        default:                      return "?";
    }
}

static const char *path_motion_result_name(Motion_Result_t result)
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

static bool path_pose_ready(const INS_Pose_t *pose)
{
    uint32_t required = INS_FLAG_IMU_VALID | INS_FLAG_YAW_ZERO_READY;
    return pose != NULL && ((pose->flags & required) == required);
}

static bool path_get_pose(INS_Pose_t *pose)
{
    return (pose != NULL) && INS_Pose_Read(pose) && path_pose_ready(pose);
}

static float path_distance_to_point(const INS_Pose_t *pose, const Path_Point_t *point)
{
    float dx = point->x_m - pose->x_m;
    float dy = point->y_m - pose->y_m;
    return sqrtf((dx * dx) + (dy * dy));
}

/**
 * @brief 计算路径段的移动方向角与车身朝向误差
 *
 * 大白话解释：
 * 假如你从 prev 点走到 target 点，你在地图上走的那个方向（比如往东偏北 30°），
 * 就叫 move_heading_deg。
 * 而你当时面朝的方向是 prev->yaw_deg（比如面朝正北 90°）。
 *
 * 这两个方向的差值就是 body_error_deg：
 *   body_error_deg = move_heading_deg - prev->yaw_deg
 *
 * 举个例子：
 *   - 你面朝正北(90°)，要走的方向也是北(90°) → body_error_deg = 0°，直走。
 *   - 你面朝正北(90°)，要走的方向是南(-90°) → body_error_deg = -180°→180°，倒车。
 *   - 你面朝正北(90°)，要走的方向是东(0°)   → body_error_deg = -90°，右转横着走。
 *
 * 结论：
 *   - |body_error_deg| <= 90° → 前进就能到
 *   - |body_error_deg| > 90°  → 需要后退
 *
 * @param prev            路径段起点（上一路点）
 * @param target          路径段终点（当前路点）
 * @param move_heading_deg [out] 从 prev 指向 target 的方向角 (deg)，0°=东, 90°=北
 * @param body_error_deg   [out] 车身朝向与移动方向的夹角 (deg)，正=偏左，已归一化到 (-180, 180]
 * @return true  计算成功
 * @return false 参数无效或两点重合（距离 < 0.001 mm）
 */
static bool path_segment_get_heading(const Path_Point_t *prev,
                                     const Path_Point_t *target,
                                     float *move_heading_deg,
                                     float *body_error_deg)
{
    float dx;
    float dy;

    if (prev == NULL || target == NULL ||
        move_heading_deg == NULL || body_error_deg == NULL) {
        return false;
    }

    dx = target->x_m - prev->x_m;
    dy = target->y_m - prev->y_m;
    if (((dx * dx) + (dy * dy)) < 0.000001f) {
        return false;
    }

    *move_heading_deg = atan2f(dy, dx) * PATH_RAD_TO_DEG;
    *body_error_deg = path_wrap_180(*move_heading_deg - prev->yaw_deg);
    return true;
}

/**
 * @brief 判断当前路径段是否需要倒退行驶（Reverse Direction）
 *
 * 计算从 prev 到 target 的空间位移方向与录制时车身朝向（prev->yaw_deg）的夹角
 * body_error_deg。
 * 若 |body_error_deg| > 90°，说明车辆需要倒车才能从 prev 移动到 target，
 * 此时返回 true（后退段）；否则返回 false（前进段）。
 *
 * @param prev   路径段起点（上一路点指针）
 * @param target 路径段终点（当前路点指针）
 * @return true  需要后退（车身朝向与位移方向夹角 > 90°）
 * @return false 可以前进（夹角 ≤ 90°）或输入无效
 */
static bool path_segment_is_reverse(const Path_Point_t *prev,
                                    const Path_Point_t *target)
{
    float move_heading_deg;
    float body_error_deg;

    if (!path_segment_get_heading(prev, target,
                                  &move_heading_deg, &body_error_deg)) {
        return false;
    }
    return fabsf(body_error_deg) > 90.0f;
}

static float path_clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static int16_t path_clamp_pwm(float value)
{
    return (int16_t)path_clamp_f32(value, 0.0f, 100.0f);
}

static float path_slew_f32(float current, float target, float max_step)
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

static bool path_motion_idle(void)
{
    return g_motionRtStatus.state == MOTION_RT_IDLE;
}

static bool path_send_motor_cmd(MotorCmdType type, int16_t val, TickType_t wait_ticks)
{
    MotorCmd cmd;

    if (g_motorCmdQueue == NULL) {
        LOGE(LOG_MOD_PATH, "error: motor queue not ready\r\n");
        return false;
    }

    cmd.type = type;
    cmd.val = val;
    return xQueueSend(g_motorCmdQueue, &cmd, wait_ticks) == pdTRUE;
}

static void path_track_stop_motors(void)
{
    if (!DriveMode_Owns(DRIVE_MODE_PATH) &&
        !DriveMode_Owns(DRIVE_MODE_PATH_FUSION)) {
        return;
    }
    (void)path_send_motor_cmd(MOTOR_CMD_LEFT_SPEED, 0, pdMS_TO_TICKS(20));
    (void)path_send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, 0, pdMS_TO_TICKS(20));
    (void)path_send_motor_cmd(MOTOR_CMD_ONOFF, 0, pdMS_TO_TICKS(20));
}

static bool path_track_apply_pwm(int16_t left_pwm, int16_t right_pwm)
{
    if (!DriveMode_Owns(DRIVE_MODE_PATH) &&
        !DriveMode_Owns(DRIVE_MODE_PATH_FUSION)) {
        return false;
    }
    if (!path_send_motor_cmd(MOTOR_CMD_LEFT_SPEED, left_pwm, pdMS_TO_TICKS(2))) {
        return false;
    }
    if (!path_send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, right_pwm, pdMS_TO_TICKS(2))) {
        return false;
    }
    return true;
}

static bool path_track_set_direction(bool reverse)
{
    TB6612_Dir dir = reverse ? TB6612_DIR_REVERSE : TB6612_DIR_FORWARD;
    return path_send_motor_cmd(MOTOR_CMD_DIR, (int16_t)dir, pdMS_TO_TICKS(20));
}

static bool path_track_start_motors(bool reverse, int16_t left_pwm, int16_t right_pwm)
{
    if (!DriveMode_Owns(DRIVE_MODE_PATH) &&
        !DriveMode_Owns(DRIVE_MODE_PATH_FUSION)) {
        return false;
    }
    if (!path_track_set_direction(reverse)) {
        return false;
    }
    if (!path_track_apply_pwm(left_pwm, right_pwm)) {
        return false;
    }
    if (!path_send_motor_cmd(MOTOR_CMD_ONOFF, 1, pdMS_TO_TICKS(20))) {
        return false;
    }
    return true;
}

static bool path_track_switch_direction(Path_Runtime_t *rt, bool reverse)
{
    if (rt == NULL || !rt->track_motor_started ||
        rt->track_reverse == reverse) {
        return true;
    }

    if (!path_track_apply_pwm(0, 0)) {
        return false;
    }
    rt->track_last_left_pwm = 0;
    rt->track_last_right_pwm = 0;

    if (!path_track_set_direction(reverse)) {
        return false;
    }

    rt->track_reverse = reverse;
    return true;
}

static bool path_send_motion_raw2(Motion_CommandType_t type, float value, float value2, uint32_t cmd_id)
{
    Motion_Command_t cmd;
    cmd.type = type;
    cmd.value = value;
    cmd.value2 = value2;
    cmd.cmd_id = cmd_id;

    if (g_motionCmdQueue == NULL) {
        LOGE(LOG_MOD_PATH, "error: motion queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_motionCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOGE(LOG_MOD_PATH, "error: motion queue full\r\n");
        return false;
    }

    return true;
}

static bool path_send_motion_raw(Motion_CommandType_t type, float value, uint32_t cmd_id)
{
    return path_send_motion_raw2(type, value, 0.0f, cmd_id);
}

static void path_reset_motion_wait(Path_Runtime_t *rt)
{
    rt->waiting_motion_cmd_id = 0U;
    rt->motion_cmd_sent = false;
    rt->motion_accepted = false;
    rt->motion_cmd_tick = 0U;
}

static bool path_start_motion_step2(Path_Runtime_t *rt, Motion_CommandType_t type, float value, float value2)
{
    uint32_t seq;
    uint32_t cmd_id;

    if (!path_motion_idle()) {
        LOGE(LOG_MOD_PATH, "error: motion busy\r\n");
        return false;
    }

    seq = (++rt->next_motion_cmd_id) & 0x0000FFFFUL;
    if (seq == 0U) {
        seq = (++rt->next_motion_cmd_id) & 0x0000FFFFUL;
    }
    cmd_id = PATH_CMD_ID_BASE | seq;

    if (!path_send_motion_raw2(type, value, value2, cmd_id)) {
        return false;
    }

    rt->waiting_motion_cmd_id = cmd_id;
    rt->motion_cmd_sent = true;
    rt->motion_accepted = false;
    rt->motion_cmd_tick = xTaskGetTickCount();
    LOGD(LOG_MOD_PATH, "send motion cmd_id=0x%08lX type=%d value=%+.3f value2=%+.3f point=%u/%u\r\n",
                        (unsigned long)cmd_id,
                        (int)type,
                        value,
                        value2,
                        (unsigned)(rt->replay_index + 1U),
                        (unsigned)s_pathCount);
    return true;
}

static bool path_start_motion_step(Path_Runtime_t *rt, Motion_CommandType_t type, float value)
{
    return path_start_motion_step2(rt, type, value, 0.0f);
}

static int8_t path_motion_step_result(Path_Runtime_t *rt)
{
    uint32_t elapsed_ms;

    if (!rt->motion_cmd_sent) {
        return 0;
    }

    if (g_motionRtStatus.rejected_cmd_id == rt->waiting_motion_cmd_id) {
        LOGD(LOG_MOD_PATH, "motion rejected cmd_id=0x%08lX\r\n",
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
        LOGD(LOG_MOD_PATH, "motion finished cmd_id=0x%08lX result=%s\r\n",
                            (unsigned long)rt->waiting_motion_cmd_id,
                            path_motion_result_name(g_motionRtStatus.last_result));
        return -1;
    }

    elapsed_ms = (uint32_t)((xTaskGetTickCount() - rt->motion_cmd_tick) * portTICK_PERIOD_MS);
    if (!rt->motion_accepted && elapsed_ms >= PATH_MOTION_START_TIMEOUT_MS) {
        LOGD(LOG_MOD_PATH, "motion did not start cmd_id=0x%08lX state=%d active=0x%08lX done=0x%08lX result=%s\r\n",
                            (unsigned long)rt->waiting_motion_cmd_id,
                            (int)g_motionRtStatus.state,
                            (unsigned long)g_motionRtStatus.active_cmd_id,
                            (unsigned long)g_motionRtStatus.done_cmd_id,
                            path_motion_result_name(g_motionRtStatus.last_result));
        return -1;
    }

    return 0;
}

static bool path_add_point(const INS_Pose_t *pose)
{
    if (pose == NULL || s_pathCount >= PATH_MAX_POINTS) {
        return false;
    }

    s_pathPoints[s_pathCount].x_m = pose->x_m;
    s_pathPoints[s_pathCount].y_m = pose->y_m;
    s_pathPoints[s_pathCount].yaw_deg = pose->yaw_deg;
    s_pathCount++;
    return true;
}

static bool path_flash_probe(DevFlash **out_flash)
{
    DevFlash *flash;
    DevFlash_JEDECID_t id;

    if (out_flash == NULL) {
        return false;
    }

    flash = GetFlash();
    if (flash == NULL) {
        LOGE(LOG_MOD_PATH, "error: flash handle NULL\r\n");
        return false;
    }

    flash->init(flash);
    if (!flash->readJEDECID(flash, &id) ||
        id.manufacturer != 0xEF ||
        id.memoryType != 0x40) {
        LOGE(LOG_MOD_PATH, "error: JEDEC invalid\r\n");
        return false;
    }

    *out_flash = flash;
    return true;
}

static uint32_t path_checksum_bytes(const uint8_t *data, uint32_t len)
{
    uint32_t hash = 2166136261UL;
    uint32_t i;

    for (i = 0U; i < len; i++) {
        hash ^= (uint32_t)data[i];
        hash *= 16777619UL;
    }

    return hash;
}

static bool path_flash_program_bytes(DevFlash *flash,
                                     uint32_t addr,
                                     const uint8_t *data,
                                     uint32_t len)
{
    uint32_t offset = 0U;
    uint32_t page_left;
    uint16_t chunk;

    if (flash == NULL || data == NULL) {
        return false;
    }

    while (offset < len) {
        page_left = 256UL - ((addr + offset) & 0xFFUL);
        chunk = (uint16_t)(len - offset);
        if ((uint32_t)chunk > page_left) {
            chunk = (uint16_t)page_left;
        }
        if (chunk > 256U) {
            chunk = 256U;
        }

        if (!flash->pageProgram(flash, addr + offset, data + offset, chunk)) {
            return false;
        }
        offset += (uint32_t)chunk;
    }

    return true;
}

static bool path_is_active(const Path_Runtime_t *rt)
{
    return rt != NULL &&
           (rt->state == PATH_STATE_RECORDING ||
            rt->state == PATH_STATE_REPLAY_TURN ||
            rt->state == PATH_STATE_REPLAY_DRIVE ||
            rt->state == PATH_STATE_REPLAY_SEGMENT ||
            rt->state == PATH_STATE_REPLAY_TRACK);
}

static void path_save_to_flash(const Path_Runtime_t *rt)
{
    DevFlash *flash;
    Path_FlashHeader_t header;
    uint32_t data_bytes;
    uint32_t total_bytes;
    uint32_t i;

    s_pathOperationOk = false;
    if (path_is_active(rt)) {
        LOGE(LOG_MOD_PATH, "error: stop record/replay before save\r\n");
        return;
    }
    if (s_pathCount < 2U) {
        LOGE(LOG_MOD_PATH, "error: need at least 2 points\r\n");
        return;
    }

    data_bytes = (uint32_t)s_pathCount * (uint32_t)sizeof(Path_Point_t);
    total_bytes = (uint32_t)sizeof(header) + data_bytes;
    if (total_bytes > PATH_FLASH_BYTES) {
        LOGE(LOG_MOD_PATH, "error: path too large bytes=%lu\r\n",
                (unsigned long)total_bytes);
        return;
    }

    if (!path_flash_probe(&flash)) {
        return;
    }

    memset(&header, 0, sizeof(header));
    header.magic = PATH_FLASH_MAGIC;
    header.version = PATH_FLASH_VERSION;
    header.point_size = (uint16_t)sizeof(Path_Point_t);
    header.max_points = PATH_MAX_POINTS;
    header.count = s_pathCount;
    header.data_bytes = data_bytes;
    header.checksum = path_checksum_bytes((const uint8_t *)s_pathPoints, data_bytes);

    LOGI(LOG_MOD_PATH, "save start addr=0x%08lX count=%u bytes=%lu\r\n",
            (unsigned long)PATH_FLASH_START_ADDR,
            (unsigned)s_pathCount,
            (unsigned long)total_bytes);

    for (i = 0U; i < PATH_FLASH_SECTOR_COUNT; i++) {
        if (!flash->sectorErase(flash,
                                PATH_FLASH_START_ADDR + (i * PATH_FLASH_SECTOR_SIZE))) {
            LOGE(LOG_MOD_PATH, "error: erase failed sector=%lu\r\n",
                    (unsigned long)i);
            return;
        }
    }

    if (!path_flash_program_bytes(flash,
                                  PATH_FLASH_START_ADDR,
                                  (const uint8_t *)&header,
                                  (uint32_t)sizeof(header)) ||
        !path_flash_program_bytes(flash,
                                  PATH_FLASH_START_ADDR + (uint32_t)sizeof(header),
                                  (const uint8_t *)s_pathPoints,
                                  data_bytes)) {
        LOGE(LOG_MOD_PATH, "error: write failed\r\n");
        return;
    }

    LOGI(LOG_MOD_PATH, "save ok count=%u checksum=0x%08lX\r\n",
            (unsigned)s_pathCount,
            (unsigned long)header.checksum);
    s_pathOperationOk = true;
}

static void path_load_from_flash(Path_Runtime_t *rt)
{
    DevFlash *flash;
    Path_FlashHeader_t header;
    uint32_t checksum;

    s_pathOperationOk = false;
    if (path_is_active(rt)) {
        LOGE(LOG_MOD_PATH, "error: stop record/replay before load\r\n");
        return;
    }

    if (!path_flash_probe(&flash)) {
        return;
    }

    if (!flash->read(flash,
                     PATH_FLASH_START_ADDR,
                     (uint8_t *)&header,
                     (uint16_t)sizeof(header))) {
        LOGE(LOG_MOD_PATH, "error: header read failed\r\n");
        return;
    }

    if (header.magic != PATH_FLASH_MAGIC ||
        header.version != PATH_FLASH_VERSION ||
        header.point_size != sizeof(Path_Point_t) ||
        header.count > PATH_MAX_POINTS ||
        header.count < 2U ||
        header.data_bytes != ((uint32_t)header.count * (uint32_t)sizeof(Path_Point_t))) {
        LOGE(LOG_MOD_PATH, "error: no valid path in flash\r\n");
        return;
    }

    if (((uint32_t)sizeof(header) + header.data_bytes) > PATH_FLASH_BYTES) {
        LOGE(LOG_MOD_PATH, "error: stored path too large\r\n");
        return;
    }

    memset(s_pathPoints, 0, sizeof(s_pathPoints));
    if (!flash->read(flash,
                     PATH_FLASH_START_ADDR + (uint32_t)sizeof(header),
                     (uint8_t *)s_pathPoints,
                     (uint16_t)header.data_bytes)) {
        s_pathCount = 0U;
        LOGE(LOG_MOD_PATH, "error: data read failed\r\n");
        return;
    }

    checksum = path_checksum_bytes((const uint8_t *)s_pathPoints, header.data_bytes);
    if (checksum != header.checksum) {
        memset(s_pathPoints, 0, sizeof(s_pathPoints));
        s_pathCount = 0U;
        LOGE(LOG_MOD_PATH, "error: checksum mismatch got=0x%08lX expect=0x%08lX\r\n",
                (unsigned long)checksum,
                (unsigned long)header.checksum);
        return;
    }

    s_pathCount = header.count;
    rt->state = PATH_STATE_IDLE;
    rt->replay_index = 0U;
    rt->track_target_index = 0U;
    path_reset_motion_wait(rt);

    LOGI(LOG_MOD_PATH, "load ok count=%u checksum=0x%08lX\r\n",
            (unsigned)s_pathCount,
            (unsigned long)checksum);
    s_pathOperationOk = true;
}

static void path_print_help(void)
{
    LOGI_RELIABLE(LOG_MOD_PATH, "commands:\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path help\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path status\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path clear\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path record start\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path record stop\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path print\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path replay\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path fusion start|stop|status\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path stop\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path save\r\n");
    LOGI_RELIABLE(LOG_MOD_PATH, "  path load\r\n");
}

static void path_print_status(const Path_Runtime_t *rt)
{
    LOGI_RELIABLE(LOG_MOD_PATH, "state=%s count=%u replay=%u/%u target=%u dir=%c dist=%.3f final=%.3f herr=%+.1f pwm L=%d R=%d wait=0x%08lX active=0x%08lX done=0x%08lX rejected=0x%08lX result=%s\r\n",
            path_state_name(rt->state),
            (unsigned)s_pathCount,
            (unsigned)(rt->replay_index + 1U),
            (unsigned)s_pathCount,
            (unsigned)rt->track_target_index,
            rt->track_reverse ? 'B' : 'F',
            rt->track_last_dist_m,
            rt->track_last_final_dist_m,
            rt->track_last_heading_error_deg,
            (int)rt->track_last_left_pwm,
            (int)rt->track_last_right_pwm,
            (unsigned long)rt->waiting_motion_cmd_id,
            (unsigned long)g_motionRtStatus.active_cmd_id,
            (unsigned long)g_motionRtStatus.done_cmd_id,
            (unsigned long)g_motionRtStatus.rejected_cmd_id,
            path_motion_result_name(g_motionRtStatus.last_result));
    LOGI_RELIABLE(LOG_MOD_PATH, "fusion=%u mode=%u gray=%+.2f path=%+.2f final=%+.2f cfg=%.2f/%.1f/%.1f\r\n",
            rt->fusion_active ? 1U : 0U,
            (unsigned)rt->fusion_mode,
            (double)rt->fusion_gray_trim_pwm,
            (double)rt->fusion_path_trim_pwm,
            (double)rt->fusion_final_trim_pwm,
            (double)g_pathFusionConfig.gray_weight,
            (double)g_pathFusionConfig.gray_turn_to_pwm,
            (double)g_pathFusionConfig.gray_trim_max_pwm);
}

static void path_print_points(void)
{
    uint16_t i;

    LOGI(LOG_MOD_PATH, "points count=%u\r\n", (unsigned)s_pathCount);
    for (i = 0U; i < s_pathCount; i++) {
        LOGI(LOG_MOD_PATH, "#%02u X=%+.3f Y=%+.3f YAW=%+.1f\r\n",
                (unsigned)i,
                s_pathPoints[i].x_m,
                s_pathPoints[i].y_m,
                s_pathPoints[i].yaw_deg);
    }
}

static void path_enter_error(Path_Runtime_t *rt, const char *reason)
{
    (void)path_send_motion_raw(MOTION_CMD_STOP, 0.0f, 0U);
    path_track_stop_motors();
    rt->state = PATH_STATE_ERROR;
    path_reset_motion_wait(rt);
    rt->track_motor_started = false;
    rt->track_last_left_pwm = 0;
    rt->track_last_right_pwm = 0;
    rt->track_last_final_dist_m = 0.0f;
    LOGE(LOG_MOD_PATH, "error: %s\r\n", reason);
    path_status_publish(rt, PATH_CMD_REPLAY, PATH_RESULT_ERROR,
                        PATH_ERROR_MOTION, true);
}

static void path_start_record(Path_Runtime_t *rt)
{
    INS_Pose_t pose;

    s_pathOperationOk = false;
    if (!path_get_pose(&pose)) {
        LOGE(LOG_MOD_PATH, "error: INS not ready\r\n");
        return;
    }

    memset(s_pathPoints, 0, sizeof(s_pathPoints));
    s_pathCount = 0U;
    (void)path_add_point(&pose);//记录起点
    rt->state = PATH_STATE_RECORDING;
    rt->replay_index = 0U;
    path_reset_motion_wait(rt);
    LOGI(LOG_MOD_PATH, "record start\r\n");
    s_pathOperationOk = true;
}

static void path_stop_record(Path_Runtime_t *rt)
{
    s_pathOperationOk = false;
    if (rt->state == PATH_STATE_RECORDING) {
        rt->state = PATH_STATE_IDLE;
        LOGI(LOG_MOD_PATH, "record stop count=%u\r\n", (unsigned)s_pathCount);
        s_pathOperationOk = true;
    } else {
        LOGI(LOG_MOD_PATH, "record not active\r\n");
    }
}

static void path_start_replay(Path_Runtime_t *rt)
{
    s_pathOperationOk = false;
    if (s_pathCount < 2U) {
        LOGE(LOG_MOD_PATH, "error: need at least 2 points\r\n");
        return;
    }
    if (!path_motion_idle()) {
        LOGE(LOG_MOD_PATH, "error: motion busy\r\n");
        return;
    }

    rt->state = PATH_STATE_REPLAY_TRACK;
    rt->fusion_active = false;
    rt->fusion_mode = PATH_FUSION_MODE_PATH_ONLY;
    rt->fusion_gray_trim_pwm = 0.0f;
    rt->fusion_path_trim_pwm = 0.0f;
    rt->fusion_final_trim_pwm = 0.0f;
    rt->replay_index = 1U;
    rt->track_target_index = 1U;
    rt->track_last_dist_m = 0.0f;
    rt->track_last_final_dist_m = 0.0f;
    rt->track_last_heading_error_deg = 0.0f;
    rt->track_last_left_pwm = 0;
    rt->track_last_right_pwm = 0;
    rt->track_last_log_tick = 0U;
    rt->track_near_final_log_tick = 0U;
    rt->track_motor_started = false;
    rt->track_reverse = false;
    path_reset_motion_wait(rt);
    LOGI(LOG_MOD_PATH, "replay start count=%u mode=continuous_track\r\n", (unsigned)s_pathCount);
    s_pathOperationOk = true;
}

static void path_start_fusion(Path_Runtime_t *rt)
{
    s_pathOperationOk = false;
    if (s_pathCount < 2U) {
        LOGE(LOG_MOD_PATH, "error: need at least 2 points\r\n");
        return;
    }
    if (!path_motion_idle()) {
        LOGE(LOG_MOD_PATH, "error: motion busy\r\n");
        return;
    }

    /* Start tracking immediately. Until a fresh gray sample arrives,
     * path_update_track() deliberately keeps using the INS path trim only. */
    rt->state = PATH_STATE_REPLAY_TRACK;
    rt->replay_index = 1U;
    rt->track_target_index = 1U;
    rt->track_last_dist_m = 0.0f;
    rt->track_last_final_dist_m = 0.0f;
    rt->track_last_heading_error_deg = 0.0f;
    rt->track_last_left_pwm = 0;
    rt->track_last_right_pwm = 0;
    rt->track_last_log_tick = 0U;
    rt->track_near_final_log_tick = 0U;
    rt->track_motor_started = false;
    rt->track_reverse = false;
    rt->fusion_active = true;
    rt->fusion_gray_trim_pwm = 0.0f;
    rt->fusion_path_trim_pwm = 0.0f;
    rt->fusion_final_trim_pwm = 0.0f;
    rt->fusion_mode = PATH_FUSION_MODE_STALE;
    path_reset_motion_wait(rt);
    LOGI(LOG_MOD_PATH, "fusion start count=%u path-only until gray fresh\r\n",
         (unsigned)s_pathCount);
    s_pathOperationOk = true;
}

static void path_stop(Path_Runtime_t *rt)
{
    (void)path_send_motion_raw(MOTION_CMD_STOP, 0.0f, 0U);
    path_track_stop_motors();
    rt->state = PATH_STATE_IDLE;
    path_reset_motion_wait(rt);
    rt->track_motor_started = false;
    rt->track_reverse = false;
    rt->fusion_active = false;
    rt->fusion_mode = PATH_FUSION_MODE_PATH_ONLY;
    rt->track_last_left_pwm = 0;
    rt->track_last_right_pwm = 0;
    rt->track_last_final_dist_m = 0.0f;
    LOGI(LOG_MOD_PATH, "stop ok\r\n");
    s_pathOperationOk = true;
}

static void path_handle_command(Path_Runtime_t *rt, const Path_Command_t *cmd)
{
    if (cmd == NULL) {
        return;
    }

    switch (cmd->type) {
        case PATH_CMD_HELP:
            path_print_help();
            break;
        case PATH_CMD_STATUS:
            path_print_status(rt);
            break;
        case PATH_CMD_CLEAR:
            if (rt->state == PATH_STATE_RECORDING) {
                LOGE(LOG_MOD_PATH, "error: stop record before clear\r\n");
            } else {
                s_pathCount = 0U;
                memset(s_pathPoints, 0, sizeof(s_pathPoints));
                LOGI(LOG_MOD_PATH, "clear ok\r\n");
            }
            break;
        case PATH_CMD_RECORD_START:
            path_start_record(rt);
            path_status_publish(rt, cmd->type,
                                s_pathOperationOk ? PATH_RESULT_OK : PATH_RESULT_ERROR,
                                s_pathOperationOk ? PATH_ERROR_NONE : PATH_ERROR_INS_NOT_READY,
                                true);
            break;
        case PATH_CMD_RECORD_STOP:
            path_stop_record(rt);
            path_status_publish(rt, cmd->type,
                                s_pathOperationOk ? PATH_RESULT_OK : PATH_RESULT_ERROR,
                                s_pathOperationOk ? PATH_ERROR_NONE : PATH_ERROR_ACTIVE,
                                true);
            break;
        case PATH_CMD_PRINT:
            path_print_points();
            break;
        case PATH_CMD_REPLAY:
            path_start_replay(rt);
            path_status_publish(rt, cmd->type,
                                s_pathOperationOk ? PATH_RESULT_RUNNING : PATH_RESULT_ERROR,
                                s_pathOperationOk ? PATH_ERROR_NONE :
                                ((s_pathCount < 2U) ? PATH_ERROR_TOO_SHORT : PATH_ERROR_MOTION_BUSY),
                                true);
            break;
        case PATH_CMD_FUSION_START:
            path_start_fusion(rt);
            path_status_publish(rt, cmd->type,
                                s_pathOperationOk ? PATH_RESULT_RUNNING : PATH_RESULT_ERROR,
                                s_pathOperationOk ? PATH_ERROR_NONE :
                                ((s_pathCount < 2U) ? PATH_ERROR_TOO_SHORT : PATH_ERROR_MOTION_BUSY),
                                true);
            break;
        case PATH_CMD_STOP:
            path_stop(rt);
            path_status_publish(rt, cmd->type, PATH_RESULT_OK,
                                PATH_ERROR_NONE, true);
            break;
        case PATH_CMD_SAVE:
            path_save_to_flash(rt);
            path_status_publish(rt, cmd->type,
                                s_pathOperationOk ? PATH_RESULT_OK : PATH_RESULT_ERROR,
                                s_pathOperationOk ? PATH_ERROR_NONE : PATH_ERROR_FLASH,
                                true);
            break;
        case PATH_CMD_LOAD:
            path_load_from_flash(rt);
            path_status_publish(rt, cmd->type,
                                s_pathOperationOk ? PATH_RESULT_OK : PATH_RESULT_ERROR,
                                s_pathOperationOk ? PATH_ERROR_NONE : PATH_ERROR_FLASH,
                                true);
            break;
        default:
            LOGE(LOG_MOD_PATH, "error: bad command\r\n");
            break;
    }
}
/**
 * @brief 周期性录制更新函数（每 50ms 由 path_task 调用）
 *
 * 仅在 PATH_STATE_RECORDING 状态下工作。
 * 以 5cm 位置变化 / 5deg 朝向变化为阈值，采样当前 INS 位姿并存入路径点缓冲区。
 *
 * 工作流程：
 *   1. 若缓冲区为空（起点缺失），先补录当前位姿作为第 0 点
 *   2. 检查当前位姿与最后一个录制点的距离和朝向差
 *   3. 超过阈值 -> 记录新点；缓冲区满 -> 自动停止录制
 */
static void path_update_record(Path_Runtime_t *rt)
{
    INS_Pose_t pose;
    Path_Point_t *last;
    float dist;
    float yaw_delta;

    if (rt->state != PATH_STATE_RECORDING) {
        return;
    }

    if (!path_get_pose(&pose)) {
        path_enter_error(rt, "INS lost");
        return;
    }

    if (s_pathCount == 0U) {
        (void)path_add_point(&pose);
        return;
    }

    last = &s_pathPoints[s_pathCount - 1U];
    dist = path_distance_to_point(&pose, last);
    yaw_delta = fabsf(path_wrap_180(pose.yaw_deg - last->yaw_deg));

    if (dist >= PATH_RECORD_MIN_DIST_M || yaw_delta >= PATH_RECORD_MIN_YAW_DEG) {
        if (!path_add_point(&pose)) {
            rt->state = PATH_STATE_IDLE;
            LOGI(LOG_MOD_PATH, "record full count=%u\r\n", (unsigned)s_pathCount);
        } else {
            LOGI(LOG_MOD_PATH, "record point #%u X=%+.3f Y=%+.3f YAW=%+.1f\r\n",
                    (unsigned)(s_pathCount - 1U),
                    pose.x_m,
                    pose.y_m,
                    pose.yaw_deg);
        }
    }
}

static void path_advance_replay(Path_Runtime_t *rt)
{
    rt->replay_index++;
    path_reset_motion_wait(rt);
    if (rt->replay_index >= s_pathCount) {
        rt->state = PATH_STATE_DONE;
        path_status_publish(rt, PATH_CMD_REPLAY, PATH_RESULT_DONE,
                            PATH_ERROR_NONE, true);
        LOGI(LOG_MOD_PATH, "replay done count=%u\r\n", (unsigned)s_pathCount);
    } else {
        rt->state = PATH_STATE_REPLAY_SEGMENT;
    }
}

/**
 * @brief 连续轨迹跟踪更新函数（每 50ms 由 path_task -> path_update_replay 调用）
 *
 * 仅在 PATH_STATE_REPLAY_TRACK 状态下工作，是当前默认的回放方式。
 *
 * 算法流程：
 *   1. 检查终点距离，若已接近终点且索引在最后 N 个点内 → 停止电机，状态切为 DONE
 *   2. Lookahead 推进：从当前索引开始依次向后检查，找到距离当前位置 >= 0.10m 的目标点
 *   3. 计算目标方向角 heading_deg = atan2(target->y - pose.y, target->x - pose.x)
 *   4. 航向误差 heading_error = heading_deg - pose.yaw_deg（正 = 偏左）
 *   5. 比例控制：trim = heading_error * YAW_KP(0.35)，限幅 ±8 PWM
 *   6. PWM 生成：left  = BASE_PWM(16) - trim, right = BASE_PWM(16) + trim
 *   7. PWM 限幅 [6, 30] 和缓变（每 50ms 最大变化 2）
 *   8. 发送到电机（首次启动需发 DIR + ON 命令）
 *   9. 每 500ms 输出一次 [PATH_TRACK] 日志
 */
static void path_update_track(Path_Runtime_t *rt)
{
    INS_Pose_t pose;                                      /*!< 当前 INS 位姿（含坐标和朝向） */
    Path_Point_t *target;                                 /*!< 当前跟踪的目标路点指针 */
    Path_Point_t *final;                                  /*!< 路径终点指针（最后一个路点） */
    const Path_Point_t *segment_start;                    /*!< 当前路段起点（上一路点，用于判定前进/后退） */
    TickType_t now;                                      /*!< 当前系统 tick，用于日志降频 */
    float final_dist;                                     /*!< 当前位置到终点的欧氏距离 (m) */
    float target_dist;                                    /*!< 当前位置到目标路点的欧氏距离 (m) */
    float dx;                                             /*!< 目标路点相对当前位置的 X 轴偏移 (m)，target->x_m - pose.x_m */
    float dy;                                             /*!< 目标路点相对当前位置的 Y 轴偏移 (m)，target->y_m - pose.y_m */
    float heading_deg;                                    /*!< 从当前位置指向目标路点的期望方向角 (deg)，atan2(dy, dx) */
    float track_heading_deg;                              /*!< 实际跟踪方向角 (deg)：前进段 = heading_deg，后退段 = heading_deg + 180° */
    float heading_error;                                  /*!< 航向误差 (deg)：正 = 偏左，已归一化到 (-180, 180] */
    float segment_move_heading_deg;                       /*!< 当前路段 from prev to target 的空间位移方向角 (deg) */
    float segment_body_error_deg;                         /*!< 路段位移方向与录制时车身朝向的夹角 (deg) */
    float trim_raw;                                       /*!< 未经限幅的差速修正量 (PWM) */
    float trim;                                           /*!< 限幅后的差速修正量 (PWM)，范围 [-8, 8] */
    float gray_trim = 0.0f;
    float path_trim;
    GrayLine_Status_t gray_status;
    float left_pwm_raw;                                   /*!< 未限幅的左轮目标 PWM */
    float right_pwm_raw;                                  /*!< 未限幅的右轮目标 PWM */
    float left_pwm_f;                                     /*!< 缓变后的左轮 PWM（浮点中间值） */
    float right_pwm_f;                                    /*!< 缓变后的右轮 PWM（浮点中间值） */
    int16_t left_pwm;                                     /*!< 最终输出左轮 PWM（整数） */
    int16_t right_pwm;                                    /*!< 最终输出右轮 PWM（整数） */
    uint16_t done_min_index;                              /*!< 允许判定到达终点的最小 replay_index（防提前完成保护） */
    bool near_final;                                      /*!< 是否已临近终点：final_dist <= COMPLETE_DIST_M (0.06m) */
    bool reverse_segment;                                 /*!< 当前路段是否需要倒车行驶 */
    bool direction_changed;                               /*!< 行进方向是否刚发生切换（前进↔后退） */

    if (rt->state != PATH_STATE_REPLAY_TRACK) {
        return;
    }

    if (!path_get_pose(&pose)) {
        path_enter_error(rt, "INS lost");
        return;
    }
    if (s_pathCount < 2U || rt->replay_index >= s_pathCount) {
        path_enter_error(rt, "bad replay index");
        return;
    }

    final = &s_pathPoints[s_pathCount - 1U];
    final_dist = path_distance_to_point(&pose, final);
    rt->track_last_final_dist_m = final_dist;

    /*
     * 防提前完成保护：计算"允许完成的最小索引"
     * 距离终点 >= 3 个点时不允许触发完成，只有进入最后 3 个点时才能完成。
     * 防止小车在路径中部偶然靠近终点位置时误判为已完成。
     * 例如：PATH_TRACK_DONE_INDEX_BACKOFF=3, s_pathCount=100
     *   → done_min_index = 97（只有 idx >= 97 时距离终点已不足 3 个点，才允许 done）
     */
    done_min_index = (s_pathCount > PATH_TRACK_DONE_INDEX_BACKOFF) ?
                     (uint16_t)(s_pathCount - PATH_TRACK_DONE_INDEX_BACKOFF) :
                     (uint16_t)(s_pathCount - 1U);

    /*
     * near_final：当前位置距离终点 <= 0.06m（位置距离，不是时间）
     * 完成条件 = near_final && replay_index >= done_min_index
     *   = 位置距离足够近 + 索引已进入路径尾部 → 才真正完成
     */
    near_final = final_dist <= PATH_TRACK_COMPLETE_DIST_M;
    if (near_final && rt->replay_index >= done_min_index) {
        path_track_stop_motors();
        rt->track_motor_started = false;
        rt->track_reverse = false;
        rt->track_last_left_pwm = 0;
        rt->track_last_right_pwm = 0;
        rt->track_last_dist_m = final_dist;
        rt->state = PATH_STATE_DONE;
        path_reset_motion_wait(rt);
        path_status_publish(rt, PATH_CMD_REPLAY, PATH_RESULT_DONE,
                            PATH_ERROR_NONE, true);
        LOGI(LOG_MOD_PATH, "replay done track dist=%.3f X=%+.3f Y=%+.3f YAW=%+.1f\r\n",
                final_dist,
                pose.x_m,
                pose.y_m,
                pose.yaw_deg);
        return;
    } else if (near_final) {
        /*
         * 日志降频输出：路径中部偶然靠近终点时，每 500ms 输出一条诊断日志。
         * 条件：距离终点 <= 0.06m，但 replay_index < done_min_index（还没到路径尾部）
         * → 防提前完成保护拦截了 done，但输出日志便于调试。
         * 降频控制：首次打印 或 距上次打印 >= 500ms 时才再次打印，防止刷屏。
         */
        now = xTaskGetTickCount();
        if (rt->track_near_final_log_tick == 0U ||
            ((uint32_t)((now - rt->track_near_final_log_tick) * portTICK_PERIOD_MS) >= PATH_TRACK_LOG_PERIOD_MS)) {
            rt->track_near_final_log_tick = now;
            LOGD(LOG_MOD_PATH, "near final ignored idx=%u/%u final_dist=%.3f done_idx=%u\r\n",
                                (unsigned)rt->replay_index,
                                (unsigned)s_pathCount,
                                final_dist,
                                (unsigned)done_min_index);
        }
    }

    /*
     * Lookahead 推进：向前寻找"前视距离"之外的目标点
     * 从当前 replay_index 开始逐个检查，找到距离小车 >= LOOKAHEAD_M (0.10m) 的路点。
     * 这样小车不会盯着脚下而是"看向前方"，提前转向使路径更平滑。
     * 如果所有点都不满足（路径末尾），replay_index 会停在 s_pathCount - 1（最后一个点）。
     */
    while (rt->replay_index < (s_pathCount - 1U)) {
        target_dist = path_distance_to_point(&pose, &s_pathPoints[rt->replay_index]);
        if (target_dist >= PATH_TRACK_LOOKAHEAD_M) {
            break;  /* 找到足够远的目标点，停止推进 */
        }
        rt->replay_index++;  /* 当前点太近，索引后移继续找 */
    }

    target = &s_pathPoints[rt->replay_index];                                           /* 获取目标路点指针 */
    segment_start = (rt->replay_index > 0U) ?
                    &s_pathPoints[rt->replay_index - 1U] :
                    &s_pathPoints[0U];
    if (path_segment_get_heading(segment_start, target,
                                 &segment_move_heading_deg,
                                 &segment_body_error_deg)) {
        reverse_segment = fabsf(segment_body_error_deg) > 90.0f;
    } else {
        segment_move_heading_deg = 0.0f;
        segment_body_error_deg = 0.0f;
        reverse_segment = false;
    }

    dx = target->x_m - pose.x_m;                                                        /* 目标方向 X 分量 */
    dy = target->y_m - pose.y_m;                                                        /* 目标方向 Y 分量 */
    target_dist = sqrtf((dx * dx) + (dy * dy));                                          /* 到目标点的直线距离 */
    if (target_dist < 0.001f) {                                                         /* 若距离接近零（无意义） */
        target_dist = final_dist;                                                       /* 回退：使用到终点的距离 */
        dx = final->x_m - pose.x_m;                                                     /* 重新计算方向指向终点 */
        dy = final->y_m - pose.y_m;
    }

    heading_deg = atan2f(dy, dx) * PATH_RAD_TO_DEG;                                     /* 计算目标方向角 (deg) */
    track_heading_deg = reverse_segment ? path_wrap_180(heading_deg + 180.0f) : heading_deg;
    heading_error = path_wrap_180(track_heading_deg - pose.yaw_deg);                     /* 航向误差：目标方向 - 当前朝向，正 = 偏左 */
    trim_raw = heading_error * PATH_TRACK_YAW_KP;
    trim = path_clamp_f32(trim_raw,
                          -PATH_TRACK_TRIM_MAX,
                          PATH_TRACK_TRIM_MAX);

    path_trim = trim;
    rt->fusion_path_trim_pwm = path_trim;
    rt->fusion_gray_trim_pwm = 0.0f;
    rt->fusion_final_trim_pwm = path_trim;
    rt->fusion_mode = PATH_FUSION_MODE_PATH_ONLY;
    if (rt->fusion_active) {
        if (reverse_segment) {
            rt->fusion_mode = PATH_FUSION_MODE_REVERSE;
        } else if (GrayLine_ReadStatus(&gray_status) &&
                   gray_status.path_assist_active && gray_status.line_fresh &&
                   (((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) -
                     gray_status.sample_tick) <= PATH_FUSION_GRAY_MAX_AGE_MS)) {
            gray_trim = path_clamp_f32(gray_status.pos_turn_ff_mps *
                                       g_pathFusionConfig.gray_turn_to_pwm,
                                       -g_pathFusionConfig.gray_trim_max_pwm,
                                       g_pathFusionConfig.gray_trim_max_pwm);
            rt->fusion_gray_trim_pwm = gray_trim;
            trim = (g_pathFusionConfig.gray_weight * gray_trim) +
                   ((1.0f - g_pathFusionConfig.gray_weight) * path_trim);
            trim = path_clamp_f32(trim, -PATH_TRACK_TRIM_MAX, PATH_TRACK_TRIM_MAX);
            rt->fusion_mode = PATH_FUSION_MODE_BLEND;
        } else {
            rt->fusion_mode = PATH_FUSION_MODE_STALE;
        }
        rt->fusion_final_trim_pwm = trim;
    }

    direction_changed = rt->track_motor_started && (rt->track_reverse != reverse_segment);

    if (!path_track_switch_direction(rt, reverse_segment)) {
        path_enter_error(rt, "motor dir failed");
        return;
    }

    if (reverse_segment) {
        left_pwm_raw = PATH_TRACK_BASE_PWM + trim;
        right_pwm_raw = PATH_TRACK_BASE_PWM - trim;
    } else {
        left_pwm_raw = PATH_TRACK_BASE_PWM - trim;                                       /* 左轮：基速 - 修正 (偏左时左轮减速) */
        right_pwm_raw = PATH_TRACK_BASE_PWM + trim;                                      /* 右轮：基速 + 修正 (偏左时右轮加速) */
    }
    left_pwm_f = path_clamp_f32(left_pwm_raw, PATH_TRACK_PWM_MIN, PATH_TRACK_PWM_MAX);   /* 左轮 PWM 限幅 [6,30] */
    right_pwm_f = path_clamp_f32(right_pwm_raw, PATH_TRACK_PWM_MIN, PATH_TRACK_PWM_MAX); /* 右轮 PWM 限幅 [6,30] */

    left_pwm_f = path_slew_f32((float)rt->track_last_left_pwm,                          /* 左轮 PWM 缓变：防突变 */
                               left_pwm_f,
                               PATH_TRACK_PWM_SLEW);
    right_pwm_f = path_slew_f32((float)rt->track_last_right_pwm,                        /* 右轮 PWM 缓变 */
                                right_pwm_f,
                                PATH_TRACK_PWM_SLEW);
    left_pwm_f = path_clamp_f32(left_pwm_f, PATH_TRACK_PWM_MIN, PATH_TRACK_PWM_MAX);     /* 缓变后再限幅 */
    right_pwm_f = path_clamp_f32(right_pwm_f, PATH_TRACK_PWM_MIN, PATH_TRACK_PWM_MAX);
    left_pwm = path_clamp_pwm(left_pwm_f);                                               /* float 转 int16_t */
    right_pwm = path_clamp_pwm(right_pwm_f);

    if (!rt->track_motor_started) {                                                     /* 首次启动：需发 DIR + ON */
        if (!path_track_start_motors(reverse_segment, left_pwm, right_pwm)) {
            path_enter_error(rt, "motor start failed");
            return;
        }
        rt->track_motor_started = true;
        rt->track_reverse = reverse_segment;
    } else if (!path_track_apply_pwm(left_pwm, right_pwm)) {                            /* 运行中：仅更新 PWM */
        path_enter_error(rt, "motor pwm failed");
        return;
    }

    rt->track_target_index = rt->replay_index;                                           /* 保存当前跟踪目标索引 */
    rt->track_last_dist_m = target_dist;                                                 /* 保存到目标点的距离 */
    rt->track_last_heading_error_deg = heading_error;                                    /* 保存航向误差 */
    rt->track_last_left_pwm = left_pwm;                                                  /* 保存左轮 PWM (用于下次缓变) */
    rt->track_last_right_pwm = right_pwm;                                                /* 保存右轮 PWM */

    if (direction_changed) {
        LOGD(LOG_MOD_PATH, "%c->%c idx=%u seg=%u->%u recYaw=%+.1f move=%+.1f segErr=%+.1f pwm=%d/%d\r\n",
                            reverse_segment ? 'F' : 'B',
                            reverse_segment ? 'B' : 'F',
                            (unsigned)rt->track_target_index,
                            (unsigned)(rt->track_target_index - 1U),
                            (unsigned)rt->track_target_index,
                            segment_start->yaw_deg,
                            segment_move_heading_deg,
                            segment_body_error_deg,
                            (int)rt->track_last_left_pwm,
                            (int)rt->track_last_right_pwm);
    }

    now = xTaskGetTickCount();
    if (rt->track_last_log_tick == 0U ||                                                /* 首次 或 距上次 >= 500ms */
        ((uint32_t)((now - rt->track_last_log_tick) * portTICK_PERIOD_MS) >= PATH_TRACK_LOG_PERIOD_MS)) {
        rt->track_last_log_tick = now;
        LOGD(LOG_MOD_PATH, "idx=%u/%u seg=%u->%u dir=%c recYaw=%+.1f move=%+.1f segErr=%+.1f track=%+.1f herr=%+.1f rawTrim=%+.2f trim=%+.2f rawPwm=%.1f/%.1f pwm=%d/%d X=%+.3f Y=%+.3f YAW=%+.1f\r\n",
                            (unsigned)rt->track_target_index,                           /* 目标点索引 */
                            (unsigned)s_pathCount,                                      /* 总点数 */
                            (unsigned)(rt->track_target_index - 1U),
                            (unsigned)rt->track_target_index,
                            reverse_segment ? 'B' : 'F',
                            segment_start->yaw_deg,
                            segment_move_heading_deg,
                            segment_body_error_deg,
                            track_heading_deg,
                            rt->track_last_dist_m,                                      /* 到目标距离 */
                            rt->track_last_heading_error_deg,                           /* 航向误差 */
                            trim_raw,
                            trim,
                            left_pwm_raw,
                            right_pwm_raw,
                            (int)rt->track_last_left_pwm,                               /* 左轮 PWM */
                            (int)rt->track_last_right_pwm,                              /* 右轮 PWM */
                            pose.x_m,                                                   /* 当前 X */
                            pose.y_m,                                                   /* 当前 Y */
                            pose.yaw_deg);                                              /* 当前朝向 */
    }
}

/**
 * @brief 回放更新函数（每 50ms 由 path_task 调用）
 *
 * 当前默认使用 REPLAY_TRACK 连续轨迹跟踪模式：
 *   交由 path_update_track() 处理，基于 Lookahead 前视 + 航向误差比例控制
 *   直接驱动电机 PWM，实现平滑连续的回放行驶。
 *
 * 旧版分段回放（REPLAY_SEGMENT/TURN/DRIVE）已关闭，代码保留仅做参考。
 */
static void path_update_replay(Path_Runtime_t *rt)
{
    const Path_Point_t *prev;
    Path_Point_t *target;
    float dx;
    float dy;
    float dist;
    float yaw_delta;
    float radius_m;
    float half_angle_rad;
    int8_t motion_result;
    bool reverse_segment;

    if (rt->state == PATH_STATE_REPLAY_TRACK) {
        path_update_track(rt);
        return;
    }

    if (rt->state != PATH_STATE_REPLAY_SEGMENT &&
        rt->state != PATH_STATE_REPLAY_TURN &&
        rt->state != PATH_STATE_REPLAY_DRIVE) {
        return;
    }

    if (rt->replay_index >= s_pathCount) {
        rt->state = PATH_STATE_DONE;
        path_reset_motion_wait(rt);
        return;
    }

    if (rt->motion_cmd_sent) {
        motion_result = path_motion_step_result(rt);
        if (motion_result > 0) {
            path_advance_replay(rt);
        } else if (motion_result < 0) {
            path_enter_error(rt, "motion step failed");
        }
        return;
    }

    prev = &s_pathPoints[rt->replay_index - 1U];
    target = &s_pathPoints[rt->replay_index];
    dx = target->x_m - prev->x_m;
    dy = target->y_m - prev->y_m;
    dist = sqrtf((dx * dx) + (dy * dy));
    yaw_delta = path_wrap_180(target->yaw_deg - prev->yaw_deg);
    reverse_segment = path_segment_is_reverse(prev, target);

    if (dist <= PATH_MIN_DRIVE_M && fabsf(yaw_delta) <= PATH_YAW_TOL_DEG) {
        path_advance_replay(rt);
        return;
    }

    if (dist > PATH_MAX_LEG_M) {
        path_enter_error(rt, "leg too long");
        return;
    }

    if (!reverse_segment &&
        fabsf(yaw_delta) >= PATH_REPLAY_ARC_MIN_YAW_DEG && dist > PATH_MIN_DRIVE_M) {
        half_angle_rad = fabsf(yaw_delta) * PATH_DEG_TO_RAD * 0.5f;
        radius_m = dist / (2.0f * sinf(half_angle_rad));
        if (radius_m >= PATH_REPLAY_MIN_ARC_RADIUS_M &&
            radius_m <= PATH_REPLAY_MAX_ARC_RADIUS_M) {
            if (!path_start_motion_step2(rt, MOTION_CMD_ARC, radius_m, yaw_delta)) {
                path_enter_error(rt, "motion arc failed");
            }
            return;
        }
    }

    if (fabsf(yaw_delta) > PATH_YAW_TOL_DEG && dist <= PATH_MIN_DRIVE_M) {
        if (!path_start_motion_step(rt, MOTION_CMD_TURN, yaw_delta)) {
            path_enter_error(rt, "motion turn failed");
        }
        return;
    }

    if (dist > PATH_MIN_DRIVE_M) {
        Motion_CommandType_t drive_type = reverse_segment ? MOTION_CMD_BACK : MOTION_CMD_FWD;
        if (!path_start_motion_step(rt, drive_type, dist)) {
            path_enter_error(rt, "motion drive failed");
        }
    } else {
        path_advance_replay(rt);
    }
}

void path_task(void *pvParameters)
{
    Path_Runtime_t rt;
    Path_Command_t cmd;

    (void)pvParameters;
    memset(&rt, 0, sizeof(rt));
    rt.state = PATH_STATE_IDLE;
    path_status_publish(&rt, PATH_CMD_STATUS, PATH_RESULT_NONE,
                        PATH_ERROR_NONE, true);

    while (1) {
        while (g_pathCmdQueue != NULL &&
               xQueueReceive(g_pathCmdQueue, &cmd, 0) == pdTRUE) {
            path_handle_command(&rt, &cmd);
        }

        path_update_record(&rt);
        path_update_replay(&rt);
        path_status_publish(&rt, PATH_CMD_STATUS, PATH_RESULT_NONE,
                            PATH_ERROR_NONE, false);
        vTaskDelay(pdMS_TO_TICKS(PATH_TASK_PERIOD_MS));
    }
}
