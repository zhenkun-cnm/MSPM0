/**
 * @file    app_path.c
 * @brief   Teach-and-replay path test based on INS pose and motion commands.
 */
#include "app_path.h"
#include "app_ins.h"
#include "app_motion.h"
#include "app_tb6612.h"
#include "dev_flash.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define PATH_TASK_PERIOD_MS          50U
#define PATH_MAX_POINTS              500U
#define PATH_RECORD_MIN_DIST_M       0.05f
#define PATH_RECORD_MIN_YAW_DEG      5.0f
#define PATH_POS_TOL_M               0.05f
#define PATH_YAW_TOL_DEG             3.0f
#define PATH_MIN_DRIVE_M             0.03f
#define PATH_MAX_LEG_M               1.00f
#define PATH_REPLAY_ARC_MIN_YAW_DEG  5.0f
#define PATH_REPLAY_MIN_ARC_RADIUS_M 0.15f
#define PATH_REPLAY_MAX_ARC_RADIUS_M 2.00f
#define PATH_MOTION_START_TIMEOUT_MS 500U
#define PATH_CMD_ID_BASE             0x72000000UL
#define PATH_RAD_TO_DEG              57.295779513082320876f
#define PATH_DEG_TO_RAD              0.017453292519943295f
#define PATH_TRACK_LOOKAHEAD_M       0.10f
#define PATH_TRACK_COMPLETE_DIST_M   0.06f
#define PATH_TRACK_DONE_INDEX_BACKOFF 3U
#define PATH_TRACK_BASE_PWM          16.0f
#define PATH_TRACK_YAW_KP            0.35f
#define PATH_TRACK_TRIM_MAX          8.0f
#define PATH_TRACK_PWM_MIN           6.0f
#define PATH_TRACK_PWM_MAX           30.0f
#define PATH_TRACK_PWM_SLEW          2.0f
#define PATH_TRACK_LOG_PERIOD_MS     500U
#define PATH_FLASH_START_ADDR        0x000F0000UL
#define PATH_FLASH_SECTOR_SIZE       4096UL
#define PATH_FLASH_SECTOR_COUNT      2U
#define PATH_FLASH_BYTES             (PATH_FLASH_SECTOR_SIZE * PATH_FLASH_SECTOR_COUNT)
#define PATH_FLASH_MAGIC             0x48545052UL /* "RPTH" little-endian */
#define PATH_FLASH_VERSION           1U

QueueHandle_t g_pathCmdQueue = NULL;

typedef enum {
    PATH_STATE_IDLE = 0,
    PATH_STATE_RECORDING,
    PATH_STATE_REPLAY_TURN,
    PATH_STATE_REPLAY_DRIVE,
    PATH_STATE_REPLAY_SEGMENT,
    PATH_STATE_REPLAY_TRACK,
    PATH_STATE_DONE,
    PATH_STATE_ERROR
} Path_State_t;

typedef struct {
    float x_m;
    float y_m;
    float yaw_deg;
} Path_Point_t;

typedef struct {
    Path_State_t state;
    uint16_t replay_index;
    uint32_t next_motion_cmd_id;
    uint32_t waiting_motion_cmd_id;
    bool motion_cmd_sent;
    bool motion_accepted;
    TickType_t motion_cmd_tick;
    uint16_t track_target_index;
    float track_last_dist_m;
    float track_last_final_dist_m;
    float track_last_heading_error_deg;
    int16_t track_last_left_pwm;
    int16_t track_last_right_pwm;
    TickType_t track_last_log_tick;
    TickType_t track_near_final_log_tick;
    bool track_motor_started;
} Path_Runtime_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t point_size;
    uint16_t max_points;
    uint16_t count;
    uint32_t data_bytes;
    uint32_t checksum;
} Path_FlashHeader_t;

static Path_Point_t s_pathPoints[PATH_MAX_POINTS];
static uint16_t s_pathCount = 0U;

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
        LOG_RAW("[PATH] error: motor queue not ready\r\n");
        return false;
    }

    cmd.type = type;
    cmd.val = val;
    return xQueueSend(g_motorCmdQueue, &cmd, wait_ticks) == pdTRUE;
}

static void path_track_stop_motors(void)
{
    (void)path_send_motor_cmd(MOTOR_CMD_LEFT_SPEED, 0, pdMS_TO_TICKS(20));
    (void)path_send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, 0, pdMS_TO_TICKS(20));
    (void)path_send_motor_cmd(MOTOR_CMD_ONOFF, 0, pdMS_TO_TICKS(20));
}

static bool path_track_apply_pwm(int16_t left_pwm, int16_t right_pwm)
{
    if (!path_send_motor_cmd(MOTOR_CMD_LEFT_SPEED, left_pwm, pdMS_TO_TICKS(2))) {
        return false;
    }
    if (!path_send_motor_cmd(MOTOR_CMD_RIGHT_SPEED, right_pwm, pdMS_TO_TICKS(2))) {
        return false;
    }
    return true;
}

static bool path_track_start_motors(int16_t left_pwm, int16_t right_pwm)
{
    if (!path_send_motor_cmd(MOTOR_CMD_DIR, (int16_t)TB6612_DIR_FORWARD, pdMS_TO_TICKS(20))) {
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

static bool path_send_motion_raw2(Motion_CommandType_t type, float value, float value2, uint32_t cmd_id)
{
    Motion_Command_t cmd;
    cmd.type = type;
    cmd.value = value;
    cmd.value2 = value2;
    cmd.cmd_id = cmd_id;

    if (g_motionCmdQueue == NULL) {
        LOG_RAW("[PATH] error: motion queue not ready\r\n");
        return false;
    }

    if (xQueueSend(g_motionCmdQueue, &cmd, pdMS_TO_TICKS(20)) != pdTRUE) {
        LOG_RAW("[PATH] error: motion queue full\r\n");
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
        LOG_RAW("[PATH] error: motion busy\r\n");
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
    log_printf_internal("[PATH_STEP] send motion cmd_id=0x%08lX type=%d value=%+.3f value2=%+.3f point=%u/%u\r\n",
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
        log_printf_internal("[PATH_STEP] motion rejected cmd_id=0x%08lX\r\n",
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
        log_printf_internal("[PATH_STEP] motion finished cmd_id=0x%08lX result=%s\r\n",
                            (unsigned long)rt->waiting_motion_cmd_id,
                            path_motion_result_name(g_motionRtStatus.last_result));
        return -1;
    }

    elapsed_ms = (uint32_t)((xTaskGetTickCount() - rt->motion_cmd_tick) * portTICK_PERIOD_MS);
    if (!rt->motion_accepted && elapsed_ms >= PATH_MOTION_START_TIMEOUT_MS) {
        log_printf_internal("[PATH_STEP] motion did not start cmd_id=0x%08lX state=%d active=0x%08lX done=0x%08lX result=%s\r\n",
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
        LOG_RAW("[PATH_FLASH] error: flash handle NULL\r\n");
        return false;
    }

    flash->init(flash);
    if (!flash->readJEDECID(flash, &id) ||
        id.manufacturer != 0xEF ||
        id.memoryType != 0x40) {
        LOG_RAW("[PATH_FLASH] error: JEDEC invalid\r\n");
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

    if (path_is_active(rt)) {
        LOG_RAW("[PATH_FLASH] error: stop record/replay before save\r\n");
        return;
    }
    if (s_pathCount < 2U) {
        LOG_RAW("[PATH_FLASH] error: need at least 2 points\r\n");
        return;
    }

    data_bytes = (uint32_t)s_pathCount * (uint32_t)sizeof(Path_Point_t);
    total_bytes = (uint32_t)sizeof(header) + data_bytes;
    if (total_bytes > PATH_FLASH_BYTES) {
        LOG_RAW("[PATH_FLASH] error: path too large bytes=%lu\r\n",
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

    LOG_RAW("[PATH_FLASH] save start addr=0x%08lX count=%u bytes=%lu\r\n",
            (unsigned long)PATH_FLASH_START_ADDR,
            (unsigned)s_pathCount,
            (unsigned long)total_bytes);

    for (i = 0U; i < PATH_FLASH_SECTOR_COUNT; i++) {
        if (!flash->sectorErase(flash,
                                PATH_FLASH_START_ADDR + (i * PATH_FLASH_SECTOR_SIZE))) {
            LOG_RAW("[PATH_FLASH] error: erase failed sector=%lu\r\n",
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
        LOG_RAW("[PATH_FLASH] error: write failed\r\n");
        return;
    }

    LOG_RAW("[PATH_FLASH] save ok count=%u checksum=0x%08lX\r\n",
            (unsigned)s_pathCount,
            (unsigned long)header.checksum);
}

static void path_load_from_flash(Path_Runtime_t *rt)
{
    DevFlash *flash;
    Path_FlashHeader_t header;
    uint32_t checksum;

    if (path_is_active(rt)) {
        LOG_RAW("[PATH_FLASH] error: stop record/replay before load\r\n");
        return;
    }

    if (!path_flash_probe(&flash)) {
        return;
    }

    if (!flash->read(flash,
                     PATH_FLASH_START_ADDR,
                     (uint8_t *)&header,
                     (uint16_t)sizeof(header))) {
        LOG_RAW("[PATH_FLASH] error: header read failed\r\n");
        return;
    }

    if (header.magic != PATH_FLASH_MAGIC ||
        header.version != PATH_FLASH_VERSION ||
        header.point_size != sizeof(Path_Point_t) ||
        header.count > PATH_MAX_POINTS ||
        header.count < 2U ||
        header.data_bytes != ((uint32_t)header.count * (uint32_t)sizeof(Path_Point_t))) {
        LOG_RAW("[PATH_FLASH] error: no valid path in flash\r\n");
        return;
    }

    if (((uint32_t)sizeof(header) + header.data_bytes) > PATH_FLASH_BYTES) {
        LOG_RAW("[PATH_FLASH] error: stored path too large\r\n");
        return;
    }

    memset(s_pathPoints, 0, sizeof(s_pathPoints));
    if (!flash->read(flash,
                     PATH_FLASH_START_ADDR + (uint32_t)sizeof(header),
                     (uint8_t *)s_pathPoints,
                     (uint16_t)header.data_bytes)) {
        s_pathCount = 0U;
        LOG_RAW("[PATH_FLASH] error: data read failed\r\n");
        return;
    }

    checksum = path_checksum_bytes((const uint8_t *)s_pathPoints, header.data_bytes);
    if (checksum != header.checksum) {
        memset(s_pathPoints, 0, sizeof(s_pathPoints));
        s_pathCount = 0U;
        LOG_RAW("[PATH_FLASH] error: checksum mismatch got=0x%08lX expect=0x%08lX\r\n",
                (unsigned long)checksum,
                (unsigned long)header.checksum);
        return;
    }

    s_pathCount = header.count;
    rt->state = PATH_STATE_IDLE;
    rt->replay_index = 0U;
    rt->track_target_index = 0U;
    path_reset_motion_wait(rt);

    LOG_RAW("[PATH_FLASH] load ok count=%u checksum=0x%08lX\r\n",
            (unsigned)s_pathCount,
            (unsigned long)checksum);
}

static void path_print_help(void)
{
    LOG_RAW("[PATH] commands:\r\n");
    LOG_RAW("  path help\r\n");
    LOG_RAW("  path status\r\n");
    LOG_RAW("  path clear\r\n");
    LOG_RAW("  path record start\r\n");
    LOG_RAW("  path record stop\r\n");
    LOG_RAW("  path print\r\n");
    LOG_RAW("  path replay\r\n");
    LOG_RAW("  path stop\r\n");
    LOG_RAW("  path save\r\n");
    LOG_RAW("  path load\r\n");
}

static void path_print_status(const Path_Runtime_t *rt)
{
    LOG_RAW("[PATH] state=%s count=%u replay=%u/%u target=%u dist=%.3f final=%.3f herr=%+.1f pwm L=%d R=%d wait=0x%08lX active=0x%08lX done=0x%08lX rejected=0x%08lX result=%s\r\n",
            path_state_name(rt->state),
            (unsigned)s_pathCount,
            (unsigned)(rt->replay_index + 1U),
            (unsigned)s_pathCount,
            (unsigned)rt->track_target_index,
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
}

static void path_print_points(void)
{
    uint16_t i;

    LOG_RAW("[PATH] points count=%u\r\n", (unsigned)s_pathCount);
    for (i = 0U; i < s_pathCount; i++) {
        LOG_RAW("[PATH_PT] #%02u X=%+.3f Y=%+.3f YAW=%+.1f\r\n",
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
    LOG_RAW("[PATH] error: %s\r\n", reason);
}

static void path_start_record(Path_Runtime_t *rt)
{
    INS_Pose_t pose;

    if (!path_get_pose(&pose)) {
        LOG_RAW("[PATH] error: INS not ready\r\n");
        return;
    }

    memset(s_pathPoints, 0, sizeof(s_pathPoints));
    s_pathCount = 0U;
    (void)path_add_point(&pose);
    rt->state = PATH_STATE_RECORDING;
    rt->replay_index = 0U;
    path_reset_motion_wait(rt);
    LOG_RAW("[PATH] record start\r\n");
}

static void path_stop_record(Path_Runtime_t *rt)
{
    if (rt->state == PATH_STATE_RECORDING) {
        rt->state = PATH_STATE_IDLE;
        LOG_RAW("[PATH] record stop count=%u\r\n", (unsigned)s_pathCount);
    } else {
        LOG_RAW("[PATH] record not active\r\n");
    }
}

static void path_start_replay(Path_Runtime_t *rt)
{
    if (s_pathCount < 2U) {
        LOG_RAW("[PATH] error: need at least 2 points\r\n");
        return;
    }
    if (!path_motion_idle()) {
        LOG_RAW("[PATH] error: motion busy\r\n");
        return;
    }

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
    path_reset_motion_wait(rt);
    LOG_RAW("[PATH] replay start count=%u mode=continuous_track\r\n", (unsigned)s_pathCount);
}

static void path_stop(Path_Runtime_t *rt)
{
    (void)path_send_motion_raw(MOTION_CMD_STOP, 0.0f, 0U);
    path_track_stop_motors();
    rt->state = PATH_STATE_IDLE;
    path_reset_motion_wait(rt);
    rt->track_motor_started = false;
    rt->track_last_left_pwm = 0;
    rt->track_last_right_pwm = 0;
    rt->track_last_final_dist_m = 0.0f;
    LOG_RAW("[PATH] stop ok\r\n");
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
                LOG_RAW("[PATH] error: stop record before clear\r\n");
            } else {
                s_pathCount = 0U;
                memset(s_pathPoints, 0, sizeof(s_pathPoints));
                LOG_RAW("[PATH] clear ok\r\n");
            }
            break;
        case PATH_CMD_RECORD_START:
            path_start_record(rt);
            break;
        case PATH_CMD_RECORD_STOP:
            path_stop_record(rt);
            break;
        case PATH_CMD_PRINT:
            path_print_points();
            break;
        case PATH_CMD_REPLAY:
            path_start_replay(rt);
            break;
        case PATH_CMD_STOP:
            path_stop(rt);
            break;
        case PATH_CMD_SAVE:
            path_save_to_flash(rt);
            break;
        case PATH_CMD_LOAD:
            path_load_from_flash(rt);
            break;
        default:
            LOG_RAW("[PATH] error: bad command\r\n");
            break;
    }
}

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
            LOG_RAW("[PATH] record full count=%u\r\n", (unsigned)s_pathCount);
        } else {
            LOG_RAW("[PATH] record point #%u X=%+.3f Y=%+.3f YAW=%+.1f\r\n",
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
        LOG_RAW("[PATH] replay done count=%u\r\n", (unsigned)s_pathCount);
    } else {
        rt->state = PATH_STATE_REPLAY_SEGMENT;
    }
}

static void path_update_track(Path_Runtime_t *rt)
{
    INS_Pose_t pose;
    Path_Point_t *target;
    Path_Point_t *final;
    TickType_t now;
    float final_dist;
    float target_dist;
    float dx;
    float dy;
    float heading_deg;
    float heading_error;
    float trim;
    float left_pwm_f;
    float right_pwm_f;
    int16_t left_pwm;
    int16_t right_pwm;
    uint16_t done_min_index;
    bool near_final;

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
    done_min_index = (s_pathCount > PATH_TRACK_DONE_INDEX_BACKOFF) ?
                     (uint16_t)(s_pathCount - PATH_TRACK_DONE_INDEX_BACKOFF) :
                     (uint16_t)(s_pathCount - 1U);
    near_final = final_dist <= PATH_TRACK_COMPLETE_DIST_M;
    if (near_final && rt->replay_index >= done_min_index) {
        path_track_stop_motors();
        rt->track_motor_started = false;
        rt->track_last_left_pwm = 0;
        rt->track_last_right_pwm = 0;
        rt->track_last_dist_m = final_dist;
        rt->state = PATH_STATE_DONE;
        path_reset_motion_wait(rt);
        LOG_RAW("[PATH] replay done track dist=%.3f X=%+.3f Y=%+.3f YAW=%+.1f\r\n",
                final_dist,
                pose.x_m,
                pose.y_m,
                pose.yaw_deg);
        return;
    } else if (near_final) {
        now = xTaskGetTickCount();
        if (rt->track_near_final_log_tick == 0U ||
            ((uint32_t)((now - rt->track_near_final_log_tick) * portTICK_PERIOD_MS) >= PATH_TRACK_LOG_PERIOD_MS)) {
            rt->track_near_final_log_tick = now;
            log_printf_internal("[PATH_TRACK] near final ignored idx=%u/%u final_dist=%.3f done_idx=%u\r\n",
                                (unsigned)rt->replay_index,
                                (unsigned)s_pathCount,
                                final_dist,
                                (unsigned)done_min_index);
        }
    }

    while (rt->replay_index < (s_pathCount - 1U)) {
        target_dist = path_distance_to_point(&pose, &s_pathPoints[rt->replay_index]);
        if (target_dist >= PATH_TRACK_LOOKAHEAD_M) {
            break;
        }
        rt->replay_index++;
    }

    target = &s_pathPoints[rt->replay_index];
    dx = target->x_m - pose.x_m;
    dy = target->y_m - pose.y_m;
    target_dist = sqrtf((dx * dx) + (dy * dy));
    if (target_dist < 0.001f) {
        target_dist = final_dist;
        dx = final->x_m - pose.x_m;
        dy = final->y_m - pose.y_m;
    }

    heading_deg = atan2f(dy, dx) * PATH_RAD_TO_DEG;
    heading_error = path_wrap_180(heading_deg - pose.yaw_deg);
    trim = path_clamp_f32(heading_error * PATH_TRACK_YAW_KP,
                          -PATH_TRACK_TRIM_MAX,
                          PATH_TRACK_TRIM_MAX);

    left_pwm_f = PATH_TRACK_BASE_PWM - trim;
    right_pwm_f = PATH_TRACK_BASE_PWM + trim;
    left_pwm_f = path_clamp_f32(left_pwm_f, PATH_TRACK_PWM_MIN, PATH_TRACK_PWM_MAX);
    right_pwm_f = path_clamp_f32(right_pwm_f, PATH_TRACK_PWM_MIN, PATH_TRACK_PWM_MAX);

    left_pwm_f = path_slew_f32((float)rt->track_last_left_pwm,
                               left_pwm_f,
                               PATH_TRACK_PWM_SLEW);
    right_pwm_f = path_slew_f32((float)rt->track_last_right_pwm,
                                right_pwm_f,
                                PATH_TRACK_PWM_SLEW);
    left_pwm_f = path_clamp_f32(left_pwm_f, PATH_TRACK_PWM_MIN, PATH_TRACK_PWM_MAX);
    right_pwm_f = path_clamp_f32(right_pwm_f, PATH_TRACK_PWM_MIN, PATH_TRACK_PWM_MAX);
    left_pwm = path_clamp_pwm(left_pwm_f);
    right_pwm = path_clamp_pwm(right_pwm_f);

    if (!rt->track_motor_started) {
        if (!path_track_start_motors(left_pwm, right_pwm)) {
            path_enter_error(rt, "motor start failed");
            return;
        }
        rt->track_motor_started = true;
    } else if (!path_track_apply_pwm(left_pwm, right_pwm)) {
        path_enter_error(rt, "motor pwm failed");
        return;
    }

    rt->track_target_index = rt->replay_index;
    rt->track_last_dist_m = target_dist;
    rt->track_last_heading_error_deg = heading_error;
    rt->track_last_left_pwm = left_pwm;
    rt->track_last_right_pwm = right_pwm;

    now = xTaskGetTickCount();
    if (rt->track_last_log_tick == 0U ||
        ((uint32_t)((now - rt->track_last_log_tick) * portTICK_PERIOD_MS) >= PATH_TRACK_LOG_PERIOD_MS)) {
        rt->track_last_log_tick = now;
        log_printf_internal("[PATH_TRACK] idx=%u/%u dist=%.3f herr=%+.1f pwm=%d/%d X=%+.3f Y=%+.3f YAW=%+.1f\r\n",
                            (unsigned)rt->track_target_index,
                            (unsigned)s_pathCount,
                            rt->track_last_dist_m,
                            rt->track_last_heading_error_deg,
                            (int)rt->track_last_left_pwm,
                            (int)rt->track_last_right_pwm,
                            pose.x_m,
                            pose.y_m,
                            pose.yaw_deg);
    }
}

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

    if (dist <= PATH_MIN_DRIVE_M && fabsf(yaw_delta) <= PATH_YAW_TOL_DEG) {
        path_advance_replay(rt);
        return;
    }

    if (dist > PATH_MAX_LEG_M) {
        path_enter_error(rt, "leg too long");
        return;
    }

    if (fabsf(yaw_delta) >= PATH_REPLAY_ARC_MIN_YAW_DEG && dist > PATH_MIN_DRIVE_M) {
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
        if (!path_start_motion_step(rt, MOTION_CMD_FWD, dist)) {
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

    LOG_INFO("[PATH] task ready: path help\r\n");

    while (1) {
        while (g_pathCmdQueue != NULL &&
               xQueueReceive(g_pathCmdQueue, &cmd, 0) == pdTRUE) {
            path_handle_command(&rt, &cmd);
        }

        path_update_record(&rt);
        path_update_replay(&rt);
        vTaskDelay(pdMS_TO_TICKS(PATH_TASK_PERIOD_MS));
    }
}
