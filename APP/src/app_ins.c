/**
 * @file    app_ins.c
 * @brief   Wheel odometry + existing IMU yaw pose estimator.
 */
#include "app_ins.h"
#include "app_imu.h"
#include "app_motor_encoder.h"
#include "dev_flash.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "port_log.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

INS_PoseGlobal_t g_insPoseGlobal = {{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0U}, NULL};
QueueHandle_t g_insCmdQueue = NULL;

bool INS_Pose_Read(INS_Pose_t *out)
{
    if (out == NULL || g_insPoseGlobal.lock == NULL) return false;
    if (xSemaphoreTake(g_insPoseGlobal.lock, pdMS_TO_TICKS(1)) != pdTRUE) return false;
    *out = g_insPoseGlobal.pose;
    xSemaphoreGive(g_insPoseGlobal.lock);
    return true;
}

void INS_Pose_Write(const INS_Pose_t *in)
{
    if (in == NULL || g_insPoseGlobal.lock == NULL) return;
    if (xSemaphoreTake(g_insPoseGlobal.lock, pdMS_TO_TICKS(2)) != pdTRUE) return;
    g_insPoseGlobal.pose = *in;
    xSemaphoreGive(g_insPoseGlobal.lock);
}

#define INS_TASK_PERIOD_MS             10U
#define INS_PRINT_PERIOD_MS            100U
#define INS_WAIT_PRINT_PERIOD_MS       1000U
#define INS_FLASH_LOG_PERIOD_MS        200U

#define INS_WHEEL_DIAMETER_M           0.048f
#define INS_WHEEL_BASE_M               0.125f
#define INS_PI_F                       3.14159265358979323846f
#define INS_WHEEL_CIRCUMFERENCE_M      (INS_PI_F * INS_WHEEL_DIAMETER_M)

#define INS_LEFT_SIGN                  (1.0f)
#define INS_RIGHT_SIGN                 (1.0f)
#define INS_IMU_YAW_SIGN               (1.0f)

#define INS_FLASH_LOG_START_ADDR       (0x00100000UL)
#define INS_FLASH_LOG_SECTOR_SIZE      (4096UL)
#define INS_FLASH_LOG_SECTOR_COUNT     (8UL)
#define INS_FLASH_LOG_BYTES            (INS_FLASH_LOG_SECTOR_SIZE * INS_FLASH_LOG_SECTOR_COUNT)
#define INS_FLASH_RECORD_SIZE          (32U)
#define INS_FLASH_RECORD_COUNT         (INS_FLASH_LOG_BYTES / INS_FLASH_RECORD_SIZE)
#define INS_FLASH_RECORD_MAGIC         (0x4953U) /* "IS" */

typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint16_t flags;
    uint32_t seq;
    uint32_t tick_ms;
    int32_t x_mm;
    int32_t y_mm;
    int16_t yaw_cdeg;
    int16_t v_mms;
    int16_t w_cdegps;
    int16_t left_delta;
    int16_t right_delta;
    int16_t yaw_source;
} INS_LogRecord_t;

typedef struct {
    bool ready;
    bool enabled;
    bool full;
    uint32_t nextAddr;
    uint32_t seq;
    DevFlash *flash;
} INS_FlashLog_t;

static float wrap_180(float deg)
{
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static int16_t clamp_i16_from_float(float v)
{
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

static int16_t clamp_i16_from_i32(int32_t v)
{
    if (v > 32767) return 32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static float counts_to_m(int32_t counts, float countsPerRev, float sign)
{
    return sign * ((float)counts / countsPerRev) * INS_WHEEL_CIRCUMFERENCE_M;
}

static void update_flags(INS_Pose_t *pose,
                         bool imuValid,
                         bool yawZeroReady,
                         const INS_FlashLog_t *log)
{
    uint32_t flags = 0;

    if (imuValid) flags |= INS_FLAG_IMU_VALID;
    if (yawZeroReady) flags |= INS_FLAG_YAW_ZERO_READY;
    if (log->ready) flags |= INS_FLAG_FLASH_READY;
    if (log->full) flags |= INS_FLAG_FLASH_FULL;
    if (log->enabled) flags |= INS_FLAG_LOG_ENABLED;

    pose->flags = flags;
}

static bool flash_log_probe(INS_FlashLog_t *log)
{
    DevFlash_JEDECID_t id;

    if (log->flash == NULL) {
        log->flash = GetFlash();
    }

    if (log->flash == NULL) {
        LOG_RAW("[INS_CMD] error: flash handle NULL\r\n");
        return false;
    }

    log->flash->init(log->flash);
    if (!log->flash->readJEDECID(log->flash, &id) ||
        id.manufacturer != 0xEF || id.memoryType != 0x40) {
        LOG_RAW("[INS_CMD] error: flash JEDEC invalid\r\n");
        return false;
    }

    log->ready = true;
    return true;
}

static bool flash_log_init(INS_FlashLog_t *log)
{
    log->ready = false;
    log->enabled = false;
    log->full = false;
    log->nextAddr = INS_FLASH_LOG_START_ADDR;
    log->seq = 0;

    if (!flash_log_probe(log)) {
        return false;
    }

    for (uint32_t i = 0; i < INS_FLASH_LOG_SECTOR_COUNT; i++) {
        if (!log->flash->sectorErase(log->flash,
            INS_FLASH_LOG_START_ADDR + (i * INS_FLASH_LOG_SECTOR_SIZE))) {
            LOG_RAW("[INS_CMD] error: flash erase failed at sector %lu\r\n",
                    (unsigned long)i);
            return false;
        }
    }

    log->ready = true;
    log->enabled = true;
    LOG_RAW("[INS_CMD] log on ok start=0x%08lX bytes=%lu\r\n",
            (unsigned long)INS_FLASH_LOG_START_ADDR,
            (unsigned long)INS_FLASH_LOG_BYTES);
    return true;
}

static void flash_log_print(INS_FlashLog_t *log)
{
    INS_LogRecord_t rec;
    uint32_t printed = 0;

    if (!flash_log_probe(log)) {
        return;
    }

    LOG_RAW("[INS_LOG] begin start=0x%08lX max=%lu\r\n",
            (unsigned long)INS_FLASH_LOG_START_ADDR,
            (unsigned long)INS_FLASH_RECORD_COUNT);

    for (uint32_t i = 0; i < INS_FLASH_RECORD_COUNT; i++) {
        uint32_t addr = INS_FLASH_LOG_START_ADDR +
                        (i * (uint32_t)INS_FLASH_RECORD_SIZE);

        if (!log->flash->read(log->flash, addr,
                              (uint8_t *)&rec, sizeof(rec))) {
            LOG_RAW("[INS_LOG] read error index=%lu addr=0x%08lX\r\n",
                    (unsigned long)i, (unsigned long)addr);
            break;
        }

        if (rec.magic != INS_FLASH_RECORD_MAGIC) {
            break;
        }

        LOG_RAW("[INS_LOG] #%04lu boot_ms=%lu X=%+.3f Y=%+.3f YAW=%+.2f V=%+.3f W=%+.2f Ld=%d Rd=%d SRC=%c F=%u\r\n",
                (unsigned long)rec.seq,
                (unsigned long)rec.tick_ms,
                (double)rec.x_mm / 1000.0,
                (double)rec.y_mm / 1000.0,
                (double)rec.yaw_cdeg / 100.0,
                (double)rec.v_mms / 1000.0,
                (double)rec.w_cdegps / 100.0,
                (int)rec.left_delta,
                (int)rec.right_delta,
                (char)rec.yaw_source,
                (unsigned int)rec.flags);
        printed++;
    }

    if (printed == 0U) {
        LOG_RAW("[INS_LOG] empty\r\n");
    }

    LOG_RAW("[INS_LOG] end count=%lu\r\n", (unsigned long)printed);
}

static void flash_log_write(INS_FlashLog_t *log,
                            const INS_Pose_t *pose,
                            int32_t leftDelta,
                            int32_t rightDelta,
                            char yawSource)
{
    INS_LogRecord_t rec;

    if (!log->ready || !log->enabled || log->full || log->flash == NULL) {
        return;
    }

    if ((log->nextAddr + INS_FLASH_RECORD_SIZE) >
        (INS_FLASH_LOG_START_ADDR + INS_FLASH_LOG_BYTES)) {
        log->full = true;
        log->enabled = false;
        LOG_INFO("[INS] Flash log full\r\n");
        return;
    }

    rec.magic = INS_FLASH_RECORD_MAGIC;
    rec.flags = (uint16_t)pose->flags;
    rec.seq = log->seq++;
    rec.tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    rec.x_mm = (int32_t)(pose->x_m * 1000.0f);
    rec.y_mm = (int32_t)(pose->y_m * 1000.0f);
    rec.yaw_cdeg = clamp_i16_from_float(pose->yaw_deg * 100.0f);
    rec.v_mms = clamp_i16_from_float(pose->v_mps * 1000.0f);
    rec.w_cdegps = clamp_i16_from_float(pose->w_dps * 100.0f);
    rec.left_delta = clamp_i16_from_i32(leftDelta);
    rec.right_delta = clamp_i16_from_i32(rightDelta);
    rec.yaw_source = (int16_t)yawSource;

    log->flash->pageProgram(log->flash, log->nextAddr,
                            (const uint8_t *)&rec, sizeof(rec));
    log->nextAddr += INS_FLASH_RECORD_SIZE;
}

static void print_help(void)
{
    LOG_RAW("[INS_CMD] commands:\r\n");
    LOG_RAW("[INS_CMD]   ins help\r\n");
    LOG_RAW("[INS_CMD]   ins status\r\n");
    LOG_RAW("[INS_CMD]   ins reset\r\n");
    LOG_RAW("[INS_CMD]   ins log on\r\n");
    LOG_RAW("[INS_CMD]   ins log off\r\n");
    LOG_RAW("[INS_CMD]   ins log print\r\n");
}

static void print_status(const INS_Pose_t *pose,
                         bool ready,
                         const INS_FlashLog_t *log,
                         char yawSource)
{
    LOG_RAW("[INS_CMD] status ready=%u log=%u flash=%u full=%u seq=%lu addr=0x%08lX src=%c\r\n",
            ready ? 1U : 0U,
            log->enabled ? 1U : 0U,
            log->ready ? 1U : 0U,
            log->full ? 1U : 0U,
            (unsigned long)log->seq,
            (unsigned long)log->nextAddr,
            yawSource);
    LOG_RAW("[INS_CMD] pose X=%+.3f Y=%+.3f YAW=%+.2f V=%+.3f W=%+.2f L=%+.3f R=%+.3f F=%lu\r\n",
            pose->x_m, pose->y_m, pose->yaw_deg,
            pose->v_mps, pose->w_dps,
            pose->left_m, pose->right_m,
            (unsigned long)pose->flags);
}

static void handle_commands(INS_Pose_t *pose,
                            INS_FlashLog_t *log,
                            bool ready,
                            bool imuValid,
                            bool *yawZeroReady,
                            float *yawZero,
                            float latestRawYaw,
                            bool latestRawYawValid,
                            float *prevYaw,
                            bool *havePrevYaw,
                            int32_t *logAccumLeft,
                            int32_t *logAccumRight,
                            char yawSource)
{
    INS_Command_t cmd;

    while (g_insCmdQueue != NULL &&
           xQueueReceive(g_insCmdQueue, &cmd, 0) == pdTRUE) {
        switch (cmd.type) {
            case INS_CMD_HELP:
                print_help();
                break;

            case INS_CMD_STATUS:
                update_flags(pose, imuValid, *yawZeroReady, log);
                print_status(pose, ready, log, yawSource);
                break;

            case INS_CMD_RESET:
                if (!ready || !latestRawYawValid) {
                    LOG_RAW("[INS_CMD] error: not ready\r\n");
                    break;
                }
                *yawZero = latestRawYaw;
                *yawZeroReady = true;
                *prevYaw = 0.0f;
                *havePrevYaw = true;
                *logAccumLeft = 0;
                *logAccumRight = 0;
                pose->x_m = 0.0f;
                pose->y_m = 0.0f;
                pose->yaw_deg = 0.0f;
                pose->v_mps = 0.0f;
                pose->w_dps = 0.0f;
                pose->left_m = 0.0f;
                pose->right_m = 0.0f;
                update_flags(pose, true, true, log);
                LOG_RAW("[INS_CMD] reset ok\r\n");
                break;

            case INS_CMD_LOG_ON:
                if (!ready) {
                    LOG_RAW("[INS_CMD] error: not ready\r\n");
                    break;
                }
                if (log->ready && !log->full) {
                    log->enabled = true;
                    LOG_RAW("[INS_CMD] log on ok\r\n");
                } else if (log->full) {
                    LOG_RAW("[INS_CMD] error: flash full\r\n");
                } else {
                    (void)flash_log_init(log);
                }
                update_flags(pose, imuValid, *yawZeroReady, log);
                break;

            case INS_CMD_LOG_OFF:
                log->enabled = false;
                update_flags(pose, imuValid, *yawZeroReady, log);
                LOG_RAW("[INS_CMD] log off ok\r\n");
                break;

            case INS_CMD_LOG_PRINT:
                flash_log_print(log);
                update_flags(pose, imuValid, *yawZeroReady, log);
                break;

            default:
                LOG_RAW("[INS_CMD] error: unknown command\r\n");
                break;
        }
    }
}

void ins_task(void *pvParameters)
{
    INS_Pose_t pose = {0};
    INS_FlashLog_t flashLog = {0};
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastPrint = lastWake;
    TickType_t lastWaitPrint = lastWake;
    TickType_t lastFlash = lastWake;
    bool yawZeroReady = false;
    bool havePrevYaw = false;
    bool insReady = false;
    bool latestRawYawValid = false;
    bool imuValid = false;
    float yawZero = 0.0f;
    float prevYaw = 0.0f;
    float latestRawYaw = 0.0f;
    int32_t logAccumLeft = 0;
    int32_t logAccumRight = 0;
    int64_t prevLeftTotal = 0;
    int64_t prevRightTotal = 0;
    uint32_t prevOdomSeq = 0;
    bool odomRefReady = false;
    char yawSource = 'W';

    (void)pvParameters;

    LOG_INFO("[INS] Init: wheel=48mm base=125mm M1L=1054 M2R=985\r\n");

    while (1) {
        MotorEncoderSnapshot_t odom;
        int32_t sumLeftCounts = 0;
        int32_t sumRightCounts = 0;
        int32_t frameCount = 0;
        float leftStep;
        float rightStep;
        float ds;
        float dt = (float)INS_TASK_PERIOD_MS / 1000.0f;
        float yawDeg = pose.yaw_deg;
        float yawForStep;

        if (!insReady) {
            IMU_Data_t imu;

            if (MotorEncoder_ReadSnapshot(&odom)) {
                prevLeftTotal = odom.left_total_counts;
                prevRightTotal = odom.right_total_counts;
                prevOdomSeq = odom.seq;
                odomRefReady = true;
            }

            imuValid = false;
            if (IMU_Data_Read(&imu)) {
                latestRawYaw = imu.yaw;
                latestRawYawValid = true;
                yawSource = imu.yawSource;
                yawZero = imu.yaw;
                yawZeroReady = true;
                havePrevYaw = true;
                prevYaw = 0.0f;
                pose = (INS_Pose_t){0};
                update_flags(&pose, true, true, &flashLog);
                insReady = true;
                lastPrint = xTaskGetTickCount();
                lastFlash = lastPrint;
                LOG_INFO("[INS] Ready: yaw zero=%+.2f src=%c\r\n",
                         yawZero, imu.yawSource);
            } else if ((xTaskGetTickCount() - lastWaitPrint) >=
                       pdMS_TO_TICKS(INS_WAIT_PRINT_PERIOD_MS)) {
                lastWaitPrint = xTaskGetTickCount();
                LOG_INFO("[INS] Waiting for IMU yaw before pose output\r\n");
            }

            handle_commands(&pose, &flashLog, insReady, imuValid,
                            &yawZeroReady, &yawZero, latestRawYaw,
                            latestRawYawValid, &prevYaw, &havePrevYaw,
                            &logAccumLeft, &logAccumRight, yawSource);

            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(INS_TASK_PERIOD_MS));
            continue;
        }

        if (MotorEncoder_ReadSnapshot(&odom)) {
            if (!odomRefReady) {
                prevLeftTotal = odom.left_total_counts;
                prevRightTotal = odom.right_total_counts;
                prevOdomSeq = odom.seq;
                odomRefReady = true;
            } else {
                sumLeftCounts = (int32_t)(odom.left_total_counts - prevLeftTotal);
                sumRightCounts = (int32_t)(odom.right_total_counts - prevRightTotal);
                frameCount = (int32_t)(odom.seq - prevOdomSeq);
                prevLeftTotal = odom.left_total_counts;
                prevRightTotal = odom.right_total_counts;
                prevOdomSeq = odom.seq;
            }
        }

        if (frameCount > 0) {
            dt = ((float)frameCount * (float)MOTOR_ENC_PERIOD_MS) / 1000.0f;
        }
        logAccumLeft += sumLeftCounts;
        logAccumRight += sumRightCounts;

        leftStep = counts_to_m(sumLeftCounts,
                               (float)MOTOR1_COUNTS_PER_OUTPUT_REV_CAL,
                               INS_LEFT_SIGN);
        rightStep = counts_to_m(sumRightCounts,
                                (float)MOTOR2_COUNTS_PER_OUTPUT_REV_CAL,
                                INS_RIGHT_SIGN);
        ds = (leftStep + rightStep) * 0.5f;

        {
            IMU_Data_t imu;
            imuValid = false;
            if (IMU_Data_Read(&imu)) {
                latestRawYaw = imu.yaw;
                latestRawYawValid = true;
                yawSource = imu.yawSource;
                yawDeg = wrap_180((imu.yaw - yawZero) * INS_IMU_YAW_SIGN);
                imuValid = true;
            } else if (frameCount > 0) {
                float dyawWheelRad = (rightStep - leftStep) / INS_WHEEL_BASE_M;
                yawDeg = wrap_180(pose.yaw_deg + dyawWheelRad * (180.0f / INS_PI_F));
                yawSource = 'W';
            }
        }

        yawForStep = pose.yaw_deg + wrap_180(yawDeg - pose.yaw_deg) * 0.5f;
        pose.x_m += ds * cosf(yawForStep * (INS_PI_F / 180.0f));
        pose.y_m += ds * sinf(yawForStep * (INS_PI_F / 180.0f));
        pose.left_m += leftStep;
        pose.right_m += rightStep;
        pose.yaw_deg = yawDeg;
        pose.v_mps = (dt > 0.0f) ? (ds / dt) : 0.0f;

        if (havePrevYaw && dt > 0.0f) {
            pose.w_dps = wrap_180(yawDeg - prevYaw) / dt;
        } else {
            pose.w_dps = 0.0f;
            havePrevYaw = true;
        }
        prevYaw = yawDeg;

        update_flags(&pose, imuValid, yawZeroReady, &flashLog);

        handle_commands(&pose, &flashLog, insReady, imuValid,
                        &yawZeroReady, &yawZero, latestRawYaw,
                        latestRawYawValid, &prevYaw, &havePrevYaw,
                        &logAccumLeft, &logAccumRight, yawSource);

        INS_Pose_Write(&pose);

        if ((xTaskGetTickCount() - lastPrint) >= pdMS_TO_TICKS(INS_PRINT_PERIOD_MS)) {
            lastPrint = xTaskGetTickCount();
#if LOG_PRINT_INS_ENABLE
            LOG_RAW("[INS] X:%+.3f Y:%+.3f YAW:%+.2f V:%+.3f W:%+.2f L:%+.3f R:%+.3f SRC:%c F:%lu\r\n",
                    pose.x_m, pose.y_m, pose.yaw_deg,
                    pose.v_mps, pose.w_dps,
                    pose.left_m, pose.right_m,
                    yawSource, (unsigned long)pose.flags);
#endif
        }

        if ((xTaskGetTickCount() - lastFlash) >= pdMS_TO_TICKS(INS_FLASH_LOG_PERIOD_MS)) {
            lastFlash = xTaskGetTickCount();
            flash_log_write(&flashLog, &pose,
                            logAccumLeft,
                            logAccumRight,
                            yawSource);
            logAccumLeft = 0;
            logAccumRight = 0;
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(INS_TASK_PERIOD_MS));
    }
}
