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
#include "port_log.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

QueueHandle_t g_insPoseQueue = NULL;

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
    bool full;
    uint32_t nextAddr;
    uint32_t seq;
    TickType_t lastWrite;
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

static float counts_to_m(int32_t counts, float countsPerRev, float sign)
{
    return sign * ((float)counts / countsPerRev) * INS_WHEEL_CIRCUMFERENCE_M;
}

static void flash_log_init(INS_FlashLog_t *log)
{
    DevFlash_JEDECID_t id;

    log->ready = false;
    log->full = false;
    log->nextAddr = INS_FLASH_LOG_START_ADDR;
    log->seq = 0;
    log->lastWrite = xTaskGetTickCount();
    log->flash = GetFlash();

    if (log->flash == NULL) {
        LOG_ERROR("[INS] Flash handle NULL; logging disabled\r\n");
        return;
    }

    log->flash->init(log->flash);
    if (!log->flash->readJEDECID(log->flash, &id) ||
        id.manufacturer != 0xEF || id.memoryType != 0x40) {
        LOG_ERROR("[INS] Flash JEDEC invalid; logging disabled\r\n");
        return;
    }

    for (uint32_t i = 0; i < INS_FLASH_LOG_SECTOR_COUNT; i++) {
        log->flash->sectorErase(log->flash,
            INS_FLASH_LOG_START_ADDR + (i * INS_FLASH_LOG_SECTOR_SIZE));
    }

    log->ready = true;
    LOG_INFO("[INS] Flash log ready: start=0x%08lX bytes=%lu\r\n",
             (unsigned long)INS_FLASH_LOG_START_ADDR,
             (unsigned long)INS_FLASH_LOG_BYTES);
}

static void flash_log_write(INS_FlashLog_t *log,
                            const INS_Pose_t *pose,
                            int32_t leftDelta,
                            int32_t rightDelta,
                            char yawSource)
{
    INS_LogRecord_t rec;

    if (!log->ready || log->full || log->flash == NULL) {
        return;
    }

    if ((log->nextAddr + INS_FLASH_RECORD_SIZE) >
        (INS_FLASH_LOG_START_ADDR + INS_FLASH_LOG_BYTES)) {
        log->full = true;
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
    rec.left_delta = (int16_t)leftDelta;
    rec.right_delta = (int16_t)rightDelta;
    rec.yaw_source = (int16_t)yawSource;

    log->flash->pageProgram(log->flash, log->nextAddr,
                            (const uint8_t *)&rec, sizeof(rec));
    log->nextAddr += INS_FLASH_RECORD_SIZE;
}

void ins_task(void *pvParameters)
{
    (void)pvParameters;

    INS_Pose_t pose = {0};
    INS_FlashLog_t flashLog = {0};
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t startTick = lastWake;
    TickType_t lastPrint = lastWake;
    TickType_t lastWaitPrint = lastWake;
    TickType_t lastFlash = lastWake;
    bool yawZeroReady = false;
    bool havePrevYaw = false;
    bool flashInitDone = false;
    bool insReady = false;
    float yawZero = 0.0f;
    float prevYaw = 0.0f;
    int32_t logAccumLeft = 0;
    int32_t logAccumRight = 0;

    LOG_INFO("[INS] Init: wheel=48mm base=125mm M1L=1054 M2R=985\r\n");

    while (1) {
        MotorOdomDelta_t odom;
        int32_t sumLeftCounts = 0;
        int32_t sumRightCounts = 0;
        int32_t frameCount = 0;
        float leftStep = 0.0f;
        float rightStep = 0.0f;
        float ds = 0.0f;
        float dt = (float)INS_TASK_PERIOD_MS / 1000.0f;
        float yawDeg = pose.yaw_deg;
        float yawForStep;
        char yawSource = 'W';

        if (!insReady) {
            IMU_Data_t imu;

            while (g_motorOdomQueue != NULL &&
                   xQueueReceive(g_motorOdomQueue, &odom, 0) == pdTRUE) {
                /* Discard motion before the yaw zero reference is ready. */
            }

            if (g_imuDataQueue != NULL &&
                xQueuePeek(g_imuDataQueue, &imu, 0) == pdTRUE) {
                yawZero = imu.yaw;
                yawZeroReady = true;
                havePrevYaw = true;
                prevYaw = 0.0f;
                pose = (INS_Pose_t){0};
                pose.flags = INS_FLAG_IMU_VALID | INS_FLAG_YAW_ZERO_READY;
                startTick = xTaskGetTickCount();
                lastPrint = startTick;
                lastFlash = startTick;
                logAccumLeft = 0;
                logAccumRight = 0;
                insReady = true;
                LOG_INFO("[INS] Ready: yaw zero=%+.2f src=%c\r\n",
                         yawZero, imu.yawSource);
            } else if ((xTaskGetTickCount() - lastWaitPrint) >=
                       pdMS_TO_TICKS(INS_WAIT_PRINT_PERIOD_MS)) {
                lastWaitPrint = xTaskGetTickCount();
                LOG_INFO("[INS] Waiting for IMU yaw before pose output\r\n");
            }

            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(INS_TASK_PERIOD_MS));
            continue;
        }

        while (g_motorOdomQueue != NULL &&
               xQueueReceive(g_motorOdomQueue, &odom, 0) == pdTRUE) {
            sumLeftCounts += odom.left_counts;
            sumRightCounts += odom.right_counts;
            frameCount++;
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
            if (g_imuDataQueue != NULL &&
                xQueuePeek(g_imuDataQueue, &imu, 0) == pdTRUE) {
                if (!yawZeroReady) {
                    yawZero = imu.yaw;
                    yawZeroReady = true;
                    prevYaw = 0.0f;
                    havePrevYaw = true;
                }
                yawDeg = wrap_180((imu.yaw - yawZero) * INS_IMU_YAW_SIGN);
                yawSource = imu.yawSource;
                pose.flags |= INS_FLAG_IMU_VALID | INS_FLAG_YAW_ZERO_READY;
            } else if (frameCount > 0) {
                float dyawWheelRad = (rightStep - leftStep) / INS_WHEEL_BASE_M;
                yawDeg = wrap_180(pose.yaw_deg + dyawWheelRad * (180.0f / INS_PI_F));
                pose.flags &= ~(uint32_t)INS_FLAG_IMU_VALID;
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

        if (flashLog.ready) {
            pose.flags |= INS_FLAG_FLASH_READY;
        }
        if (flashLog.full) {
            pose.flags |= INS_FLAG_FLASH_FULL;
        }

        if (g_insPoseQueue != NULL) {
            xQueueOverwrite(g_insPoseQueue, &pose);
        }

        if (!flashInitDone &&
            (xTaskGetTickCount() - startTick) >= pdMS_TO_TICKS(2000)) {
            flashInitDone = true;
            flash_log_init(&flashLog);
        }

        if ((xTaskGetTickCount() - lastPrint) >= pdMS_TO_TICKS(INS_PRINT_PERIOD_MS)) {
            lastPrint = xTaskGetTickCount();
            LOG_RAW("[INS] X:%+.3f Y:%+.3f YAW:%+.2f V:%+.3f W:%+.2f L:%+.3f R:%+.3f SRC:%c F:%lu\r\n",
                    pose.x_m, pose.y_m, pose.yaw_deg,
                    pose.v_mps, pose.w_dps,
                    pose.left_m, pose.right_m,
                    yawSource, (unsigned long)pose.flags);
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
