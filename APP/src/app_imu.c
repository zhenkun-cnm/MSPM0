/**
 * @file    app_imu.c
 * @brief   App layer 9-axis attitude task.
 */
#include "app_imu.h"
#include "app_stack_monitor.h"
#include "imu_ahrs9.h"
#include "imu_filter.h"
#include "port_imu.h"
#include "port_lis3mdl.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <math.h>

/* I2C initialization must retain the known-good compiler behavior. */
#pragma clang optimize off

#define IMU_TASK_STACK_SIZE      448
#define IMU_TASK_PRIORITY        (4)
#define IMU_TASK_PERIOD_MS       10
#define IMU_PRINT_PERIOD_MS      1000

#define ACCEL_SENSITIVITY        2048.0f
#define GYRO_SENSITIVITY         16.4f
#define GYRO_CALIB_MS            4000
#define MAG2D_CALIB_MS           5000

#define ACCEL_LP_ALPHA           0.35f
#define GYRO_DEADBAND_DPS        0.05f
#define GYRO_STILL_DPS           0.35f
#define GYRO_BIAS_ADAPT_ALPHA    0.002f
#define ACCEL_STILL_LOW_G        0.92f
#define ACCEL_STILL_HIGH_G       1.08f

#define MAG2D_MIN_HALF_RANGE     120.0f
#define MAG2D_GATE_LOW           0.65f
#define MAG2D_GATE_HIGH          1.45f
#define MAG2D_MAX_TILT_DEG       25.0f

IMU_DataGlobal_t g_imuDataGlobal = {{0.0f, 0.0f, 0.0f, 0.0f, false, 'G'}, NULL};
static volatile IMU_InitStatus_t s_imuInitStatus = IMU_INIT_PENDING;

IMU_InitStatus_t IMU_InitStatus_Get(void)
{
    return s_imuInitStatus;
}

bool IMU_Data_Read(IMU_Data_t *out)
{
    if (out == NULL || g_imuDataGlobal.lock == NULL) return false;
    if (xSemaphoreTake(g_imuDataGlobal.lock, pdMS_TO_TICKS(1)) != pdTRUE) return false;
    *out = g_imuDataGlobal.data;
    xSemaphoreGive(g_imuDataGlobal.lock);
    return true;
}

void IMU_Data_Write(const IMU_Data_t *in)
{
    if (in == NULL || g_imuDataGlobal.lock == NULL) return;
    if (xSemaphoreTake(g_imuDataGlobal.lock, pdMS_TO_TICKS(2)) != pdTRUE) return;
    g_imuDataGlobal.data = *in;
    xSemaphoreGive(g_imuDataGlobal.lock);
}

typedef struct {
    float gx;
    float gy;
    float gz;
} GyroBias;

typedef struct {
    float offset[2];
    float scale[2];
    float radius;
    float quality;
    bool valid;
} Mag2DCal;

static float abs_f(float v)
{
    return (v < 0.0f) ? -v : v;
}

static float apply_deadband(float v, float threshold)
{
    return (abs_f(v) < threshold) ? 0.0f : v;
}

static float clamp_f(float v, float low, float high)
{
    if (v < low) return low;
    if (v > high) return high;
    return v;
}

static void calibrate_gyro(GyroBias *bias)
{
    int32_t sx = 0, sy = 0, sz = 0;
    int samples = 0;
    int loops = GYRO_CALIB_MS / IMU_TASK_PERIOD_MS;

    bias->gx = 0.0f; bias->gy = 0.0f; bias->gz = 0.0f;
    LOGI_INIT(LOG_MOD_IMU, "Keep still: gyro calib %ds\r\n", GYRO_CALIB_MS / 1000);

    for (int i = 0; i < loops; i++) {
        int16_t gx, gy, gz;
        if (PORT_IMU_IsOk()) {
            if (PORT_IMU_ReadGyroRaw(&gx, &gy, &gz)) {
                sx += gx; sy += gy; sz += gz;
                samples++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }

    if (samples > 0) {
        bias->gx = ((float)sx / (float)samples) / GYRO_SENSITIVITY;
        bias->gy = ((float)sy / (float)samples) / GYRO_SENSITIVITY;
        bias->gz = ((float)sz / (float)samples) / GYRO_SENSITIVITY;
    }
    LOGI_INIT(LOG_MOD_IMU, "Gyro bias dps: %+.4f %+.4f %+.4f\r\n",
            bias->gx, bias->gy, bias->gz);
}

static void calibrate_mag2d(Mag2DCal *cal)
{
    int16_t mx, my, mz;
    float minv[2] = {32767.0f, 32767.0f};
    float maxv[2] = {-32768.0f, -32768.0f};
    int samples = 0;
    int loops = MAG2D_CALIB_MS / IMU_TASK_PERIOD_MS;

    cal->offset[0] = 0.0f; cal->offset[1] = 0.0f;
    cal->scale[0] = 1.0f; cal->scale[1] = 1.0f;
    cal->radius = 1.0f;
    cal->quality = 0.0f;
    cal->valid = false;

    for (int i = 0; i < loops; i++) {
        if (PORT_LIS3MDL_IsOk()) {
            PORT_LIS3MDL_ReadMagRaw(&mx, &my, &mz);
            if ((float)mx < minv[0]) minv[0] = (float)mx;
            if ((float)my < minv[1]) minv[1] = (float)my;
            if ((float)mx > maxv[0]) maxv[0] = (float)mx;
            if ((float)my > maxv[1]) maxv[1] = (float)my;
            samples++;
        }
        vTaskDelay(pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }

    if (samples > 0) {
        float range[2];
        float avg;
        float ratio;

        for (int axis = 0; axis < 2; axis++) {
            cal->offset[axis] = (maxv[axis] + minv[axis]) * 0.5f;
            range[axis] = (maxv[axis] - minv[axis]) * 0.5f;
        }

        avg = (range[0] + range[1]) * 0.5f;
        ratio = (range[0] > range[1]) ? (range[1] / range[0]) : (range[0] / range[1]);
        if (range[0] > MAG2D_MIN_HALF_RANGE &&
            range[1] > MAG2D_MIN_HALF_RANGE &&
            avg > MAG2D_MIN_HALF_RANGE) {
            cal->scale[0] = avg / range[0];
            cal->scale[1] = avg / range[1];
            cal->radius = avg;
            cal->quality = clamp_f(ratio * 100.0f, 0.0f, 100.0f);
            cal->valid = (ratio > 0.35f);
        }
    }

    LOGI_INIT(LOG_MOD_IMU,
            "MAG2D: offset=%+.1f/%+.1f scale=%.3f/%.3f radius=%.1f quality=%.0f %s\r\n",
            cal->offset[0], cal->offset[1], cal->scale[0], cal->scale[1],
            cal->radius, cal->quality, cal->valid ? "OK" : "INVALID");
    if (!cal->valid) {
        LOGW_RELIABLE(LOG_MOD_IMU, "MAG2D calibration invalid\r\n");
    }
}

static bool correct_mag2d(const Mag2DCal *cal, int16_t rx, int16_t ry,
                          float roll, float pitch, float *mx, float *my, float *mz)
{
    float x = ((float)rx - cal->offset[0]) * cal->scale[0];
    float y = ((float)ry - cal->offset[1]) * cal->scale[1];
    float norm = sqrtf(x * x + y * y);
    float ratio;

    if (!cal->valid || cal->radius <= 1.0f || norm <= 1.0f ||
        abs_f(roll) > MAG2D_MAX_TILT_DEG || abs_f(pitch) > MAG2D_MAX_TILT_DEG) {
        *mx = 0.0f; *my = 0.0f; *mz = 0.0f;
        return false;
    }

    ratio = norm / cal->radius;
    if (ratio < MAG2D_GATE_LOW || ratio > MAG2D_GATE_HIGH) {
        *mx = 0.0f; *my = 0.0f; *mz = 0.0f;
        return false;
    }

    *mx = x;
    *my = y;
    *mz = 0.0f;
    return true;
}

static bool update_still_bias(GyroBias *bias, float rawGxDps, float rawGyDps, float rawGzDps,
                              float ax, float ay, float az)
{
    float cgx = rawGxDps - bias->gx;
    float cgy = rawGyDps - bias->gy;
    float cgz = rawGzDps - bias->gz;
    float gyroNorm = sqrtf(cgx * cgx + cgy * cgy + cgz * cgz);
    float accNorm = sqrtf(ax * ax + ay * ay + az * az);

    if (gyroNorm < GYRO_STILL_DPS &&
        accNorm > ACCEL_STILL_LOW_G &&
        accNorm < ACCEL_STILL_HIGH_G) {
        bias->gx += (rawGxDps - bias->gx) * GYRO_BIAS_ADAPT_ALPHA;
        bias->gy += (rawGyDps - bias->gy) * GYRO_BIAS_ADAPT_ALPHA;
        bias->gz += (rawGzDps - bias->gz) * GYRO_BIAS_ADAPT_ALPHA;
        return true;
    }

    return false;
}

static void imu_task(void *arg)
{
    GyroBias gyroBias;
    Mag2DCal magCal;
    Ahrs9Config cfg = {1.5f, 0.025f, 0.010f};
    float axf = 0.0f, ayf = 0.0f, azf = 1.0f;
    TickType_t lastWake;
    TickType_t lastPrint;
    (void)arg;

    PORT_IMU_Init();
    PORT_LIS3MDL_Init();

    if (!PORT_IMU_IsOk()) {
        LOGE(LOG_MOD_IMU, "ICM not ready; attitude task stopped\r\n");
        s_imuInitStatus = IMU_INIT_FAILED;
        vTaskDelete(NULL);
        return;
    }

    calibrate_gyro(&gyroBias);
    calibrate_mag2d(&magCal);
    ahrs9_init(&cfg);
    ahrs9_reset_yaw();
    s_imuInitStatus = IMU_INIT_READY;
    LOGI_INIT(LOG_MOD_IMU, "IMU startup calibration complete\r\n");
    lastWake = xTaskGetTickCount();
    lastPrint = lastWake;

    while (1) {
        int16_t ax, ay, az, gx, gy, gz, mx, my, mz;
        float rawGxDps, rawGyDps, rawGzDps;
        float gxDps, gyDps, gzDps;
        float mxCal = 0.0f, myCal = 0.0f, mzCal = 0.0f;
        bool magUsed = false;
        bool stillLock = false;
        float rollPrev = 0.0f, pitchPrev = 0.0f;

        bool accelValid = PORT_IMU_ReadAccelRaw(&ax, &ay, &az);
        bool gyroValid = PORT_IMU_ReadGyroRaw(&gx, &gy, &gz);

        if (!accelValid || !gyroValid) {
            IMU_Data_t data;
            ahrs9_get_euler(&data.roll, &data.pitch, &data.yaw);
            data.gyro_z_dps = 0.0f;
            data.gyro_z_valid = false;
            data.yawSource = 'G';
            IMU_Data_Write(&data);
            vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
            continue;
        }

        axf = imu_lowpass((float)ax / ACCEL_SENSITIVITY, axf, ACCEL_LP_ALPHA);
        ayf = imu_lowpass((float)ay / ACCEL_SENSITIVITY, ayf, ACCEL_LP_ALPHA);
        azf = imu_lowpass((float)az / ACCEL_SENSITIVITY, azf, ACCEL_LP_ALPHA);

        rawGxDps = (float)gx / GYRO_SENSITIVITY;
        rawGyDps = (float)gy / GYRO_SENSITIVITY;
        rawGzDps = (float)gz / GYRO_SENSITIVITY;

        stillLock = update_still_bias(&gyroBias, rawGxDps, rawGyDps, rawGzDps,
                                      axf, ayf, azf);

        gxDps = rawGxDps - gyroBias.gx;
        gyDps = rawGyDps - gyroBias.gy;
        gzDps = rawGzDps - gyroBias.gz;

        gxDps = apply_deadband(gxDps, GYRO_DEADBAND_DPS);
        gyDps = apply_deadband(gyDps, GYRO_DEADBAND_DPS);
        gzDps = apply_deadband(gzDps, GYRO_DEADBAND_DPS);

        if (PORT_LIS3MDL_IsOk()) {
            PORT_LIS3MDL_ReadMagRaw(&mx, &my, &mz);
            ahrs9_get_euler(&rollPrev, &pitchPrev, NULL);
            magUsed = correct_mag2d(&magCal, mx, my, rollPrev, pitchPrev,
                                    &mxCal, &myCal, &mzCal);
        }

        ahrs9_update(gxDps, gyDps, gzDps,
                     axf, ayf, azf,
                     mxCal, myCal, mzCal,
                     (float)IMU_TASK_PERIOD_MS / 1000.0f);

        {
            IMU_Data_t data;
            ahrs9_get_euler(&data.roll, &data.pitch, &data.yaw);
            data.gyro_z_dps = gzDps;
            data.gyro_z_valid = true;
            data.yawSource = magUsed ? 'M' : (stillLock ? 'B' : 'G');
            IMU_Data_Write(&data);
        }

        if ((xTaskGetTickCount() - lastPrint) >= pdMS_TO_TICKS(IMU_PRINT_PERIOD_MS)) {
            float roll, pitch, yaw;
            char yawSource = magUsed ? 'M' : (stillLock ? 'B' : 'G');
            lastPrint = xTaskGetTickCount();
            ahrs9_get_euler(&roll, &pitch, &yaw);
            LOGD(LOG_MOD_IMU, "R:%+7.2f P:%+7.2f Y:%+7.2f YSRC:%c\r\n",
                    roll, pitch, yaw, yawSource);
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}

void app_imu_start(void)
{
    s_imuInitStatus = IMU_INIT_PENDING;
    g_imuDataGlobal.lock = xSemaphoreCreateMutex();
    if (g_imuDataGlobal.lock == NULL) {
        LOGE(LOG_MOD_IMU, "IMU mutex create failed\r\n");
        s_imuInitStatus = IMU_INIT_FAILED;
        return;
    }
    TaskHandle_t imuHandle = NULL;
    BaseType_t status = xTaskCreate(imu_task, "imu_task",
                                      IMU_TASK_STACK_SIZE, NULL,
                                      IMU_TASK_PRIORITY, &imuHandle);
    if (status == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_IMU,
                                   imuHandle,
                                   IMU_TASK_STACK_SIZE);
    } else {
        LOGE(LOG_MOD_IMU, "IMU task create failed\r\n");
        s_imuInitStatus = IMU_INIT_FAILED;
    }
}
