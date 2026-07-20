/**
 * @file    app_ins.h
 * @brief   App layer inertial/odometry pose estimator.
 */
#ifndef APP_INS_H
#define APP_INS_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INS_CMD_QUEUE_LEN   4U

typedef enum {
    INS_FLAG_IMU_VALID       = (1U << 0),  /**< IMU 数据有效标志 */
    INS_FLAG_YAW_ZERO_READY  = (1U << 1),  /**< 航向已归零（INS reset 后） */
    INS_FLAG_FLASH_READY     = (1U << 2),  /**< Flash 存储设备已就绪 */
    INS_FLAG_FLASH_FULL      = (1U << 3),  /**< Flash 记录空间已满 */
    INS_FLAG_LOG_ENABLED     = (1U << 4),  /**< INS 日志记录已开启 */
} INS_Flags_t;

typedef enum {
    INS_CMD_HELP = 0,          /**< 打印 INS 帮助信息 */
    INS_CMD_STATUS,            /**< 查询 INS 状态 */
    INS_CMD_RESET,             /**< 重置位姿（当前位置归零，yaw 归零） */
    INS_CMD_LOG_ON,            /**< 开启 Flash 日志记录 */
    INS_CMD_LOG_OFF,           /**< 关闭 Flash 日志记录 */
    INS_CMD_LOG_PRINT,         /**< 打印 Flash 中记录的日志 */
} INS_CommandType_t;

typedef struct {
    INS_CommandType_t type;
} INS_Command_t;

typedef struct {
    float x_m;                  /**< INS 估计的 X 坐标，单位 m */
    float y_m;                  /**< INS 估计的 Y 坐标，单位 m */
    float yaw_deg;              /**< 当前航向角（融合 IMU + 轮式里程计），单位 ° */
    float v_mps;                /**< 当前线速度，单位 m/s */
    float w_dps;                /**< 当前角速度，单位 °/s */
    float left_m;               /**< 左轮累计行驶距离，单位 m */
    float right_m;              /**< 右轮累计行驶距离，单位 m */
    uint32_t flags;             /**< 状态标志位（见 INS_Flags_t 枚举） */
} INS_Pose_t;

typedef struct {
    INS_Pose_t pose;            /**< INS 位姿数据 */
    SemaphoreHandle_t lock;     /**< 保护位姿数据的互斥锁 */
} INS_PoseGlobal_t;

extern INS_PoseGlobal_t g_insPoseGlobal;
extern QueueHandle_t g_insCmdQueue;

bool INS_Pose_Read(INS_Pose_t *out);
void INS_Pose_Write(const INS_Pose_t *in);

void ins_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_INS_H */
