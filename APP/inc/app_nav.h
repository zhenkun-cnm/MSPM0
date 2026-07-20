/**
 * @file    app_nav.h
 * @brief   INS-based point navigation command layer.
 */
#ifndef APP_NAV_H
#define APP_NAV_H

#include "FreeRTOS.h"
#include "queue.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NAV_CMD_QUEUE_LEN 4U

typedef enum {
    NAV_STATE_IDLE = 0,              /**< 空闲/待命状态 */
    NAV_STATE_TURN_TO_TARGET,        /**< 转向目标方向 */
    NAV_STATE_DRIVE_TO_TARGET,       /**< 直行前往目标点 */
    NAV_STATE_TURN_TO_FINAL_YAW,     /**< 到达后调整最终航向 */
    NAV_STATE_SQUARE_DRIVE,          /**< 正方形直行段 */
    NAV_STATE_SQUARE_TURN,           /**< 正方形转弯段 */
    NAV_STATE_DONE,                  /**< 动作完成 */
    NAV_STATE_ERROR                  /**< 错误状态 */
} Nav_State_t;

typedef enum {
    NAV_CMD_HELP = 0,                /**< 打印帮助信息 */
    NAV_CMD_STATUS,                  /**< 查询导航状态 */
    NAV_CMD_STOP,                    /**< 停止当前导航任务 */
    NAV_CMD_GOTO,                    /**< goto(x, y, yaw) 点到点导航 */
    NAV_CMD_SQUARE                  /**< square(side) 走正方形 */
} Nav_CommandType_t;

typedef struct {
    Nav_CommandType_t type;          /**< 命令类型 */
    float x_m;                       /**< goto 目标 X 坐标，单位 m */
    float y_m;                       /**< goto 目标 Y 坐标，单位 m */
    float yaw_deg;                   /**< goto 目标最终航向角，单位 ° */
    float side_m;                    /**< square 边长，单位 m */
} Nav_Command_t;

extern QueueHandle_t g_navCmdQueue;

void nav_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_NAV_H */
