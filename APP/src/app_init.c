/**
 * @file    app_init.c
 * @brief   App 层初始化与启动实现
 * @note    四层解耦架构 - App 层
 *          include 链: dev_led.h / port_log.h / FreeRTOS.h
 *          禁止: ti_msp_dl_config.h / DL_GPIO_* / 寄存器操作
 */
 
#include "app_init.h"
#include "app_menu.h"
#include "app_button.h"
#include "app_tb6612.h"
#include "app_imu.h"
#include "app_motor_encoder.h"
#include "app_ins.h"
#include "app_ins_cmd.h"
#include "app_motion.h"
#include "dev_led.h"
#include "dev_encoder.h"
#include "port_log.h"
#include "port_imu.h"
#include "port_lis3mdl.h"
#include "app_imu.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
 #include "freertosconfig.h"
/* 编码器→菜单事件队列定义（声明见 app_menu.h） */
QueueHandle_t g_menuEvtQueue = NULL;
 
/* 外部引用的应用任务 */
// extern void led_task(void *pvParameters);
extern void encoder_task(void *pvParameters);
extern void flash_init_task(void *pvParameters);
extern void tft_task(void *pvParameters);
extern void tb6612_task(void *pvParameters);
extern void motor_encoder_task(void *pvParameters);
extern void ins_task(void *pvParameters);
extern void ins_cmd_task(void *pvParameters);
extern void motion_task(void *pvParameters);
 
/* 任务句柄 */
static TaskHandle_t s_startTaskHandle;
// static TaskHandle_t s_ledTaskHandle;
static TaskHandle_t s_encoderTaskHandle;
// static TaskHandle_t s_buttonTaskHandle;
 
/* 本地函数 */
static void start_task(void *pvParameters);
 
/* ============ app_init / app_start ============ */
 
void app_init(void)
{
    xTaskCreate(
        start_task,
        "start_task",
        512,
        NULL,
        1,
        &s_startTaskHandle
    );
}
 
void app_start(void)
{
    vTaskStartScheduler();
}
 
/* ============ start_task ============ */
 
static void start_task(void *pvParameters)
{
    (void)pvParameters;

    setbuf(stdout, NULL);
 
    taskENTER_CRITICAL();
 
    /* 先创建编码器→菜单事件队列（必须在 encoder_task / tft_task 之前） */
    g_menuEvtQueue = xQueueCreate(MENU_EVT_QUEUE_LEN, sizeof(DevEncoder_Event_t));
    if (g_menuEvtQueue == NULL) {
        LOG_ERROR("[INIT] menu event queue create failed!\r\n");
    }
 
    /* 创建菜单→电机命令队列（必须在 tft_task / tb6612_task 之前） */
    g_motorCmdQueue = xQueueCreate(MOTOR_CMD_QUEUE_LEN, sizeof(MotorCmd));
    if (g_motorCmdQueue == NULL) {
        LOG_ERROR("[INIT] motor cmd queue create failed!\r\n");
    }

    g_motorOdomQueue = xQueueCreate(MOTOR_ODOM_QUEUE_LEN, sizeof(MotorOdomDelta_t));
    if (g_motorOdomQueue == NULL) {
        LOG_ERROR("[INIT] motor odom queue create failed!\r\n");
    }

    g_imuDataQueue = xQueueCreate(IMU_DATA_QUEUE_LEN, sizeof(IMU_Data_t));
    if (g_imuDataQueue == NULL) {
        LOG_ERROR("[INIT] IMU data queue create failed!\r\n");
    }

    g_insPoseQueue = xQueueCreate(INS_POSE_QUEUE_LEN, sizeof(INS_Pose_t));
    if (g_insPoseQueue == NULL) {
        LOG_ERROR("[INIT] INS pose queue create failed!\r\n");
    }

    g_insCmdQueue = xQueueCreate(INS_CMD_QUEUE_LEN, sizeof(INS_Command_t));
    if (g_insCmdQueue == NULL) {
        LOG_ERROR("[INIT] INS cmd queue create failed!\r\n");
    }

    g_motionCmdQueue = xQueueCreate(MOTION_CMD_QUEUE_LEN, sizeof(Motion_Command_t));
    if (g_motionCmdQueue == NULL) {
        LOG_ERROR("[INIT] motion cmd queue create failed!\r\n");
    }
 
    /* 创建编码器应用任务（内部包含 5ms 周期轮询） */
    xTaskCreate(
        encoder_task,
        "encoder_task",
        128,
        NULL,
        2,
        &s_encoderTaskHandle
    );

    /* 创建 TFT 显示任务 */
    xTaskCreate(
        tft_task,
        "tft_task",
        256,
        NULL,
        2,
        NULL
    );
    LOG_INFO("  TFT task created (prio=2)\r\n");
 
    /* 创建 TB6612 电机控制任务 */
    xTaskCreate(
        tb6612_task,
        "tb6612",
        128,
        NULL,
        2,
        NULL
    );
    LOG_INFO("  TB6612 task created (prio=2)\r\n");
 
    /* 创建电机编码器采集任务 (每 10ms) */
    xTaskCreate(
        motor_encoder_task,
        "motor_enc",
        256,
        NULL,
        2,
        NULL
    );
    LOG_INFO("  Motor encoder task created (prio=2)\r\n");

    /* 创建 IMU 数据采集任务 (ICM-20608 + LIS3MDLRT, 每 100ms) */
    app_imu_start();

    BaseType_t insCreateOk = xTaskCreate(
        ins_task,
        "ins",
        512,
        NULL,
        2,
        NULL
    );
    if (insCreateOk != pdPASS) {
        LOG_ERROR("[INIT] INS task create failed!\r\n");
    } else {
        LOG_INFO("  INS task created (prio=2)\r\n");
    }

    BaseType_t insCmdCreateOk = xTaskCreate(
        ins_cmd_task,
        "ins_cmd",
        256,
        NULL,
        1,
        NULL
    );
    if (insCmdCreateOk != pdPASS) {
        LOG_ERROR("[INIT] INS cmd task create failed!\r\n");
    } else {
        LOG_INFO("  INS cmd task created (prio=1)\r\n");
    }

    BaseType_t motionCreateOk = xTaskCreate(
        motion_task,
        "motion",
        384,
        NULL,
        2,
        NULL
    );
    if (motionCreateOk != pdPASS) {
        LOG_ERROR("[INIT] motion task create failed!\r\n");
    } else {
        LOG_INFO("  Motion task created (prio=2)\r\n");
    }

    /* 创建 Flash 开机自检任务 (W25Q64, 最低优先级, 完成后自动删除) */
    xTaskCreate(
        flash_init_task,
        "flash_test",
        256,
        NULL,
        1,
        NULL
    );
    LOG_INFO("  Flash test task created (prio=1)\r\n");

    LOG_INFO("====================================\r\n");
 
    taskEXIT_CRITICAL();
 
    /* 删除开始任务自身（必须在临界区之外，否则调度器异常） */
    vTaskDelete(NULL);
}
