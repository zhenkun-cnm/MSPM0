/**
 * @file    app_init.c
 * @brief   App 层初始化与启动实现
 * @note    四层解耦架构 - App 层
 *          include 链: dev_led.h / port_log.h / FreeRTOS.h
 *          禁止: ti_msp_dl_config.h / DL_GPIO_* / 寄存器操作
 */

#include "app_init.h"
#include "app_menu.h"
#include "dev_led.h"
#include "dev_encoder.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* 编码器→菜单事件队列定义（声明见 app_menu.h） */
QueueHandle_t g_menuEvtQueue = NULL;

/* 外部引用的应用任务 */
extern void led_task(void *pvParameters);
extern void encoder_task(void *pvParameters);
extern void flash_init_task(void *pvParameters);
extern void imu_task(void *pvParameters);
extern void tft_task(void *pvParameters);

/* 任务句柄 */
static TaskHandle_t s_startTaskHandle;
static TaskHandle_t s_ledTaskHandle;
static TaskHandle_t s_encoderTaskHandle;

/* 本地函数 */
static void start_task(void *pvParameters);

/* ============ app_init / app_start ============ */

void app_init(void)
{
    xTaskCreate(
        start_task,
        "start_task",
        128,
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

    taskENTER_CRITICAL();

    /* 先创建编码器→菜单事件队列（必须在 encoder_task / tft_task 之前） */
    g_menuEvtQueue = xQueueCreate(MENU_EVT_QUEUE_LEN, sizeof(DevEncoder_Event_t));
    if (g_menuEvtQueue == NULL) {
        LOG_ERROR("[INIT] menu event queue create failed!\r\n");
    }

    /* 创建 LED 闪烁应用任务 */
    xTaskCreate(
        led_task,
        "led_task",
        128,
        NULL,
        2,
        &s_ledTaskHandle
    );

    /* 创建编码器应用任务（内部包含 5ms 周期轮询） */
    xTaskCreate(
        encoder_task,
        "encoder_task",
        256,
        NULL,
        2,
        &s_encoderTaskHandle
    );

    /* 创建 Flash 验证任务（读取 JEDEC ID 后删除自身） */
    xTaskCreate(
        flash_init_task,
        "flash_init",
        256,
        NULL,
        1,
        NULL
    );

    /* 创建 IMU 数据采集任务（每 1 秒打印一次 6 轴原始值） */
    xTaskCreate(
        imu_task,
        "imu_task",
        384,
        NULL,
        2,
        NULL
    );

    /* 系统启动信息 */
    LOG_INFO("====================================\r\n");
    LOG_INFO("  System Boot OK\r\n");
    LOG_INFO("  LED task created (prio=2)\r\n");
    LOG_INFO("  Encoder task created (prio=2)\r\n");
    LOG_INFO("  Flash init task created (prio=1)\r\n");
    LOG_INFO("  IMU task created (prio=2)\r\n");

    /* 创建 TFT 显示任务 */
    xTaskCreate(
        tft_task,
        "tft_task",
        512,
        NULL,
        2,
        NULL
    );
    LOG_INFO("  TFT task created (prio=2)\r\n");
    LOG_INFO("====================================\r\n");

    /* 删除开始任务自身 */
    vTaskDelete(NULL);

    taskEXIT_CRITICAL();
}