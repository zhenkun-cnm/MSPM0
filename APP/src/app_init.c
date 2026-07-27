/**
 * @file    app_init.c
 * @brief   App layer initialization and task startup.
 */

#include "app_init.h"
#include "app_menu.h"
#include "app_button.h"
#include "app_tb6612.h"
#include "app_imu.h"
#include "app_motor_encoder.h"
#include "app_ins.h"
#include "app_ins_cmd.h"
#include "app_car_comm.h"
#include "app_motion.h"
#include "app_nav.h"
#include "app_path.h"
#include "app_gray.h"
#include "app_gray_line.h"
#include "app_drive_mode.h"
#include "app_stack_monitor.h"
#include "app_test1.h"
#include "dev_led.h"
#include "dev_encoder.h"
#include "port_log.h"
#include "port_imu.h"
#include "port_lis3mdl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "timers.h"
#include <stdio.h>

/* Keep startup and task creation on the known-good non-size-optimized path. */
#pragma clang optimize off

#define START_TASK_STACK_WORDS       256U
#define ENCODER_TASK_STACK_WORDS     140U
#define TFT_TASK_STACK_WORDS         256U
#define TB6612_TASK_STACK_WORDS      96U
#define MOTOR_ENC_TASK_STACK_WORDS   256U
#define INS_TASK_STACK_WORDS         288U
#define INS_CMD_TASK_STACK_WORDS     256U
#define CAR_COMM_TASK_STACK_WORDS    192U
#define MOTION_TASK_STACK_WORDS      192U
#define NAV_TASK_STACK_WORDS         160U
#define PATH_TASK_STACK_WORDS        384U
#define GRAY_TASK_STACK_WORDS        154U
#define GRAYLINE_TASK_STACK_WORDS    250U
#define DRIVE_MODE_TASK_STACK_WORDS  160U
#define FLASH_TASK_STACK_WORDS       192U
#define TEST1_TASK_STACK_WORDS       384U
#define DEFERRED_START_DELAY_MS       100U

QueueHandle_t g_menuEvtQueue = NULL;

extern void encoder_task(void *pvParameters);
extern void flash_init_task(void *pvParameters);
extern void tft_task(void *pvParameters);
extern void tb6612_task(void *pvParameters);
extern void motor_encoder_task(void *pvParameters);
extern void ins_task(void *pvParameters);
extern void ins_cmd_task(void *pvParameters);
extern void motion_task(void *pvParameters);
extern void nav_task(void *pvParameters);
extern void path_task(void *pvParameters);
extern void gray_task(void *pvParameters);
extern void grayline_task(void *pvParameters);
#if APP_TEST1_ENABLE
extern void test1_task(void *pvParameters);
#endif

static TaskHandle_t s_startTaskHandle = NULL;
static TaskHandle_t s_encoderTaskHandle = NULL;
static TaskHandle_t s_tftTaskHandle = NULL;
static TaskHandle_t s_tb6612TaskHandle = NULL;
static TaskHandle_t s_motorEncoderTaskHandle = NULL;
static TaskHandle_t s_insTaskHandle = NULL;
static TaskHandle_t s_insCmdTaskHandle = NULL;
static TaskHandle_t s_carCommTaskHandle = NULL;
static TaskHandle_t s_motionTaskHandle = NULL;
static TaskHandle_t s_navTaskHandle = NULL;
static TaskHandle_t s_pathTaskHandle = NULL;
static TaskHandle_t s_grayTaskHandle = NULL;
static TaskHandle_t s_grayLineTaskHandle = NULL;
static TaskHandle_t s_driveModeTaskHandle = NULL;
static TaskHandle_t s_flashTaskHandle = NULL;
static TimerHandle_t s_deferredStartTimer = NULL;
#if APP_TEST1_ENABLE
static TaskHandle_t s_test1TaskHandle = NULL;
#endif

static void start_task(void *pvParameters);
static void start_deferred_tasks(TimerHandle_t timer);
static void schedule_deferred_tasks(void);

void app_init(void)
{
    BaseType_t status = xTaskCreate(start_task,
                                    "start_task",
                                    START_TASK_STACK_WORDS,
                                    NULL,
                                    1,
                                    &s_startTaskHandle);
    if (status == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_START,
                                   s_startTaskHandle,
                                   START_TASK_STACK_WORDS);
    }
}

void app_start(void)
{
    vTaskStartScheduler();
}

static void start_task(void *pvParameters)
{
    (void)pvParameters;

    setbuf(stdout, NULL);

    taskENTER_CRITICAL();

    g_menuEvtQueue = xQueueCreate(MENU_EVT_QUEUE_LEN, sizeof(DevEncoder_Event_t));
    if (g_menuEvtQueue == NULL) {
        LOGE(LOG_MOD_SYS, "menu event queue create failed!\r\n");
    }

    g_motorCmdQueue = xQueueCreate(MOTOR_CMD_QUEUE_LEN, sizeof(MotorCmd));
    if (g_motorCmdQueue == NULL) {
        LOGE(LOG_MOD_SYS, "motor cmd queue create failed!\r\n");
    }

    g_insPoseGlobal.lock = xSemaphoreCreateMutex();
    if (g_insPoseGlobal.lock == NULL) {
        LOGE(LOG_MOD_SYS, "INS pose mutex create failed!\r\n");
    }

    g_insCmdQueue = xQueueCreate(INS_CMD_QUEUE_LEN, sizeof(INS_Command_t));
    if (g_insCmdQueue == NULL) {
        LOGE(LOG_MOD_SYS, "INS cmd queue create failed!\r\n");
    }

    g_motionCmdQueue = xQueueCreate(MOTION_CMD_QUEUE_LEN, sizeof(Motion_Command_t));
    if (g_motionCmdQueue == NULL) {
        LOGE(LOG_MOD_SYS, "motion cmd queue create failed!\r\n");
    }

    g_navCmdQueue = xQueueCreate(NAV_CMD_QUEUE_LEN, sizeof(Nav_Command_t));
    if (g_navCmdQueue == NULL) {
        LOGE(LOG_MOD_SYS, "nav cmd queue create failed!\r\n");
    }

    g_pathCmdQueue = xQueueCreate(PATH_CMD_QUEUE_LEN, sizeof(Path_Command_t));
    if (g_pathCmdQueue == NULL) {
        LOGE(LOG_MOD_SYS, "path cmd queue create failed!\r\n");
    }

    g_grayLineCmdQueue = xQueueCreate(GRAYLINE_CMD_QUEUE_LEN, sizeof(GrayLine_Command_t));
    if (g_grayLineCmdQueue == NULL) {
        LOGE(LOG_MOD_SYS, "grayline cmd queue create failed!\r\n");
    }

    g_driveModeCmdQueue = xQueueCreate(DRIVE_MODE_CMD_QUEUE_LEN, sizeof(DriveMode_Command_t));
    if (g_driveModeCmdQueue == NULL) {
        LOGE(LOG_MOD_SYS, "drive mode queue create failed!\r\n");
    }

    if (!CarComm_Init()) {
        LOGE(LOG_MOD_SYS, "car comm queue create failed!\r\n");
    }

#if APP_TEST1_ENABLE
    g_test1CmdQueue = xQueueCreate(TEST1_CMD_QUEUE_LEN, sizeof(Test1_Command_t));
    if (g_test1CmdQueue == NULL) {
        LOGE(LOG_MOD_SYS, "test1 cmd queue create failed!\r\n");
    }
#endif

    if (xTaskCreate(encoder_task,
                    "encoder_task",
                    ENCODER_TASK_STACK_WORDS,
                    NULL,
                    2,
                    &s_encoderTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_ENCODER,
                                   s_encoderTaskHandle,
                                   ENCODER_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "encoder task create failed!\r\n");
    }

    if (xTaskCreate(tft_task,
                    "tft_task",
                    TFT_TASK_STACK_WORDS,
                    NULL,
                    1,
                    &s_tftTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_TFT,
                                   s_tftTaskHandle,
                                   TFT_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "TFT task create failed!\r\n");
    }

    if (xTaskCreate(tb6612_task,
                    "tb6612",
                    TB6612_TASK_STACK_WORDS,
                    NULL,
                    3,
                    &s_tb6612TaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_TB6612,
                                   s_tb6612TaskHandle,
                                   TB6612_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "TB6612 task create failed!\r\n");
    }

    if (xTaskCreate(motor_encoder_task,
                    "motor_enc",
                    MOTOR_ENC_TASK_STACK_WORDS,
                    NULL,
                    3,
                    &s_motorEncoderTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_MOTOR_ENC,
                                   s_motorEncoderTaskHandle,
                                   MOTOR_ENC_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "motor encoder task create failed!\r\n");
    }

    app_imu_start();

    if (xTaskCreate(ins_task,
                    "ins",
                    INS_TASK_STACK_WORDS,
                    NULL,
                    4,
                    &s_insTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_INS,
                                   s_insTaskHandle,
                                   INS_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "INS task create failed!\r\n");
    }

    if (xTaskCreate(ins_cmd_task,
                    "ins_cmd",
                    INS_CMD_TASK_STACK_WORDS,
                    NULL,
                    1,
                    &s_insCmdTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_INS_CMD,
                                   s_insCmdTaskHandle,
                                   INS_CMD_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "INS cmd task create failed!\r\n");
    }

    if (xTaskCreate(car_comm_task,
                    "car_comm",
                    CAR_COMM_TASK_STACK_WORDS,
                    NULL,
                    2,
                    &s_carCommTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_CAR_COMM,
                                   s_carCommTaskHandle,
                                   CAR_COMM_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "car comm task create failed!\r\n");
    }

    if (xTaskCreate(motion_task,
                    "motion",
                    MOTION_TASK_STACK_WORDS,
                    NULL,
                    3,
                    &s_motionTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_MOTION,
                                   s_motionTaskHandle,
                                   MOTION_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "motion task create failed!\r\n");
    }

    if (xTaskCreate(nav_task,
                    "nav",
                    NAV_TASK_STACK_WORDS,
                    NULL,
                    2,
                    &s_navTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_NAV,
                                   s_navTaskHandle,
                                   NAV_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "nav task create failed!\r\n");
    }

    if (xTaskCreate(path_task,
                    "path",
                    PATH_TASK_STACK_WORDS,
                    NULL,
                    2,
                    &s_pathTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_PATH,
                                   s_pathTaskHandle,
                                   PATH_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "path task create failed!\r\n");
    }

    if (xTaskCreate(gray_task,
                    "gray",
                    GRAY_TASK_STACK_WORDS,
                    NULL,
                    3,
                    &s_grayTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_GRAY,
                                   s_grayTaskHandle,
                                   GRAY_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "gray task create failed!\r\n");
    }

    if (xTaskCreate(grayline_task,
                    "grayline",
                    GRAYLINE_TASK_STACK_WORDS,
                    NULL,
                    3,
                    &s_grayLineTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_GRAYLINE,
                                   s_grayLineTaskHandle,
                                   GRAYLINE_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "grayline task create failed!\r\n");
    }

    if (xTaskCreate(drive_mode_task,
                    "drive_mode",
                    DRIVE_MODE_TASK_STACK_WORDS,
                    NULL,
                    3,
                    &s_driveModeTaskHandle) != pdPASS) {
        LOGE(LOG_MOD_SYS, "drive mode task create failed!\r\n");
    }

#if APP_TEST1_ENABLE
    if (xTaskCreate(test1_task,
                    "test1",
                    TEST1_TASK_STACK_WORDS,
                    NULL,
                    2,
                    &s_test1TaskHandle) == pdPASS) {
    } else {
        LOGE(LOG_MOD_SYS, "test1 task create failed!\r\n");
    }
#endif

    schedule_deferred_tasks();

    LOGI_INIT(LOG_MOD_SYS, "Worker tasks created; deferred tasks scheduled\r\n");

    taskEXIT_CRITICAL();

    app_stack_monitor_clear_task(APP_STACK_MON_START);
    vTaskDelete(NULL);
}

static void start_deferred_tasks(TimerHandle_t timer)
{
    (void)xTimerDelete(timer, 0U);
    s_deferredStartTimer = NULL;

    if (xTaskCreate(flash_init_task,
                    "flash_test",
                    FLASH_TASK_STACK_WORDS,
                    NULL,
                    2,
                    &s_flashTaskHandle) == pdPASS) {
        app_stack_monitor_set_task(APP_STACK_MON_FLASH,
                                   s_flashTaskHandle,
                                   FLASH_TASK_STACK_WORDS);
    } else {
        LOGE(LOG_MOD_SYS, "flash test task create failed, heap_free=%lu\r\n",
             (unsigned long)xPortGetFreeHeapSize());
    }

    app_stack_monitor_start();
    LOGI_INIT(LOG_MOD_SYS, "Deferred worker tasks created\r\n");
}

static void schedule_deferred_tasks(void)
{
    s_deferredStartTimer = xTimerCreate("late_start",
                                        pdMS_TO_TICKS(DEFERRED_START_DELAY_MS),
                                        pdFALSE,
                                        NULL,
                                        start_deferred_tasks);
    if (s_deferredStartTimer == NULL) {
        LOGE(LOG_MOD_SYS, "deferred start timer create failed, heap_free=%lu\r\n",
             (unsigned long)xPortGetFreeHeapSize());
        return;
    }

    if (xTimerStart(s_deferredStartTimer, 0U) != pdPASS) {
        LOGE(LOG_MOD_SYS, "deferred start timer start failed, heap_free=%lu\r\n",
             (unsigned long)xPortGetFreeHeapSize());
        (void)xTimerDelete(s_deferredStartTimer, 0U);
        s_deferredStartTimer = NULL;
    }
}
