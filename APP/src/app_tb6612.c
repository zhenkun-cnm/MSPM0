/**
 * @file    app_tb6612.c
 * @brief   App layer TB6612 motor task implementation
 * @note    4-layer decoupled architecture - App layer
 *          Controls hardware via DevTB6612 handle, no DL_* / register access.
 *          Receives command queue from TFT menu, executes motor control.
 */

#include "app_tb6612.h"
#include "dev_tb6612.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Command queue instance (declared in app_tb6612.h) */
QueueHandle_t g_motorCmdQueue = NULL;

/* Direction name strings (for log / menu rendering) */
static const char* s_dirNames[] = {
    "Forward",      /* TB6612_DIR_FORWARD */
    "Reverse",      /* TB6612_DIR_REVERSE */
    "Brake",        /* TB6612_DIR_BRAKE   */
    "Coast"         /* TB6612_DIR_COAST    */
};

#define DIR_COUNT   (sizeof(s_dirNames) / sizeof(s_dirNames[0]))

/* Forward-declare port-level single-channel helpers */
extern void PORT_TB6612_LeftOnly(void);
extern void PORT_TB6612_RightOnly(void);
extern void PORT_TB6612_SetLeftDuty(uint8_t pct);
extern void PORT_TB6612_SetRightDuty(uint8_t pct);

void tb6612_task(void *pvParameters)
{
    (void)pvParameters;

    DevTB6612 *motor = GetTB6612();
    if (motor == NULL) {
        LOG_ERROR("[TB6612] Device handle is NULL!\r\n");
        vTaskDelete(NULL);
        return;
    }

    /* Init hardware: coast + duty 0 */
    motor->init(motor);
    /* LOG_INFO("[TB6612] Initialized (pins: PA12/PWMA, PA13/PWMB, "
             "PA22/AIN1, PB21/AIN2, PB23/BIN1, PA23/BIN2)\r\n"); */

    static bool     s_on       = false;
    static uint8_t  s_leftSpd  = 0;
    static uint8_t  s_rightSpd = 0;
    static MotorCmd cmd;

    while (1)
    {
        /* Block waiting for menu commands */
        if (xQueueReceive(g_motorCmdQueue, &cmd, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        switch (cmd.type) {
            case MOTOR_CMD_ONOFF:
                if (cmd.val != 0) {
                    s_on = true;
                    motor->enable(motor);
                    PORT_TB6612_SetLeftDuty(s_leftSpd);
                    PORT_TB6612_SetRightDuty(s_rightSpd);
                    /* LOG_INFO("[TB6612] Motor ON, speed=%d%%, dir=%s\r\n",
                             motor->getSpeed(motor),
                             s_dirNames[motor->getDirection(motor)]); */
                } else {
                    s_on = false;
                    motor->disable(motor);
                    /* LOG_INFO("[TB6612] Motor OFF\r\n"); */
                }
                break;

            case MOTOR_CMD_SPEED:
                if (cmd.val < 0) cmd.val = 0;
                if (cmd.val > 100) cmd.val = 100;
                motor->setSpeed(motor, (uint8_t)cmd.val);
                if (s_on) {
                    /* LOG_INFO("[TB6612] Speed=%d%%\r\n", (int)cmd.val); */
                }
                break;

            case MOTOR_CMD_DIR:
                if (cmd.val >= 0 && cmd.val < (int16_t)DIR_COUNT) {
                    motor->setDirection(motor, (TB6612_Dir)cmd.val);
                    if (s_on) {
                        /* LOG_INFO("[TB6612] Direction=%s\r\n",
                                 s_dirNames[motor->getDirection(motor)]); */
                    }
                }
                break;

            case MOTOR_CMD_LEFT_ONLY:
                PORT_TB6612_LeftOnly();
                /* LOG_INFO("[TB6612] Left wheel only\r\n"); */
                break;

            case MOTOR_CMD_RIGHT_ONLY:
                PORT_TB6612_RightOnly();
                /* LOG_INFO("[TB6612] Right wheel only\r\n"); */
                break;

            case MOTOR_CMD_LEFT_SPEED:
                if (cmd.val < 0) cmd.val = 0;
                if (cmd.val > 100) cmd.val = 100;
                s_leftSpd = (uint8_t)cmd.val;
                if (s_on) {
                    PORT_TB6612_SetLeftDuty(s_leftSpd);
                }
                /* LOG_INFO("[TB6612] Left speed=%d%%\r\n", (int)cmd.val); */
                break;

            case MOTOR_CMD_RIGHT_SPEED:
                if (cmd.val < 0) cmd.val = 0;
                if (cmd.val > 100) cmd.val = 100;
                s_rightSpd = (uint8_t)cmd.val;
                if (s_on) {
                    PORT_TB6612_SetRightDuty(s_rightSpd);
                }
                /* LOG_INFO("[TB6612] Right speed=%d%%\r\n", (int)cmd.val); */
                break;

            default:
                break;
        }
    }
}