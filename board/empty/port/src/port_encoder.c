/**
 * @file    port_encoder.c
 * @brief   Port 层 EC11 编码器驱动实现
 * @note    实现 dev_encoder.h 的 DevEncoder OOP 契约
 *          纯轮询 + 软件消抖 + Gray 码查表解码 + 按键检测
 *          调用 BSP 层 DL_GPIO_* API 操作 PA24/PA25/PA26
 *
 *          引脚定义:
 *          PA24 - Encoder A 相 (CLK)
 *          PA25 - Encoder B 相 (DT)
 *          PA26 - Encoder 按键 (SW, 按下为低电平)
 *
 *          旋转事件采用单槽模式（取走后清除，未取走可覆盖）
 *          按键事件采用环形队列
 */

#include "port_encoder.h"
#include "port_log.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* ================================================================
 *  硬件引脚定义
 * ================================================================ */
#define ENC_PORT            GPIOA
#define ENC_PIN_A           DL_GPIO_PIN_24    /* PA24 - A 相 */
#define ENC_PIN_B           DL_GPIO_PIN_25    /* PA25 - B 相 */
#define ENC_PIN_KEY         DL_GPIO_PIN_26    /* PA26 - 按键 */

#define ENC_PIN_MASK        (ENC_PIN_A | ENC_PIN_B | ENC_PIN_KEY)

/* ================================================================
 *  消抖 & 按键计时参数
 * ================================================================ */
#define DEBOUNCE_ENC_CNT    3U      /* 编码器连续 3 次稳定才算有效 (15ms @5ms周期) */
#define DEBOUNCE_KEY_CNT    2U      /* 按键连续 2 次稳定才算有效 (10ms @5ms周期) */

#define KEY_SHORT_MIN_MS    50U     /* 短按最小 50ms */
#define KEY_SHORT_MAX_MS    800U    /* 短按最大 800ms */
#define KEY_LONG_MIN_MS     1000U   /* 长按最小 1000ms */
#define KEY_DOUBLE_GAP_MS   250U    /* 双击间隔最大 250ms */

#define POLL_MS             5U      /* 轮询周期 5ms */

/* ================================================================
 *  编码器内部状态
 * ================================================================ */

/* Gray 码状态查表法:
 *   编码器 4 个状态: 00, 01, 11, 10 (顺时针循环)
 *   表索引 = (prev << 2) | curr
 *   值定义:
 *      0  = 无变化 (抖动或无效)
 *      1  = 顺时针 1 格
 *     -1  = 逆时针 1 格
 */
#define ENC_STATE_00    0U
#define ENC_STATE_01    1U
#define ENC_STATE_11    2U
#define ENC_STATE_10    3U

static const int8_t s_encGrayTable[16] = {
    /* prev=00 */   0,  1, -1,  0,    /* 00→00:0  00→01:+1  00→11:?   00→10:-1  */
    /* prev=01 */  -1,  0,  1,  0,    /* 01→00:-1 01→01:0   01→11:+1  01→10:?   */
    /* prev=11 */   1,  0,  0, -1,    /* 11→00:?  11→01:0   11→11:0   11→10:-1  */
    /* prev=10 */   0, -1,  1,  0     /* 10→00:0  10→01:-1  10→11:+1  10→10:0   */
};

/* 按键 FSM 状态 */
typedef enum {
    KEY_STATE_IDLE,             /* 空闲等待按下 */
    KEY_STATE_DEBOUNCE_DOWN,    /* 消抖按下中 */
    KEY_STATE_PRESSED,          /* 已按下，等待释放或长按 */
    KEY_STATE_DEBOUNCE_UP,      /* 消抖释放中 */
    KEY_STATE_WAIT_DOUBLE       /* 等待第二次短按 */
} KeyState_t;

/* 编码器消抖缓冲区 */
typedef struct {
    uint8_t   stableCount;      /* 已连续稳定的次数 */
    uint8_t   lastStableAB;     /* 上一次确认有效的 AB 相值 (0~3) */
} EncDebounce_t;

/* 按键消抖缓冲区 */
typedef struct {
    KeyState_t  state;          /* 按键 FSM 当前状态 */
    uint8_t     stableCount;    /* 消抖计数 */
    bool        lastStableRaw;  /* 上一次确认有效的按键电平 (true=高/释放) */
    uint32_t    pressTick;      /* 按下时刻的 tick 计数 */
    uint32_t    firstClickTick; /* 第一次短按释放时刻 tick */
    uint32_t    prevRaw;        /* 上一次原始采样值用于边沿检测 */
} KeyFSM_t;

/* ================================================================
 *  按键事件环形队列
 * ================================================================ */
#define KEY_EVT_QUEUE_SIZE  16U

typedef struct {
    DevEncoder_Event_t  buf[KEY_EVT_QUEUE_SIZE];
    uint8_t             head;
    uint8_t             tail;
} EvtQueue_t;

/* ================================================================
 *  全局静态变量
 * ================================================================ */
static DevEncoder     s_encoderIf;              /* Device 层接口实例 */
static EncDebounce_t  s_encDeb;                 /* 编码器消抖状态 */
static KeyFSM_t       s_keyFSM;                 /* 按键 FSM 状态 */
static EvtQueue_t     s_keyQueue;               /* 按键事件环形队列 */
static int32_t        s_position;               /* 位置计数值 */
static uint32_t       s_tickCount;              /* tick 计数器 (每 Poll 一次 +1) */

/* 软件 2 分频计数器 */
static int8_t s_encDivCounter = 0;

/* ================================================================
 *  按键事件队列操作（内部静态）
 * ================================================================ */

static void key_queue_init(EvtQueue_t *q)
{
    q->head = 0;
    q->tail = 0;
}

static bool key_queue_push(EvtQueue_t *q, DevEncoder_Event_t evt)
{
    uint8_t next = (uint8_t)(q->head + 1) % KEY_EVT_QUEUE_SIZE;
    if (next == q->tail) {
        return false;   /* 队列满 */
    }
    q->buf[q->head] = evt;
    q->head = next;
    return true;
}

static DevEncoder_Event_t key_queue_pop(EvtQueue_t *q)
{
    if (q->tail == q->head) {
        return ENCODER_EVT_NONE;    /* 队列空 */
    }
    DevEncoder_Event_t evt = q->buf[q->tail];
    q->tail = (uint8_t)(q->tail + 1) % KEY_EVT_QUEUE_SIZE;
    return evt;
}

/* ================================================================
 *  读取 GPIO 原始值
 * ================================================================ */

static uint32_t enc_read_pins(void)
{
    /*
     * 直接读 GPIO 寄存器，不依赖 DL_GPIO_readPins mask 行为
     */
    uint32_t din = GPIOA->DIN31_0;

    uint32_t rawA   = (din & ENC_PIN_A)   ? 1U : 0U;   /* PA24 - A 相 */
    uint32_t rawB   = (din & ENC_PIN_B)   ? 1U : 0U;   /* PA25 - B 相 */
    uint32_t rawKey = (din & ENC_PIN_KEY) ? 1U : 0U;   /* PA26 - 按键 */

    return (rawA << 2) | (rawB << 1) | rawKey;
    /* bit2=A, bit1=B, bit0=KEY (1=释放, 0=按下) */
}

/* ================================================================
 *  编码器解码 + 消抖（设置旋转单槽事件）
 * ================================================================ */

static void enc_decode(uint32_t raw)
{
    uint8_t rawAB = (uint8_t)((raw >> 1) & 0x03);   /* bit2=A, bit1=B → 0~3 */
    uint8_t prev  = s_encDeb.lastStableAB;
    uint8_t idx   = (uint8_t)((prev << 2) | rawAB);
    int8_t  dir   = s_encGrayTable[idx];

    if (dir == 0) {
        /* 没有有效方向变化，检查是否与上次稳定值相同 */
        if (rawAB == prev) {
            s_encDeb.stableCount++;
            if (s_encDeb.stableCount >= DEBOUNCE_ENC_CNT) {
                /* 已确认稳定，无变化 */
                s_encDeb.stableCount = DEBOUNCE_ENC_CNT; /* 饱和 */
            }
        } else {
            /* 与上次稳定值不同但方向为0→无效跳变，重置消抖 */
            s_encDeb.stableCount = 0;
        }
    } else {
        /* 有方向变化，检查是否已经消抖确认 */
        s_encDeb.stableCount++;
        if (s_encDeb.stableCount >= DEBOUNCE_ENC_CNT) {
            /* 消抖通过，更新位置 */
            s_position += dir;                      /* 更新位置 */
            s_encDeb.lastStableAB = rawAB;           /* 更新稳定值 */

            /* 2 分频累加 */
            s_encDivCounter += (dir > 0) ? 1 : -1;

            /* 满 2 输出一次事件 → 统一 push 到环形队列 */
            if (s_encDivCounter >= 2) {
                key_queue_push(&s_keyQueue, ENCODER_EVT_CW);
                s_encDivCounter -= 2;
            } else if (s_encDivCounter <= -2) {
                key_queue_push(&s_keyQueue, ENCODER_EVT_CCW);
                s_encDivCounter += 2;
            }

            s_encDeb.stableCount = 0;                /* 重置消抖 */
        }
    }
}

/* ================================================================
 *  按键 FSM
 * ================================================================ */

static void key_fsm(uint32_t raw)
{
    bool rawLevel = (raw & 0x01) ? true : false;    /* bit0: 1=释放, 0=按下 */
    bool edgeDown  = (s_keyFSM.prevRaw == 1) && (rawLevel == 0);  /* 下降沿 → 按下 */
    bool edgeUp    = (s_keyFSM.prevRaw == 0) && (rawLevel == 1);  /* 上升沿 → 释放 */
    s_keyFSM.prevRaw = rawLevel;

    switch (s_keyFSM.state) {

    case KEY_STATE_IDLE:
        if (edgeDown) {
            s_keyFSM.stableCount = 0;
            s_keyFSM.state = KEY_STATE_DEBOUNCE_DOWN;
        }
        break;

    case KEY_STATE_DEBOUNCE_DOWN:
        if (rawLevel == 0) {        /* 持续按下 */
            s_keyFSM.stableCount++;
            if (s_keyFSM.stableCount >= DEBOUNCE_KEY_CNT) {
                /* 消抖确认按下 */
                s_keyFSM.lastStableRaw = false;
                s_keyFSM.pressTick = s_tickCount;
                s_keyFSM.state = KEY_STATE_PRESSED;
            }
        } else {
            /* 抖动反弹，回到 IDLE */
            s_keyFSM.state = KEY_STATE_IDLE;
        }
        break;

    case KEY_STATE_PRESSED:
        if (rawLevel == 1) {        /* 释放 */
            s_keyFSM.stableCount = 0;
            s_keyFSM.state = KEY_STATE_DEBOUNCE_UP;
        } else {
            /* 检查长按 */
            uint32_t duration = (s_tickCount - s_keyFSM.pressTick) * POLL_MS;
            if (duration >= KEY_LONG_MIN_MS) {
                /* 长按确认 */
                key_queue_push(&s_keyQueue, ENCODER_EVT_KEY_LONG);
                s_keyFSM.state = KEY_STATE_IDLE;
            }
        }
        break;

    case KEY_STATE_DEBOUNCE_UP:
        if (rawLevel == 1) {        /* 持续释放 */
            s_keyFSM.stableCount++;
            if (s_keyFSM.stableCount >= DEBOUNCE_KEY_CNT) {
                /* 消抖确认释放，计算按下时长 */
                uint32_t duration = (s_tickCount - s_keyFSM.pressTick) * POLL_MS;

                if (duration <= KEY_SHORT_MAX_MS) {
                    /* 短按 */
                    if (s_keyFSM.firstClickTick == 0) {
                        /* 第一次短按，记录时刻 */
                        s_keyFSM.firstClickTick = s_tickCount;
                        s_keyFSM.state = KEY_STATE_WAIT_DOUBLE;
                    } else {
                        /* 可能是第二次短按 */
                        uint32_t gap = (s_tickCount - s_keyFSM.firstClickTick) * POLL_MS;
                        if (gap <= KEY_DOUBLE_GAP_MS) {
                            /* 双击 */
                            key_queue_push(&s_keyQueue, ENCODER_EVT_KEY_DOUBLE);
                            s_keyFSM.firstClickTick = 0;
                            s_keyFSM.state = KEY_STATE_IDLE;
                        } else {
                            /* 间隔超时，视为两个独立短按 */
                            key_queue_push(&s_keyQueue, ENCODER_EVT_KEY_SHORT);
                            /* 这次短按作为新的第一次 */
                            s_keyFSM.firstClickTick = s_tickCount;
                            s_keyFSM.state = KEY_STATE_WAIT_DOUBLE;
                        }
                    }
                } else {
                    /* 时间超过短按窗口且未触发长按，视为无效 */
                    s_keyFSM.firstClickTick = 0;
                    s_keyFSM.state = KEY_STATE_IDLE;
                }
            }
        } else {
            /* 抖动反弹回按下 */
            s_keyFSM.state = KEY_STATE_PRESSED;
        }
        break;

    case KEY_STATE_WAIT_DOUBLE:
        /* 等待第二次短按 */
        if (edgeDown) {
            /* 检测到再次按下 */
            s_keyFSM.stableCount = 0;
            s_keyFSM.state = KEY_STATE_DEBOUNCE_DOWN;
        } else {
            /* 检查是否超时 */
            uint32_t gap = (s_tickCount - s_keyFSM.firstClickTick) * POLL_MS;
            if (gap > KEY_DOUBLE_GAP_MS) {
                /* 超时未等到第二次，产生短按事件 */
                key_queue_push(&s_keyQueue, ENCODER_EVT_KEY_SHORT);
                s_keyFSM.firstClickTick = 0;
                s_keyFSM.state = KEY_STATE_IDLE;
            }
        }
        break;

    default:
        /* 异常恢复 */
        s_keyFSM.state = KEY_STATE_IDLE;
        s_keyFSM.firstClickTick = 0;
        break;
    }
}

/* ================================================================
 *  DevEncoder 接口函数实现
 * ================================================================ */

static void encoder_init(DevEncoder *self)
{
    (void)self;
    /* GPIO 初始化由 SYSCFG_DL_GPIO_init() 在 PORT_SYSTEM_Init 中完成 */

    s_position = 0;
    s_tickCount = 0;

    s_encDeb.stableCount = 0;
    s_encDeb.lastStableAB = 0;

    s_keyFSM.state = KEY_STATE_IDLE;
    s_keyFSM.stableCount = 0;
    s_keyFSM.lastStableRaw = true;      /* 默认释放（上拉） */
    s_keyFSM.pressTick = 0;
    s_keyFSM.firstClickTick = 0;
    s_keyFSM.prevRaw = true;            /* 默认释放 */

    key_queue_init(&s_keyQueue);
}

static DevEncoder_Event_t encoder_getEvent(DevEncoder *self)
{
    (void)self;

    /* 统一从环形队列取事件（旋转 + 按键，FIFO 公平消费） */
    return key_queue_pop(&s_keyQueue);
}

static int32_t encoder_getPosition(DevEncoder *self)
{
    (void)self;
    return s_position;
}

static void encoder_resetPosition(DevEncoder *self)
{
    (void)self;
    s_position = 0;
}

/* ================================================================
 *  公有 API 实现
 * ================================================================ */

void PORT_ENCODER_Init(void)
{
    /*
     * 强制重新初始化 ENCODER 三个引脚为数字输入 + 内部上拉。
     * 虽然 SYSCFG_DL_GPIO_init() 已配置，但可能被其他外设或
     * DL_GPIO_reset(GPIOA) 覆盖。此处显式确保配置生效。
     */
    DL_GPIO_initDigitalInputFeatures(ENCODER_KEY_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_encodera_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_encoderb_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    s_position = 0;
    s_tickCount = 0;

    s_encDeb.stableCount = 0;
    s_encDeb.lastStableAB = 0;

    s_keyFSM.state = KEY_STATE_IDLE;
    s_keyFSM.stableCount = 0;
    s_keyFSM.lastStableRaw = true;
    s_keyFSM.pressTick = 0;
    s_keyFSM.firstClickTick = 0;
    s_keyFSM.prevRaw = true;

    key_queue_init(&s_keyQueue);
}


void PORT_ENCODER_Poll(void)
{
    uint32_t raw = enc_read_pins();

    /* 编码器解码（设置旋转单槽事件） */
    enc_decode(raw);

    /* 按键 FSM（push 到环形队列） */
    key_fsm(raw);

    /* tick 递增 */
    s_tickCount++;
}

/* ================================================================
 *  事件名称映射表（下沉至 Port 层）
 * ================================================================ */

const char* PORT_ENCODER_EventToStr(DevEncoder_Event_t evt)
{
    switch (evt) {
        case ENCODER_EVT_CW:          return "CW";
        case ENCODER_EVT_CCW:         return "CCW";
        case ENCODER_EVT_KEY_SHORT:   return "SHORT_PRESS";
        case ENCODER_EVT_KEY_LONG:    return "LONG_PRESS";
        case ENCODER_EVT_KEY_DOUBLE:  return "DOUBLE_CLICK";
        default:                      return "UNKNOWN";
    }
}

/* ================================================================
 *  注册 Device 接口
 * ================================================================ */

DevEncoder* GetEncoder(void)
{
    s_encoderIf.init          = encoder_init;
    s_encoderIf.getEvent      = encoder_getEvent;
    s_encoderIf.getPosition   = encoder_getPosition;
    s_encoderIf.resetPosition = encoder_resetPosition;

    return &s_encoderIf;
}