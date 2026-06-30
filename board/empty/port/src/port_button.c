/**
 * @file    port_button.c
 * @brief   Port 层独立按键驱动实现
 * @note    实现 dev_button.h 的 DevButton OOP 契约
 *          纯轮询 + 软件消抖 + 五状态 FSM（短按/长按/双击），非阻塞
 *          移植自 port_encoder.c 的按键 FSM，扩展为两路独立按键
 *
 *          引脚定义（低电平有效，按下=低）:
 *          PA27 - BUTTON1 (内部上拉)
 *          PB27 - BUTTON2 (外部上拉，硬件已处理)
 */

#include "port_button.h"
#include "ti_msp_dl_config.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>

/* ================================================================
 *  硬件引脚定义（复用 SYSCFG 生成的符号）
 * ================================================================ */
#define BTN1_PORT       BUTTON_BUTTON1_PORT     /* GPIOA */
#define BTN1_PIN        BUTTON_BUTTON1_PIN      /* DL_GPIO_PIN_27 */
#define BTN2_PORT       BUTTON_BUTTON2_PORT     /* GPIOB */
#define BTN2_PIN        BUTTON_BUTTON2_PIN      /* DL_GPIO_PIN_27 */

#define BTN_COUNT       2U

/* ================================================================
 *  消抖 & 按键计时参数（与 port_encoder.c 保持一致）
 * ================================================================ */
#define DEBOUNCE_KEY_CNT    2U      /* 连续 2 次稳定才算有效 (10ms @5ms周期) */
#define KEY_SHORT_MAX_MS    800U    /* 短按最大 800ms */
#define KEY_LONG_MIN_MS     1000U   /* 长按最小 1000ms */
#define KEY_DOUBLE_GAP_MS   250U    /* 双击间隔最大 250ms */
#define POLL_MS             5U      /* 轮询周期 5ms */

/* ================================================================
 *  按键 FSM 状态
 * ================================================================ */
typedef enum {
    KEY_STATE_IDLE,             /* 空闲等待按下 */
    KEY_STATE_DEBOUNCE_DOWN,    /* 消抖按下中 */
    KEY_STATE_PRESSED,          /* 已按下，等待释放或长按 */
    KEY_STATE_DEBOUNCE_UP,      /* 消抖释放中 */
    KEY_STATE_WAIT_DOUBLE       /* 等待第二次短按 */
} KeyState_t;

typedef struct {
    KeyState_t  state;          /* FSM 当前状态 */
    uint8_t     stableCount;    /* 消抖计数 */
    uint32_t    pressTick;      /* 按下时刻的 tick 计数 */
    uint32_t    firstClickTick; /* 第一次短按释放时刻 tick */
    bool        prevRaw;        /* 上一次原始采样值（true=高/释放） */
} KeyFSM_t;

/* ================================================================
 *  按键事件环形队列
 * ================================================================ */
#define BTN_EVT_QUEUE_SIZE  16U

typedef struct {
    DevButton_Event_t  buf[BTN_EVT_QUEUE_SIZE];
    uint8_t            head;
    uint8_t            tail;
} EvtQueue_t;

/* ================================================================
 *  全局静态变量
 * ================================================================ */
static DevButton   s_buttonIf;              /* Device 层接口实例 */
static KeyFSM_t    s_fsm[BTN_COUNT];        /* 两路按键 FSM 状态 */
static EvtQueue_t  s_btnQueue;              /* 按键事件环形队列 */
static uint32_t    s_tickCount;             /* tick 计数器（每 Poll +1） */

/* ================================================================
 *  按键事件队列操作（内部静态）
 * ================================================================ */

static void btn_queue_init(EvtQueue_t *q)
{
    q->head = 0;
    q->tail = 0;
}

static bool btn_queue_push(EvtQueue_t *q, DevButton_Event_t evt)
{
    uint8_t next = (uint8_t)(q->head + 1) % BTN_EVT_QUEUE_SIZE;
    if (next == q->tail) {
        return false;   /* 队列满 */
    }
    q->buf[q->head] = evt;
    q->head = next;
    return true;
}

static DevButton_Event_t btn_queue_pop(EvtQueue_t *q)
{
    DevButton_Event_t none = { 0, BUTTON_EVT_NONE };
    if (q->tail == q->head) {
        return none;    /* 队列空 */
    }
    DevButton_Event_t evt = q->buf[q->tail];
    q->tail = (uint8_t)(q->tail + 1) % BTN_EVT_QUEUE_SIZE;
    return evt;
}

/* 推送一个带 ID 的事件 */
static void btn_emit(int idx, DevButton_EvtType_t type)
{
    DevButton_Event_t evt;
    evt.id   = (uint8_t)((idx == 0) ? BUTTON_ID_1 : BUTTON_ID_2);
    evt.type = type;
    btn_queue_push(&s_btnQueue, evt);
}

/* ================================================================
 *  读取单路按键原始电平（true=高/释放，false=低/按下）
 * ================================================================ */

static bool btn_read(int idx)
{
    if (idx == 0) {
        return (DL_GPIO_readPins(BTN1_PORT, BTN1_PIN) != 0) ? true : false;
    } else {
        return (DL_GPIO_readPins(BTN2_PORT, BTN2_PIN) != 0) ? true : false;
    }
}

/* ================================================================
 *  单路按键 FSM（移植自 port_encoder.c 的 key_fsm，按 idx 索引状态）
 * ================================================================ */

static void btn_fsm(int idx, bool rawLevel)
{
    KeyFSM_t *k = &s_fsm[idx];

    bool edgeDown = (k->prevRaw == true)  && (rawLevel == false);  /* 下降沿 → 按下 */
    bool edgeUp   = (k->prevRaw == false) && (rawLevel == true);   /* 上升沿 → 释放 */
    (void)edgeUp;
    k->prevRaw = rawLevel;

    switch (k->state) {

    case KEY_STATE_IDLE:
        if (edgeDown) {
            k->stableCount = 0;
            k->state = KEY_STATE_DEBOUNCE_DOWN;
        }
        break;

    case KEY_STATE_DEBOUNCE_DOWN:
        if (rawLevel == false) {        /* 持续按下 */
            k->stableCount++;
            if (k->stableCount >= DEBOUNCE_KEY_CNT) {
                /* 消抖确认按下 */
                k->pressTick = s_tickCount;
                k->state = KEY_STATE_PRESSED;
            }
        } else {
            /* 抖动反弹，回到 IDLE */
            k->state = KEY_STATE_IDLE;
        }
        break;

    case KEY_STATE_PRESSED:
        if (rawLevel == true) {         /* 释放 */
            k->stableCount = 0;
            k->state = KEY_STATE_DEBOUNCE_UP;
        } else {
            /* 检查长按 */
            uint32_t duration = (s_tickCount - k->pressTick) * POLL_MS;
            if (duration >= KEY_LONG_MIN_MS) {
                btn_emit(idx, BUTTON_EVT_LONG);
                k->state = KEY_STATE_IDLE;
            }
        }
        break;

    case KEY_STATE_DEBOUNCE_UP:
        if (rawLevel == true) {         /* 持续释放 */
            k->stableCount++;
            if (k->stableCount >= DEBOUNCE_KEY_CNT) {
                /* 消抖确认释放，计算按下时长 */
                uint32_t duration = (s_tickCount - k->pressTick) * POLL_MS;

                if (duration <= KEY_SHORT_MAX_MS) {
                    /* 短按 */
                    if (k->firstClickTick == 0) {
                        /* 第一次短按，记录时刻 */
                        k->firstClickTick = s_tickCount;
                        k->state = KEY_STATE_WAIT_DOUBLE;
                    } else {
                        /* 可能是第二次短按 */
                        uint32_t gap = (s_tickCount - k->firstClickTick) * POLL_MS;
                        if (gap <= KEY_DOUBLE_GAP_MS) {
                            btn_emit(idx, BUTTON_EVT_DOUBLE);
                            k->firstClickTick = 0;
                            k->state = KEY_STATE_IDLE;
                        } else {
                            /* 间隔超时，视为两个独立短按 */
                            btn_emit(idx, BUTTON_EVT_SHORT);
                            k->firstClickTick = s_tickCount;
                            k->state = KEY_STATE_WAIT_DOUBLE;
                        }
                    }
                } else {
                    /* 时间超过短按窗口且未触发长按，视为无效 */
                    k->firstClickTick = 0;
                    k->state = KEY_STATE_IDLE;
                }
            }
        } else {
            /* 抖动反弹回按下 */
            k->state = KEY_STATE_PRESSED;
        }
        break;

    case KEY_STATE_WAIT_DOUBLE:
        if (edgeDown) {
            /* 检测到再次按下 */
            k->stableCount = 0;
            k->state = KEY_STATE_DEBOUNCE_DOWN;
        } else {
            /* 检查是否超时 */
            uint32_t gap = (s_tickCount - k->firstClickTick) * POLL_MS;
            if (gap > KEY_DOUBLE_GAP_MS) {
                /* 超时未等到第二次，产生短按事件 */
                btn_emit(idx, BUTTON_EVT_SHORT);
                k->firstClickTick = 0;
                k->state = KEY_STATE_IDLE;
            }
        }
        break;

    default:
        k->state = KEY_STATE_IDLE;
        k->firstClickTick = 0;
        break;
    }
}

/* ================================================================
 *  DevButton 接口函数实现
 * ================================================================ */

static void reset_state(void)
{
    for (int i = 0; i < (int)BTN_COUNT; i++) {
        s_fsm[i].state          = KEY_STATE_IDLE;
        s_fsm[i].stableCount    = 0;
        s_fsm[i].pressTick      = 0;
        s_fsm[i].firstClickTick = 0;
        s_fsm[i].prevRaw        = true;     /* 默认释放（上拉） */
    }
    s_tickCount = 0;
    btn_queue_init(&s_btnQueue);
}

static void button_init(DevButton *self)
{
    (void)self;
    /* GPIO 已在 SYSCFG_DL_GPIO_init() 中配置为输入 */
    reset_state();
}

static DevButton_Event_t button_getEvent(DevButton *self)
{
    (void)self;
    return btn_queue_pop(&s_btnQueue);
}

/* ================================================================
 *  公有 API 实现
 * ================================================================ */

void PORT_BUTTON_Init(void)
{
    reset_state();
}

void PORT_BUTTON_Poll(void)
{
    btn_fsm(0, btn_read(0));    /* PA27 */
    btn_fsm(1, btn_read(1));    /* PB27 */
    s_tickCount++;
}

const char* PORT_BUTTON_EvtToStr(DevButton_EvtType_t t)
{
    switch (t) {
        case BUTTON_EVT_SHORT:  return "SHORT_PRESS";
        case BUTTON_EVT_LONG:   return "LONG_PRESS";
        case BUTTON_EVT_DOUBLE: return "DOUBLE_CLICK";
        default:                return "UNKNOWN";
    }
}

/* ================================================================
 *  注册 Device 接口
 * ================================================================ */

DevButton* GetButton(void)
{
    s_buttonIf.init     = button_init;
    s_buttonIf.getEvent = button_getEvent;
    return &s_buttonIf;
}
