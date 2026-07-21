/**
 * @file    port_motor_encoder2.c
 * @brief   Motor2 encoder capture without edge interrupts.
 *
 * M2 uses TIMG7 dual edge-time capture plus two DMA channels:
 *   A phase: PA28 / TIMG7_CCP0 -> CCR0 -> DMA_CH0 -> g_enc2ABuf[]
 *   B phase: PA31 / TIMG7_CCP1 -> CCR1 -> DMA_CH1 -> g_enc2BBuf[]
 *
 * The encoder path does not depend on DMA reading GPIO input registers.
 */

#include "port_motor_encoder2.h"
#include "ti_msp_dl_config.h"
#include "port_log.h"
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <stdbool.h>

#define ENC2_DMA_A_CHAN_ID          (DMA_CH0_CHAN_ID)
#define ENC2_DMA_B_CHAN_ID          (DMA_CH1_CHAN_ID)
#define ENC2_EVT_CH_A               (1U)
#define ENC2_EVT_CH_B               (2U)
#define ENC2_TIMER_LOAD             (65535U)
#define ENC2_TIMER_PRESCALE         (9U)    /* 80 MHz / 8 / (9 + 1) = 1 MHz */
#define ENC2_PHASE_A_MASK           (1U)
#define ENC2_PHASE_B_MASK           (2U)
#define ENC2_DIR_SIGN               (-1)

static uint32_t g_enc2ABuf[ENC2_BUF_SIZE];
static uint32_t g_enc2BBuf[ENC2_BUF_SIZE];

static uint32_t s_aReadIndex = 0;
static uint32_t s_bReadIndex = 0;
static uint8_t s_abState = 0;
static bool s_firstPoll = true;
static uint32_t s_diagCounter = 0;
static uint32_t s_invalidTransitions = 0;
static uint32_t s_overrunWarnings = 0;
static uint16_t s_lastAEvents = 0;
static uint16_t s_lastBEvents = 0;
static uint16_t s_lastEqualTimestamps = 0;

static const int8_t s_qdecTable[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0
};

static uint8_t read_initial_ab_state(void);

static uint32_t dma_write_offset(uint8_t chan)
{
    uint16_t remaining = DL_DMA_getTransferSize(DMA, chan);
    return (ENC2_BUF_SIZE - remaining) % ENC2_BUF_SIZE;
}

uint32_t PORT_MOTOR_ENCODER2_GetDmaWriteOffset(void)
{
    return dma_write_offset(ENC2_DMA_A_CHAN_ID);
}

void PORT_MOTOR_ENCODER2_RequestResync(void)
{
    DL_TimerG_disableEvent(TB6612_ENB_INST, DL_TIMERG_EVENT_ROUTE_1,
        DL_TIMERG_EVENT_CC0_DN_EVENT | DL_TIMERG_EVENT_CC0_UP_EVENT);
    DL_TimerG_disableEvent(TB6612_ENB_INST, DL_TIMERG_EVENT_ROUTE_2,
        DL_TIMERG_EVENT_CC1_DN_EVENT | DL_TIMERG_EVENT_CC1_UP_EVENT);

    s_aReadIndex = dma_write_offset(ENC2_DMA_A_CHAN_ID);
    s_bReadIndex = dma_write_offset(ENC2_DMA_B_CHAN_ID);
    s_abState = read_initial_ab_state();
    s_firstPoll = false;
    s_lastAEvents = 0;
    s_lastBEvents = 0;
    s_lastEqualTimestamps = 0;

    DL_TimerG_enableEvent(TB6612_ENB_INST, DL_TIMERG_EVENT_ROUTE_1,
        DL_TIMERG_EVENT_CC0_DN_EVENT | DL_TIMERG_EVENT_CC0_UP_EVENT);
    DL_TimerG_enableEvent(TB6612_ENB_INST, DL_TIMERG_EVENT_ROUTE_2,
        DL_TIMERG_EVENT_CC1_DN_EVENT | DL_TIMERG_EVENT_CC1_UP_EVENT);
}

bool PORT_MOTOR_ENCODER2_GetDiag(PortMotorEncoder2Diag_t *out)
{
    if (out == NULL) {
        return false;
    }

    out->a_dma_offset = dma_write_offset(ENC2_DMA_A_CHAN_ID);
    out->b_dma_offset = dma_write_offset(ENC2_DMA_B_CHAN_ID);
    out->ab_state = s_abState;
    out->last_a_events = s_lastAEvents;
    out->last_b_events = s_lastBEvents;
    out->last_equal_timestamps = s_lastEqualTimestamps;
    out->invalid_transitions = s_invalidTransitions;
    out->overrun_warnings = s_overrunWarnings;
    return true;
}

static uint32_t ring_count(uint32_t readIndex, uint32_t writeIndex)
{
    if (writeIndex >= readIndex) {
        return writeIndex - readIndex;
    }
    return (ENC2_BUF_SIZE - readIndex) + writeIndex;
}

static void clear_buffers(void)
{
    uint32_t i;

    for (i = 0; i < ENC2_BUF_SIZE; i++) {
        g_enc2ABuf[i] = 0;
        g_enc2BBuf[i] = 0;
    }
}

static void configure_timg7_capture(void)
{
    const DL_TimerG_ClockConfig clockConfig = {
        .clockSel = DL_TIMER_CLOCK_BUSCLK,
        .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
        .prescale = ENC2_TIMER_PRESCALE
    };

    DL_TimerG_stopCounter(TB6612_ENB_INST);
    DL_TimerG_disableEvent(TB6612_ENB_INST, DL_TIMERG_EVENT_ROUTE_1,
        DL_TIMERG_EVENT_CC0_DN_EVENT | DL_TIMERG_EVENT_CC1_DN_EVENT);
    DL_TimerG_disableEvent(TB6612_ENB_INST, DL_TIMERG_EVENT_ROUTE_2,
        DL_TIMERG_EVENT_CC0_DN_EVENT | DL_TIMERG_EVENT_CC1_DN_EVENT);

    DL_GPIO_initPeripheralInputFunction(GPIO_TB6612_ENB_C0_IOMUX,
        GPIO_TB6612_ENB_C0_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_TB6612_ENB_C1_IOMUX,
        GPIO_TB6612_ENB_C1_IOMUX_FUNC);

    DL_TimerG_setClockConfig(TB6612_ENB_INST,
        (DL_TimerG_ClockConfig *) &clockConfig);
    DL_TimerG_setLoadValue(TB6612_ENB_INST, ENC2_TIMER_LOAD);
    DL_TimerG_setCounterMode(TB6612_ENB_INST, DL_TIMER_COUNT_MODE_UP);
    DL_TimerG_setCounterRepeatMode(TB6612_ENB_INST,
        DL_TIMER_REPEAT_MODE_ENABLED);
    DL_TimerG_setCounterValueAfterEnable(TB6612_ENB_INST,
        DL_TIMER_COUNT_AFTER_EN_ZERO);

    DL_TimerG_setCaptureCompareCtl(TB6612_ENB_INST,
        DL_TIMER_CC_MODE_CAPTURE,
        DL_TIMER_CC_ZCOND_NONE |
        DL_TIMER_CC_ACOND_TIMCLK |
        DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_EDGE,
        DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareInput(TB6612_ENB_INST,
        DL_TIMER_CC_INPUT_INV_NOINVERT,
        DL_TIMER_CC_IN_SEL_CCPX,
        DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareCtl(TB6612_ENB_INST,
        DL_TIMER_CC_MODE_CAPTURE,
        DL_TIMER_CC_ZCOND_NONE |
        DL_TIMER_CC_ACOND_TIMCLK |
        DL_TIMER_CAPTURE_EDGE_DETECTION_MODE_EDGE,
        DL_TIMER_CC_1_INDEX);
    DL_TimerG_setCaptureCompareInput(TB6612_ENB_INST,
        DL_TIMER_CC_INPUT_INV_NOINVERT,
        DL_TIMER_CC_IN_SEL_CCPX,
        DL_TIMER_CC_1_INDEX);

    DL_TimerG_setCaptureCompareInputFilter(TB6612_ENB_INST,
        DL_TIMER_CC_INPUT_FILT_CPV_CONSEC_PER,
        DL_TIMER_CC_INPUT_FILT_FP_PER_3,
        DL_TIMER_CC_0_INDEX);
    DL_TimerG_enableCaptureCompareInputFilter(TB6612_ENB_INST,
        DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareInputFilter(TB6612_ENB_INST,
        DL_TIMER_CC_INPUT_FILT_CPV_CONSEC_PER,
        DL_TIMER_CC_INPUT_FILT_FP_PER_3,
        DL_TIMER_CC_1_INDEX);
    DL_TimerG_enableCaptureCompareInputFilter(TB6612_ENB_INST,
        DL_TIMER_CC_1_INDEX);

    DL_TimerG_setCounterControl(TB6612_ENB_INST,
        DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_enableClock(TB6612_ENB_INST);
    DL_TimerG_enableEvent(TB6612_ENB_INST, DL_TIMERG_EVENT_ROUTE_1,
        DL_TIMERG_EVENT_CC0_DN_EVENT | DL_TIMERG_EVENT_CC0_UP_EVENT);
    DL_TimerG_setPublisherChanID(TB6612_ENB_INST,
        DL_TIMERG_PUBLISHER_INDEX_0, ENC2_EVT_CH_A);
    DL_TimerG_enableEvent(TB6612_ENB_INST, DL_TIMERG_EVENT_ROUTE_2,
        DL_TIMERG_EVENT_CC1_DN_EVENT | DL_TIMERG_EVENT_CC1_UP_EVENT);
    DL_TimerG_setPublisherChanID(TB6612_ENB_INST,
        DL_TIMERG_PUBLISHER_INDEX_1, ENC2_EVT_CH_B);
}

static void configure_dma_channel(uint8_t chan, uint8_t trigger,
    volatile uint32_t *src, uint32_t *dest)
{
    const DL_DMA_Config dmaConfig = {
        .transferMode = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
        .extendedMode = DL_DMA_NORMAL_MODE,
        .destIncrement = DL_DMA_ADDR_INCREMENT,
        .srcIncrement = DL_DMA_ADDR_UNCHANGED,
        .destWidth = DL_DMA_WIDTH_WORD,
        .srcWidth = DL_DMA_WIDTH_WORD,
        .trigger = trigger,
        .triggerType = DL_DMA_TRIGGER_TYPE_EXTERNAL
    };

    DL_DMA_disableChannel(DMA, chan);
    DL_DMA_initChannel(DMA, chan, &dmaConfig);
    DL_DMA_setSrcAddr(DMA, chan, (uint32_t) src);
    DL_DMA_setDestAddr(DMA, chan, (uint32_t) dest);
    DL_DMA_setTransferSize(DMA, chan, ENC2_BUF_SIZE);
}

static void configure_dma(void)
{
    configure_dma_channel(ENC2_DMA_A_CHAN_ID, DMA_CH0_TRIGGER_SEL_FSUB_0,
        &TIMG7->COUNTERREGS.CC_01[0], &g_enc2ABuf[0]);
    configure_dma_channel(ENC2_DMA_B_CHAN_ID, DMA_CH1_TRIGGER_SEL_FSUB_1,
        &TIMG7->COUNTERREGS.CC_01[1], &g_enc2BBuf[0]);

    DL_DMA_setSubscriberChanID(DMA, DL_DMA_SUBSCRIBER_INDEX_0, ENC2_EVT_CH_A);
    DL_DMA_setSubscriberChanID(DMA, DL_DMA_SUBSCRIBER_INDEX_1, ENC2_EVT_CH_B);

    DL_DMA_enableChannel(DMA, ENC2_DMA_A_CHAN_ID);
    DL_DMA_enableChannel(DMA, ENC2_DMA_B_CHAN_ID);
}

static uint8_t read_initial_ab_state(void)
{
    uint32_t raw = GPIOA->DIN31_0;
    uint8_t state = 0;

    if ((raw & ENC2_BIT_A) != 0U) {
        state |= ENC2_PHASE_A_MASK;
    }
    if ((raw & ENC2_BIT_B) != 0U) {
        state |= ENC2_PHASE_B_MASK;
    }
    return state;
}

void PORT_MOTOR_ENCODER2_Init(void)
{
    clear_buffers();

    s_aReadIndex = 0;
    s_bReadIndex = 0;
    s_firstPoll = true;
    s_diagCounter = 0;
    s_invalidTransitions = 0;
    s_overrunWarnings = 0;
    s_lastAEvents = 0;
    s_lastBEvents = 0;
    s_lastEqualTimestamps = 0;

    configure_timg7_capture();
    s_abState = read_initial_ab_state();
    configure_dma();

    DL_TimerG_startCounter(TB6612_ENB_INST);

    LOG_INFO("[ENC2] TIMG7 dual capture DMA initialized A=PA28 B=PA31 state=%u\r\n",
        s_abState);
}

static void apply_phase_event(uint8_t phaseMask, int32_t *pulseCount)
{
    uint8_t oldState = s_abState;
    uint8_t newState = oldState ^ phaseMask;
    int8_t step = s_qdecTable[((uint8_t)(oldState << 2)) | newState];

    if (step == 0) {
        s_invalidTransitions++;
    } else {
        *pulseCount += ((int32_t) step * ENC2_DIR_SIGN);
    }

    s_abState = newState;
}

static bool a_event_is_earlier(uint16_t aTs, uint16_t bTs)
{
    return ((int16_t)(bTs - aTs) >= 0);
}

void PORT_MOTOR_ENCODER2_Poll(int32_t *deltaPulses)
{
    uint32_t aWrite;
    uint32_t bWrite;
    uint32_t aPending;
    uint32_t bPending;
    uint16_t aEvents = 0;
    uint16_t bEvents = 0;
    uint16_t equalTimestamps = 0;
    int32_t pulseCount = 0;

    if (deltaPulses == NULL) {
        return;
    }

    aWrite = dma_write_offset(ENC2_DMA_A_CHAN_ID);
    bWrite = dma_write_offset(ENC2_DMA_B_CHAN_ID);

    if (s_firstPoll) {
        s_aReadIndex = aWrite;
        s_bReadIndex = bWrite;
        s_abState = read_initial_ab_state();
        s_firstPoll = false;
        *deltaPulses = 0;
        return;
    }

    aPending = ring_count(s_aReadIndex, aWrite);
    bPending = ring_count(s_bReadIndex, bWrite);

    if (aPending >= (ENC2_BUF_SIZE - 4U) ||
        bPending >= (ENC2_BUF_SIZE - 4U)) {
        s_overrunWarnings++;
    }

    while (s_aReadIndex != aWrite || s_bReadIndex != bWrite) {
        bool takeA;

        if (s_aReadIndex == aWrite) {
            takeA = false;
        } else if (s_bReadIndex == bWrite) {
            takeA = true;
        } else {
            uint16_t aTs = (uint16_t) g_enc2ABuf[s_aReadIndex];
            uint16_t bTs = (uint16_t) g_enc2BBuf[s_bReadIndex];

            if (aTs == bTs) {
                equalTimestamps++;
            }
            takeA = a_event_is_earlier(aTs, bTs);
        }

        if (takeA) {
            apply_phase_event(ENC2_PHASE_A_MASK, &pulseCount);
            aEvents++;
            s_aReadIndex++;
            if (s_aReadIndex >= ENC2_BUF_SIZE) {
                s_aReadIndex = 0;
            }
        } else {
            apply_phase_event(ENC2_PHASE_B_MASK, &pulseCount);
            bEvents++;
            s_bReadIndex++;
            if (s_bReadIndex >= ENC2_BUF_SIZE) {
                s_bReadIndex = 0;
            }
        }
    }

    s_lastAEvents = aEvents;
    s_lastBEvents = bEvents;
    s_lastEqualTimestamps = equalTimestamps;

    s_diagCounter++;
    if (s_diagCounter >= 100U) {
        s_diagCounter = 0;
        LOG_DEBUG("[ENC2] Aoff=%lu Boff=%lu AB=%u invalid=%lu overrun=%lu\r\n",
            (unsigned long) aWrite,
            (unsigned long) bWrite,
            (unsigned int) s_abState,
            (unsigned long) s_invalidTransitions,
            (unsigned long) s_overrunWarnings);
    }

    *deltaPulses = pulseCount;
}
