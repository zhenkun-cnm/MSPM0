/**
 * @file    port_gray.c
 * @brief   Port layer driver for the 8-channel grayscale line sensor.
 */
#include "port_gray.h"
#include "ti_msp_dl_config.h"
#include <ti/driverlib/driverlib.h>

#define GRAY_AD0_PORT       Find_Block_AD0_PORT
#define GRAY_AD0_PIN        Find_Block_AD0_PIN
#define GRAY_AD1_PORT       Find_Block_AD1_PORT
#define GRAY_AD1_PIN        Find_Block_AD1_PIN
#define GRAY_AD2_PORT       Find_Block_AD2_PORT
#define GRAY_AD2_PIN        Find_Block_AD2_PIN
#define GRAY_OUT_PORT       Find_Block_OUT_PORT
#define GRAY_OUT_PIN        Find_Block_OUT_PIN

#define GRAY_SELECT_SETTLE_CYCLES 80U

static DevGray s_grayIf;
static Gray_Polarity_t s_polarity = GRAY_POLARITY_ACTIVE_HIGH;

static void gray_write_pin(GPIO_Regs *port, uint32_t pin, bool high)
{
    if (high) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static void gray_select_channel(uint8_t channel)
{
    gray_write_pin(GRAY_AD0_PORT, GRAY_AD0_PIN, ((channel & 0x01U) != 0U));
    gray_write_pin(GRAY_AD1_PORT, GRAY_AD1_PIN, ((channel & 0x02U) != 0U));
    gray_write_pin(GRAY_AD2_PORT, GRAY_AD2_PIN, ((channel & 0x04U) != 0U));
    delay_cycles(GRAY_SELECT_SETTLE_CYCLES);
}

static bool gray_read_out(void)
{
    return (DL_GPIO_readPins(GRAY_OUT_PORT, GRAY_OUT_PIN) != 0U) ? true : false;
}

void PORT_GRAY_Init(void)
{
    s_polarity = GRAY_POLARITY_ACTIVE_HIGH;
    gray_select_channel(0U);
}

bool PORT_GRAY_SampleAll(Gray_Snapshot_t *out, uint32_t tick, uint32_t seq)
{
    uint8_t rawMask = 0U;
    uint8_t activeMask = 0U;
    Gray_Polarity_t polarity = s_polarity;

    if (out == NULL) {
        return false;
    }

    for (uint8_t ch = 0U; ch < GRAY_CHANNEL_COUNT; ch++) {
        bool raw;
        bool active;

        gray_select_channel(ch);
        raw = gray_read_out();
        active = (polarity == GRAY_POLARITY_ACTIVE_HIGH) ? raw : !raw;

        out->raw[ch] = raw;
        out->active[ch] = active;

        if (raw) {
            rawMask |= (uint8_t)(1U << ch);
        }
        if (active) {
            activeMask |= (uint8_t)(1U << ch);
        }
    }

    out->raw_mask = rawMask;
    out->active_mask = activeMask;
    out->tick = tick;
    out->seq = seq;
    out->polarity = polarity;
    return true;
}

void PORT_GRAY_SetPolarity(Gray_Polarity_t polarity)
{
    if (polarity == GRAY_POLARITY_ACTIVE_LOW) {
        s_polarity = GRAY_POLARITY_ACTIVE_LOW;
    } else {
        s_polarity = GRAY_POLARITY_ACTIVE_HIGH;
    }
}

Gray_Polarity_t PORT_GRAY_GetPolarity(void)
{
    return s_polarity;
}

static void gray_init(DevGray *self)
{
    (void)self;
    PORT_GRAY_Init();
}

static bool gray_sample(DevGray *self, Gray_Snapshot_t *out, uint32_t tick, uint32_t seq)
{
    (void)self;
    return PORT_GRAY_SampleAll(out, tick, seq);
}

static void gray_set_polarity(DevGray *self, Gray_Polarity_t polarity)
{
    (void)self;
    PORT_GRAY_SetPolarity(polarity);
}

static Gray_Polarity_t gray_get_polarity(DevGray *self)
{
    (void)self;
    return PORT_GRAY_GetPolarity();
}

DevGray *GetGray(void)
{
    s_grayIf.init = gray_init;
    s_grayIf.sample = gray_sample;
    s_grayIf.setPolarity = gray_set_polarity;
    s_grayIf.getPolarity = gray_get_polarity;
    return &s_grayIf;
}
