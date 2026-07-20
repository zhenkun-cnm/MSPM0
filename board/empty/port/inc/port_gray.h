/**
 * @file    port_gray.h
 * @brief   Port layer driver for the 8-channel grayscale line sensor.
 */
#ifndef PORT_GRAY_H
#define PORT_GRAY_H

#include "dev_gray.h"

#ifdef __cplusplus
extern "C" {
#endif

void PORT_GRAY_Init(void);
bool PORT_GRAY_SampleAll(Gray_Snapshot_t *out, uint32_t tick, uint32_t seq);
void PORT_GRAY_SetPolarity(Gray_Polarity_t polarity);
Gray_Polarity_t PORT_GRAY_GetPolarity(void);

#ifdef __cplusplus
}
#endif

#endif /* PORT_GRAY_H */
