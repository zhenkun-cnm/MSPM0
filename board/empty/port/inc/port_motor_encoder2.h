/**
 * @file    port_motor_encoder2.h
 * @brief   Motor2 encoder capture interface.
 *
 * Hardware path is configured in port_motor_encoder2.c without changing
 * empty.syscfg:
 *   A phase: PA28 / TIMG7_CCP0 -> DMA timestamp buffer
 *   B phase: PA31 / TIMG7_CCP1 -> DMA timestamp buffer
 */
#ifndef PORT_MOTOR_ENCODER2_H
#define PORT_MOTOR_ENCODER2_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENC2_BUF_SIZE  512U
#define ENC2_BIT_A     (1UL << 28)   /* PA28 = A phase */
#define ENC2_BIT_B     (1UL << 31)   /* PA31 = B phase */

typedef struct {
    uint32_t a_dma_offset;
    uint32_t b_dma_offset;
    uint8_t ab_state;
    uint16_t last_a_events;
    uint16_t last_b_events;
    uint16_t last_equal_timestamps;
    uint32_t invalid_transitions;
    uint32_t overrun_warnings;
} PortMotorEncoder2Diag_t;

void PORT_MOTOR_ENCODER2_Init(void);
void PORT_MOTOR_ENCODER2_RequestResync(void);
void PORT_MOTOR_ENCODER2_Poll(int32_t *deltaPulses);
uint32_t PORT_MOTOR_ENCODER2_GetDmaWriteOffset(void);
bool PORT_MOTOR_ENCODER2_GetDiag(PortMotorEncoder2Diag_t *out);

#ifdef __cplusplus
}
#endif

#endif /* PORT_MOTOR_ENCODER2_H */
