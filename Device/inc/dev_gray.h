/**
 * @file    dev_gray.h
 * @brief   8-channel grayscale line sensor Device interface.
 */
#ifndef DEV_GRAY_H
#define DEV_GRAY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GRAY_CHANNEL_COUNT 8U

typedef enum {
    GRAY_POLARITY_ACTIVE_HIGH = 0,
    GRAY_POLARITY_ACTIVE_LOW  = 1
} Gray_Polarity_t;

typedef struct {
    uint8_t raw_mask;
    uint8_t active_mask;
    bool raw[GRAY_CHANNEL_COUNT];
    bool active[GRAY_CHANNEL_COUNT];
    uint32_t tick;
    uint32_t seq;
    Gray_Polarity_t polarity;
} Gray_Snapshot_t;

typedef struct DevGray {
    void (*init)(struct DevGray *self);
    bool (*sample)(struct DevGray *self, Gray_Snapshot_t *out, uint32_t tick, uint32_t seq);
    void (*setPolarity)(struct DevGray *self, Gray_Polarity_t polarity);
    Gray_Polarity_t (*getPolarity)(struct DevGray *self);
} DevGray;

DevGray *GetGray(void);

#ifdef __cplusplus
}
#endif

#endif /* DEV_GRAY_H */
