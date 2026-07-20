/**
 * @file    app_gray.h
 * @brief   App layer 8-channel grayscale sensor sampling task.
 */
#ifndef APP_GRAY_H
#define APP_GRAY_H

#include "dev_gray.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bool Gray_ReadSnapshot(Gray_Snapshot_t *out);
void Gray_SetPolarity(Gray_Polarity_t polarity);
Gray_Polarity_t Gray_GetPolarity(void);
void Gray_PrintHelp(void);
void Gray_PrintStatus(void);
void gray_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* APP_GRAY_H */
