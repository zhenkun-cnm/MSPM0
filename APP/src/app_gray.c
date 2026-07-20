/**
 * @file    app_gray.c
 * @brief   App layer 8-channel grayscale sensor sampling task.
 */
#include "app_gray.h"
#include "port_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <string.h>

#define GRAY_TASK_PERIOD_MS 10U
#define GRAY_PRINT_PERIOD_MS 100U
#define GRAY_PERIODIC_PRINT_ENABLE 0

static Gray_Snapshot_t s_graySnapshot;
static DevGray *s_grayDev = NULL;

static const char *gray_polarity_name(Gray_Polarity_t polarity)
{
    return (polarity == GRAY_POLARITY_ACTIVE_LOW) ? "LOW" : "HIGH";
}

static void gray_print_scan_line(const Gray_Snapshot_t *snap)
{
    bool any = false;

    if (snap == NULL) {
        return;
    }

    LOG_RAW("[GRAY] seq=%lu active_ch=",
            (unsigned long)snap->seq);

    for (uint8_t i = 0U; i < GRAY_CHANNEL_COUNT; i++) {
        if (snap->active[i]) {
            LOG_RAW("%s%u", any ? "," : "", (unsigned)(i + 1U));
            any = true;
        }
    }

    if (!any) {
        LOG_RAW("none");
    }

    LOG_RAW(" raw=0x%02X active=0x%02X\r\n",
            (unsigned)snap->raw_mask,
            (unsigned)snap->active_mask);
}

bool Gray_ReadSnapshot(Gray_Snapshot_t *out)
{
    if (out == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    *out = s_graySnapshot;
    taskEXIT_CRITICAL();
    return (out->seq != 0U);
}

void Gray_SetPolarity(Gray_Polarity_t polarity)
{
    if (s_grayDev == NULL) {
        s_grayDev = GetGray();
    }

    if (s_grayDev != NULL) {
        s_grayDev->setPolarity(s_grayDev, polarity);
    }
}

Gray_Polarity_t Gray_GetPolarity(void)
{
    if (s_grayDev == NULL) {
        s_grayDev = GetGray();
    }

    if (s_grayDev == NULL) {
        return GRAY_POLARITY_ACTIVE_HIGH;
    }

    return s_grayDev->getPolarity(s_grayDev);
}

void Gray_PrintHelp(void)
{
    LOG_RAW("[GRAY] commands:\r\n");
    LOG_RAW("[GRAY]   gray status\r\n");
    LOG_RAW("[GRAY]   gray polarity high\r\n");
    LOG_RAW("[GRAY]   gray polarity low\r\n");
}

void Gray_PrintStatus(void)
{
    Gray_Snapshot_t snap;

    if (!Gray_ReadSnapshot(&snap)) {
        LOG_RAW("[GRAY] status: no sample yet polarity=%s\r\n",
                gray_polarity_name(Gray_GetPolarity()));
        return;
    }

    LOG_RAW("[GRAY] seq=%lu tick=%lu polarity=%s raw=0x%02X active=0x%02X\r\n",
            (unsigned long)snap.seq,
            (unsigned long)snap.tick,
            gray_polarity_name(snap.polarity),
            (unsigned)snap.raw_mask,
            (unsigned)snap.active_mask);
    LOG_RAW("[GRAY] raw ch1..8=%u%u%u%u%u%u%u%u active ch1..8=%u%u%u%u%u%u%u%u\r\n",
            snap.raw[0] ? 1U : 0U,
            snap.raw[1] ? 1U : 0U,
            snap.raw[2] ? 1U : 0U,
            snap.raw[3] ? 1U : 0U,
            snap.raw[4] ? 1U : 0U,
            snap.raw[5] ? 1U : 0U,
            snap.raw[6] ? 1U : 0U,
            snap.raw[7] ? 1U : 0U,
            snap.active[0] ? 1U : 0U,
            snap.active[1] ? 1U : 0U,
            snap.active[2] ? 1U : 0U,
            snap.active[3] ? 1U : 0U,
            snap.active[4] ? 1U : 0U,
            snap.active[5] ? 1U : 0U,
            snap.active[6] ? 1U : 0U,
            snap.active[7] ? 1U : 0U);
}

void gray_task(void *pvParameters)
{
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastPrint = xTaskGetTickCount();
    uint32_t seq = 0U;

    (void)pvParameters;

    memset(&s_graySnapshot, 0, sizeof(s_graySnapshot));

    s_grayDev = GetGray();
    if (s_grayDev == NULL) {
        LOG_ERROR("[GRAY] device handle NULL\r\n");
        vTaskDelete(NULL);
        return;
    }

    s_grayDev->init(s_grayDev);
    LOG_INFO("[GRAY] task started, period=%lums polarity=%s\r\n",
             (unsigned long)GRAY_TASK_PERIOD_MS,
             gray_polarity_name(s_grayDev->getPolarity(s_grayDev)));

    while (1) {
        Gray_Snapshot_t next;

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(GRAY_TASK_PERIOD_MS));

        if (s_grayDev->sample(s_grayDev,
                              &next,
                              (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),
                              ++seq)) {
            taskENTER_CRITICAL();
            s_graySnapshot = next;
            taskEXIT_CRITICAL();

#if GRAY_PERIODIC_PRINT_ENABLE
            if ((xTaskGetTickCount() - lastPrint) >= pdMS_TO_TICKS(GRAY_PRINT_PERIOD_MS)) {
                lastPrint = xTaskGetTickCount();
                gray_print_scan_line(&next);
            }
#else
            (void)lastPrint;
#endif
        }
    }
}
