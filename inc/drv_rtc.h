/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_rtc.h
 * @brief       gd32e11x rtc driver for mds device
 * @date        2024-05-30
 */
#ifndef __DRV_RTC_H__
#define __DRV_RTC_H__

/* Include ----------------------------------------------------------------- */
#include "drv_chip.h"
#include "gd32e11x_rtc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Typedef ----------------------------------------------------------------- */
typedef struct DRV_RTC_InitStruct {
    uint32_t prescale;
} DRV_RTC_InitStruct_t;

typedef struct DRV_RTC_Handle {
    uintptr_t RTCx;
} DRV_RTC_Handle_t;

/* Function ---------------------------------------------------------------- */
MDS_Err_t DRV_RTC_Init(DRV_RTC_Handle_t *hrtc, const DRV_RTC_InitStruct_t *init);
MDS_Err_t DRV_RTC_DeInit(DRV_RTC_Handle_t *hrtc);
uint32_t DRV_RTC_GetTimerCount(DRV_RTC_Handle_t *hrtc);
uint32_t DRV_RTC_GetAlarmCount(DRV_RTC_Handle_t *hrtc);
MDS_Err_t DRV_RTC_TimerAlarmStartINT(DRV_RTC_Handle_t *hrtc, uint32_t timer, uint32_t alarm);
MDS_Err_t DRV_RTC_AlarmTimerStop(DRV_RTC_Handle_t *hrtc);
void DRV_RTC_IRQHandler(DRV_RTC_Handle_t *hrtc);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_RTC_H__ */
