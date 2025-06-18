/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_rtc.c
 * @brief       gd32e11x rtc driver for mds device
 * @date        2024-05-30
 */
/* Include ----------------------------------------------------------------- */
#include "drv_rtc.h"

/* Define ------------------------------------------------------------------ */
#define RTC_TIMEOUT_VALUE 1000U

/* Typedef ----------------------------------------------------------------- */
typedef struct {
    __IO uint32_t INTEN;
    __IO uint32_t CTL;
    __IO uint32_t PSCH;
    __IO uint32_t PSCL;
    __IO uint32_t DIVH;
    __IO uint32_t DIVL;
    __IO uint32_t CNTH;
    __IO uint32_t CNTL;
    __IO uint32_t ALRMH;
    __IO uint32_t ALRML;
} RTC_TypeDef;

/* Function ---------------------------------------------------------------- */
static MDS_Err_t RTC_WaitForSync(RTC_TypeDef *RTCx)
{
    RTCx->CTL &= ~RTC_CTL_RSYNF;
    for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RTCx->CTL & RTC_CTL_RSYNF) == 0U;) {
        if ((DRV_CHIP_GetTick() - tickstart) > RTC_TIMEOUT_VALUE) {
            return (MDS_ETIMEOUT);
        }
    }

    return (MDS_EOK);
}

static MDS_Err_t RTC_WaitLastOperation(RTC_TypeDef *RTCx)
{
    for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RTCx->CTL & RTC_CTL_LWOFF) == 0U;) {
        if ((DRV_CHIP_GetTick() - tickstart) > RTC_TIMEOUT_VALUE) {
            return (MDS_ETIMEOUT);
        }
    }

    return (MDS_EOK);
}

MDS_Err_t DRV_RTC_Init(DRV_RTC_Handle_t *hrtc, const DRV_RTC_InitStruct_t *init)
{
    MDS_ASSERT(hrtc != NULL);

    MDS_Err_t err;
    RTC_TypeDef *RTCx = (RTC_TypeDef *)(hrtc->RTCx);

    do {
        err = RTC_WaitForSync(RTCx);
        if (err != MDS_EOK) {
            break;
        }

        err = RTC_WaitLastOperation(RTCx);
        if (err != MDS_EOK) {
            break;
        }

        RTCx->CTL |= RTC_CTL_CMF;

        RTCx->CTL &= ~(RTC_CTL_OVIF | RTC_CTL_ALRMIF | RTC_CTL_SCIF);

        RTCx->PSCH = (init->prescale >> 0x10U) & RTC_PSCH_PSC;
        RTCx->PSCL = (init->prescale >> 0x00U) & RTC_PSCL_PSC;

        RTCx->CTL &= ~RTC_CTL_CMF;

        err = RTC_WaitLastOperation(RTCx);
    } while (0);

    return (err);
}

MDS_Err_t DRV_RTC_DeInit(DRV_RTC_Handle_t *hrtc)
{
    MDS_ASSERT(hrtc != NULL);

    MDS_Err_t err;
    RTC_TypeDef *RTCx = (RTC_TypeDef *)(hrtc->RTCx);

    do {
        err = RTC_WaitLastOperation(RTCx);
        if (err != MDS_EOK) {
            break;
        }
        RTCx->CTL |= RTC_CTL_CMF;

        RTCx->CNTL = 0x00U;
        RTCx->CNTH = 0x00U;
        RTCx->PSCL = 0x00U;
        RTCx->PSCH = 0x00U;

        RTCx->INTEN = 0x00U;
        RTCx->CTL = 0x00U;

        RTCx->CTL &= ~RTC_CTL_CMF;
        err = RTC_WaitLastOperation(RTCx);
        if (err != MDS_EOK) {
            break;
        }

        err = RTC_WaitForSync(RTCx);
    } while (0);

    return (err);
}

uint32_t DRV_RTC_GetTimerCount(DRV_RTC_Handle_t *hrtc)
{
    MDS_ASSERT(hrtc != NULL);

    RTC_TypeDef *RTCx = (RTC_TypeDef *)(hrtc->RTCx);

    uint32_t count = ((uint32_t)RTCx->CNTH << 0x10U) | RTCx->CNTL;

    return (count);
}
uint32_t DRV_RTC_GetAlarmCount(DRV_RTC_Handle_t *hrtc)
{
    MDS_ASSERT(hrtc != NULL);

    RTC_TypeDef *RTCx = (RTC_TypeDef *)(hrtc->RTCx);

    uint32_t count = ((uint32_t)RTCx->ALRMH << 0x10U) | RTCx->ALRML;

    return (count);
}

MDS_Err_t DRV_RTC_TimerAlarmStartINT(DRV_RTC_Handle_t *hrtc, uint32_t timer, uint32_t alarm)
{
    MDS_ASSERT(hrtc != NULL);

    MDS_Err_t err;
    RTC_TypeDef *RTCx = (RTC_TypeDef *)(hrtc->RTCx);

    RTCx->INTEN &= ~RTC_INTEN_ALRMIE;
    RTCx->CTL &= ~RTC_CTL_ALRMIF;
    EXTI_INTEN &= ~EXTI_INTEN_INTEN17;
    EXTI_RTEN &= ~EXTI_RTEN_RTEN17;
    EXTI_FTEN &= ~EXTI_FTEN_FTEN17;

    do {
        err = RTC_WaitLastOperation(RTCx);
        if (err != MDS_EOK) {
            break;
        }
        RTCx->CTL |= RTC_CTL_CMF;

        RTCx->CNTH = (timer >> 0x10U) & RTC_CNTH_CNT;
        RTCx->CNTL = (timer >> 0x00U) & RTC_CNTL_CNT;

        RTCx->ALRMH = ((timer + alarm) >> 0x10U) & RTC_ALRMH_ALRM;
        RTCx->ALRML = ((timer + alarm) >> 0x00U) & RTC_ALRML_ALRM;

        RTCx->CTL &= ~RTC_CTL_CMF;
        err = RTC_WaitLastOperation(RTCx);
        if (err != MDS_EOK) {
            break;
        }
    } while (0);

    if (err == MDS_EOK) {
        EXTI_RTEN |= EXTI_RTEN_RTEN17;
        EXTI_INTEN |= EXTI_INTEN_INTEN17;
        RTCx->INTEN |= RTC_INTEN_ALRMIE;
    }

    return (err);
}

MDS_Err_t DRV_RTC_AlarmTimerStop(DRV_RTC_Handle_t *hrtc)
{
    MDS_ASSERT(hrtc != NULL);

    RTC_TypeDef *RTCx = (RTC_TypeDef *)(hrtc->RTCx);

    RTCx->INTEN &= ~RTC_INTEN_ALRMIE;
    RTCx->CTL &= ~RTC_CTL_ALRMIF;
    EXTI_INTEN &= ~EXTI_INTEN_INTEN17;
    EXTI_RTEN &= ~EXTI_RTEN_RTEN17;
    EXTI_FTEN &= ~EXTI_FTEN_FTEN17;

    return (MDS_EOK);
}

void DRV_RTC_IRQHandler(DRV_RTC_Handle_t *hrtc)
{
    MDS_ASSERT(hrtc != NULL);

    RTC_TypeDef *RTCx = (RTC_TypeDef *)(hrtc->RTCx);

    uint32_t inten = RTCx->INTEN;
    uint32_t ctl = RTCx->CTL;

    if (((inten & RTC_INTEN_ALRMIE) != 0U) && ((ctl & RTC_CTL_ALRMIF) != 0U)) {
    }
    if (((inten & RTC_INTEN_OVIE) != 0U) && ((ctl & RTC_CTL_OVIF) != 0U)) {
    }
    if (((inten & RTC_INTEN_SCIE) != 0U) && ((ctl & RTC_CTL_SCIF) != 0U)) {
    }

    RTCx->CTL &= ~(RTC_CTL_ALRMIF | RTC_CTL_OVIF | RTC_CTL_SCIF);
}
