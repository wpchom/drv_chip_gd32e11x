/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_pmu.c
 * @brief       gd32e11x pmu driver for mds device
 * @date        2024-05-30
 */
/* Include ----------------------------------------------------------------- */
#include "drv_pmu.h"

/* Function ---------------------------------------------------------------- */
void DRV_PMU_DeInit(void)
{
    RCU_APB1RST |= RCU_APB1RST_PMURST;
    RCU_APB1RST &= ~RCU_APB1RST_PMURST;
}

void DRV_PMU_EnableBkUpAccess(void)
{
    PMU_CTL |= PMU_CTL_BKPWEN;
}

void DRV_PMU_DisableBkUpAccess(void)
{
    PMU_CTL &= ~PMU_CTL_BKPWEN;
}

void DRV_PMU_EnableWakeUpPin(void)
{
    PMU_CS |= PMU_CS_WUPEN;
}

void DRV_PMU_DisableWakeUpPin(void)
{
    PMU_CS &= ~PMU_CS_WUPEN;
}

void DRV_PMU_EnterSleepMode(uint8_t sleepCmd)
{
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

    if (sleepCmd == WFI_CMD) {
        __WFI();
    } else {
        __SEV();
        __WFE();
        __WFE();
    }
}

void DRV_PMU_EnterDeepSleepMode(uint32_t ldo, uint8_t sleepCmd)
{
    uint32_t reg_snap[0x4U];

    PMU_CTL &= ~((uint32_t)(PMU_CTL_STBMOD | PMU_CTL_LDOLP));

    PMU_CTL |= ldo;

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    reg_snap[0] = REG32(0xE000E010U);
    reg_snap[1] = REG32(0xE000E100U);
    reg_snap[2] = REG32(0xE000E104U);
    reg_snap[3] = REG32(0xE000E108U);

    REG32(0xE000E010U) &= 0x00010004U;
    REG32(0xE000E180U) = 0XFF7FF83DU;
    REG32(0xE000E184U) = 0XFFFFF8FFU;
    REG32(0xE000E188U) = 0xFFFFFFFFU;

    if (WFI_CMD == sleepCmd) {
        __WFI();
    } else {
        __SEV();
        __WFE();
        __WFE();
    }

    REG32(0xE000E010U) = reg_snap[0x0];
    REG32(0xE000E100U) = reg_snap[0x1];
    REG32(0xE000E104U) = reg_snap[0x2];
    REG32(0xE000E108U) = reg_snap[0x3];

    SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
}

void DRV_PMU_EnterStandbyMode(void)
{
    PMU_CTL |= PMU_CTL_STBMOD;

    PMU_CTL |= PMU_CTL_WURST;

    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    REG32(0xE000E010U) &= 0x00010004U;
    REG32(0xE000E180U) = 0XFFFFFFF7U;
    REG32(0xE000E184U) = 0XFFFFFDFFU;
    REG32(0xE000E188U) = 0xFFFFFFFFU;

    __WFI();
}

