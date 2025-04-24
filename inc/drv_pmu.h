/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_pmu.h
 * @brief       gd32e11x pmu driver for mds device
 * @date        2024-05-30
 */
#ifndef __DRV_PMU_H__
#define __DRV_PMU_H__

/* Include ----------------------------------------------------------------- */
#include "drv_chip.h"
#include "gd32e11x_pmu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function ---------------------------------------------------------------- */
void DRV_PMU_DeInit(void);

void DRV_PMU_EnableBkUpAccess(void);
void DRV_PMU_DisableBkUpAccess(void);

void DRV_PMU_EnableWakeUpPin(void);
void DRV_PMU_DisableWakeUpPin(void);

void DRV_PMU_EnterSleepMode(uint8_t sleepCmd);
void DRV_PMU_EnterDeepSleepMode(uint32_t ldo, uint8_t sleepCmd);
void DRV_PMU_EnterStandbyMode(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_PMU_H__ */
