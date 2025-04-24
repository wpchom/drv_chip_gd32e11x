/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_gpio.h
 * @brief       stm32f1xx gpio driver for mds device
 * @date        2024-05-30
 */
#ifndef __DRV_GPIO_H__
#define __DRV_GPIO_H__

/* Include ----------------------------------------------------------------- */
#include "dev_gpio.h"
#include "gd32e11x_gpio.h"
#include "gd32e11x_exti.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Function ---------------------------------------------------------------- */
MDS_Err_t DRV_GPIO_PinConfig(uintptr_t GPIOx, uint32_t GPIO_Pin, const DEV_GPIO_Config_t *config);

uint32_t DRV_GPIO_PortReadInput(uintptr_t GPIOx);
uint32_t DRV_GPIO_PortReadOutput(uintptr_t GPIOx);
void DRV_GPIO_PortWrite(uintptr_t GPIOx, uint32_t val);
void DRV_GPIO_PinHigh(uintptr_t GPIOx, uint32_t GPIO_Pin);
void DRV_GPIO_PinLow(uintptr_t GPIOx, uint32_t GPIO_Pin);
void DRV_GPIO_PinToggle(uintptr_t GPIOx, uint32_t GPIO_Pin);

void DRV_GPIO_PinIRQHandler(DEV_GPIO_Object_t *object);

/* Driver ------------------------------------------------------------------ */
extern const DEV_GPIO_Driver_t G_DRV_STM32F1XX_GPIO;

#ifdef __cplusplus
}
#endif

#endif /* __DRV_GPIO_H__ */
