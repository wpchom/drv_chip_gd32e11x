/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_gpio.c
 * @brief       stm32f1xx gpio driver for mds device
 * @date        2024-05-30
 */
/* Include ----------------------------------------------------------------- */
#include "drv_gpio.h"
#include "mds_log.h"

/* Function ---------------------------------------------------------------- */
static void DRV_GPIO_PinWrite(uintptr_t GPIOx, uint32_t GPIO_Pin, uint32_t val)
{
    uint32_t shift = __CLZ(__RBIT(GPIO_Pin));
    if (shift < 0x10U) {
        uint32_t set = (val << shift) & GPIO_Pin;
        uint32_t clr = (~set) & GPIO_Pin;
        GPIO_BOP(GPIOx) = (set << 0x10U) | clr;
    }
}

MDS_Err_t DRV_GPIO_ExtiConfig(uintptr_t GPIOx, uint32_t GPIO_Pin, DEV_GPIO_Interrupt_t intr)
{
    RCU_APB2EN |= RCU_APB2EN_AFEN;

    uint32_t portIndex = ((GPIOx - GPIOA) / (GPIOB - GPIOA)) & 0x0FU;

    for (uint32_t pinIndex = 0; (1UL << pinIndex) < (GPIO_Pin & 0x0000FFFFU); pinIndex += 1) {
        if ((GPIO_Pin & (1UL << pinIndex)) == 0x00U) {
            continue;
        }

        // AFIO_EXTISSx
        volatile uint32_t *AFIO_EXTISS = (volatile uint32_t *)(AFIO + 0x08U + sizeof(uintptr_t) * (pinIndex >> 0x02U));
        uint32_t regIndex = (*AFIO_EXTISS >> (0x04U * (pinIndex & 0x03U))) & 0x0FU;
        if ((intr == DEV_GPIO_INTR_NONE) && (regIndex != portIndex)) {
            continue;
        }

        EXTI_INTEN &= ~(1UL << pinIndex);
        EXTI_RTEN &= ~(1UL << pinIndex);
        EXTI_FTEN &= ~(1UL << pinIndex);

        uint32_t regTmp = *AFIO_EXTISS & (~(0x0FU << (4U * (pinIndex & 0x03U))));
        *AFIO_EXTISS = regTmp | (portIndex << (4U * (pinIndex & 0x03U)));

        if ((intr & DEV_GPIO_INTR_RISING) != 0x00U) {
            EXTI_RTEN |= 1UL << pinIndex;
        }
        if ((intr & DEV_GPIO_INTR_FALLING) != 0x00U) {
            EXTI_FTEN |= 1UL << pinIndex;
        }
        if ((intr & DEV_GPIO_INTR_BOTH) != 0x00U) {
            EXTI_INTEN |= 1UL << pinIndex;
        }
    }

    return (MDS_EOK);
}

MDS_Err_t DRV_GPIO_PinConfig(uintptr_t GPIOx, uint32_t GPIO_Pin, const DEV_GPIO_Config_t *config)
{
    MDS_ASSERT(config != NULL);

    uint32_t mode = 0x00U;

    if (config->mode == DEV_GPIO_MODE_INPUT) {
        if ((config->type == DEV_GPIO_TYPE_OD) || (config->type == DEV_GPIO_TYPE_PP_NO)) {
            mode = 0x04;  // 0100
        } else {
            mode = 0x08;  // 1000
        }
    } else if (config->mode == DEV_GPIO_MODE_OUTPUT) {
        if (config->type == DEV_GPIO_TYPE_OD) {
            mode = 0x05;  // 0101
        } else {
            mode = 0x01;  // 0001
        }
    } else if (config->mode == DEV_GPIO_MODE_ALTERNATE) {
        if (config->type == DEV_GPIO_TYPE_OD) {
            mode = 0x0F;  // 1111
        } else {
            mode = 0x0B;  // 1011
        }
    } else {          // DEV_GPIO_MODE_ANALOG
        mode = 0x00;  // 0000
    }

    for (uint32_t pinIndex = 0; (1UL << pinIndex) < (GPIO_Pin & 0x0000FFFFU); pinIndex += 1) {
        if ((GPIO_Pin & (1UL << pinIndex)) == 0x00U) {
            continue;
        }
        if (pinIndex < 0x08U) {
            uint32_t regCtl0 = GPIO_CTL0(GPIOx) & (~(0xFU << (4U * pinIndex)));
            GPIO_CTL0(GPIOx) = regCtl0 | (mode << (4U * pinIndex));
        } else {
            uint32_t regCtl1 = GPIO_CTL1(GPIOx) & (~(0xFU << (4U * (pinIndex - 0x08U))));
            GPIO_CTL1(GPIOx) = regCtl1 | (mode << (4U * (pinIndex - 0x08U)));
        }
        if (config->mode == DEV_GPIO_MODE_INPUT) {
            GPIO_BOP(GPIOx) = 1UL << (pinIndex + ((config->type == DEV_GPIO_TYPE_PP_DOWN) ? (0x08U) : (0x00U)));
        }
    }

    return (DRV_GPIO_ExtiConfig(GPIOx, GPIO_Pin, config->intr));
}

uint32_t DRV_GPIO_PortReadInput(uintptr_t GPIOx)
{
    return (GPIO_ISTAT(GPIOx));
}

uint32_t DRV_GPIO_PortReadOutput(uintptr_t GPIOx)
{
    return (GPIO_OCTL(GPIOx));
}

void DRV_GPIO_PortWrite(uintptr_t GPIOx, uint32_t val)
{
    GPIO_OCTL(GPIOx) = val;
}

void DRV_GPIO_PinHigh(uintptr_t GPIOx, uint32_t GPIO_Pin)
{
    GPIO_BOP(GPIOx) = GPIO_Pin;
}

void DRV_GPIO_PinLow(uintptr_t GPIOx, uint32_t GPIO_Pin)
{
    GPIO_BC(GPIOx) = GPIO_Pin;
}

void DRV_GPIO_PinToggle(uintptr_t GPIOx, uint32_t GPIO_Pin)
{
    MDS_Item_t lock = MDS_CoreInterruptLock();
    uint32_t output = DRV_GPIO_PortReadOutput(GPIOx);
    uint32_t clr = output & GPIO_Pin;
    uint32_t set = (~output) & GPIO_Pin;
    GPIO_BOP(GPIOx) = (clr << 0x10U) | set;
    MDS_CoreInterruptRestore(lock);
}

void DRV_GPIO_PinIRQHandler(DEV_GPIO_Object_t *object)
{
    if ((EXTI_PD & object->pinMask) != 0x00U) {
        EXTI_PD = object->pinMask;

        DEV_GPIO_Pin_t *pin = (DEV_GPIO_Pin_t *)(object->parent);
        if ((pin != NULL) && (pin->callback != NULL)) {
            pin->callback(pin, pin->arg);
        }
    }
}

/* Driver ------------------------------------------------------------------ */
static MDS_Err_t DDRV_GPIO_PortControl(const DEV_GPIO_Module_t *gpio, MDS_Item_t cmd, MDS_Arg_t *arg)
{
    MDS_ASSERT(gpio != NULL);

    switch (cmd) {
        case MDS_DEVICE_CMD_INIT:
        case MDS_DEVICE_CMD_DEINIT:
        case MDS_DEVICE_CMD_HANDLESZ:
            return (MDS_EOK);
    }

    if (cmd == DEV_GPIO_CMD_PIN_TOGGLE) {
        DEV_GPIO_Pin_t *pin = (DEV_GPIO_Pin_t *)arg;
        DRV_GPIO_PinToggle((uintptr_t)(pin->object.GPIOx), pin->object.pinMask);
        return (MDS_EOK);
    }

    return (MDS_EACCES);
}

static MDS_Err_t DDRV_GPIO_PinConfig(const DEV_GPIO_Pin_t *pin, const DEV_GPIO_Config_t *config)
{
    uintptr_t GPIOx = (uintptr_t)(pin->object.GPIOx);

    if (config->mode == DEV_GPIO_MODE_OUTPUT) {
        DRV_GPIO_PinWrite(GPIOx, pin->object.pinMask, pin->object.initVal);
    }

    return (DRV_GPIO_PinConfig(GPIOx, pin->object.pinMask, config));
}

static MDS_Mask_t DDRV_GPIO_PinRead(const DEV_GPIO_Pin_t *pin, bool input)
{
    uintptr_t GPIOx = (uintptr_t)(pin->object.GPIOx);

    MDS_Mask_t read = (input) ? (DRV_GPIO_PortReadInput(GPIOx)) : (DRV_GPIO_PortReadOutput(GPIOx));

    return ((read & pin->object.pinMask) >> __CLZ(__RBIT(pin->object.pinMask)));
}

static void DDRV_GPIO_PinWrite(const DEV_GPIO_Pin_t *pin, MDS_Mask_t val)
{
    uintptr_t GPIOx = (uintptr_t)(pin->object.GPIOx);

    DRV_GPIO_PinWrite(GPIOx, pin->object.pinMask, val);
}

const DEV_GPIO_Driver_t G_DRV_GD32E11X_GPIO = {
    .control = DDRV_GPIO_PortControl,
    .config = DDRV_GPIO_PinConfig,
    .read = DDRV_GPIO_PinRead,
    .write = DDRV_GPIO_PinWrite,
};
