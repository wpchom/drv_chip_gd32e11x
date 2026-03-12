/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_chip.h
 * @brief       gd32e11x chip driver for mds device
 * @date        2024-05-30
 */
#ifndef __DRV_CHIP_H__
#define __DRV_CHIP_H__

/* Include ----------------------------------------------------------------- */
#include "mds_sys.h"
#include "gd32e11x.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SET_BIT(REG, BIT)   ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT) ((REG) &= ~(BIT))
#define READ_BIT(REG, BIT)  ((REG) & (BIT))
#define CLEAR_REG(REG)      ((REG) = (0x0))
#define WRITE_REG(REG, VAL) ((REG) = (VAL))
#define READ_REG(REG)       ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)                                                       \
    WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))

/* Function ---------------------------------------------------------------- */
static inline MDS_Tick_t DRV_CHIP_GetTick(void)
{
    uint32_t count = 0;

    if ((DCB->DEMCR & DCB_DEMCR_TRCENA_Msk) == 0U) {
        MDS_Lock_t lock = MDS_CriticalLock(NULL);
        DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        MDS_CriticalRestore(NULL, lock);
    } else {
        count = DWT->CYCCNT / (SystemCoreClock / CONFIG_MDS_CLOCK_TICK_FREQ_HZ);
    }

    return (count);
}

void DRV_CHIP_JumpIntoVectorAddress(uintptr_t vectorAddress);
void DRV_CHIP_JumpIntoDFU(void);
void DRV_CHIP_SystemReset(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CHIP_H__ */
