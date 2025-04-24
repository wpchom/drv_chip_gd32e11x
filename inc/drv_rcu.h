/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_rcu.h
 * @brief       gd32e11x rcu driver for mds device
 * @date        2024-05-30
 */
#ifndef __DRV_RCU_H__
#define __DRV_RCU_H__

/* Include ----------------------------------------------------------------- */
#include "drv_chip.h"
#include "gd32e11x_rcu.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Define ------------------------------------------------------------------ */
// PLLState
#define RCU_PLL_NONE 0x00000000U
#define RCU_PLL_OFF  0x00000001U
#define RCU_PLL_ON   0x00000002U

// ClockType
#define RCU_CLOCKTYPE_SYSCLK 0x00000001U
#define RCU_CLOCKTYPE_HCLK   0x00000002U
#define RCU_CLOCKTYPE_PCLK1  0x00000004U
#define RCU_CLOCKTYPE_PCLK2  0x00000008U

// OscillatorType
#define RCU_OSCILLATORTYPE_NONE   0x00000000U
#define RCU_OSCILLATORTYPE_HXTAL  0x00000001U
#define RCU_OSCILLATORTYPE_IRC8M  0x00000002U
#define RCU_OSCILLATORTYPE_LXTAL  0x00000004U
#define RCU_OSCILLATORTYPE_IRC40K 0x00000008U
#define RCU_OSCILLATORTYPE_IRC48M 0x00000010U

// HXTALState
#define RCU_HXTAL_OFF    0x00000000U
#define RCU_HXTAL_ON     RCU_CTL_HXTALEN
#define RCU_HXTAL_BYPASS ((uint32_t)(RCU_CTL_HXTALBPS | RCU_CTL_HXTALEN))

// LXTALState
#define RCU_LXTAL_OFF    0x00000000U
#define RCU_LXTAL_ON     RCU_BDCTL_LXTALEN
#define RCU_LXTAL_BYPASS ((uint32_t)(RCU_BDCTL_LXTALBPS | RCU_BDCTL_LXTALEN))

// IRC8MState
#define RCU_IRC8M_OFF 0x00000000U
#define RCU_IRC8M_ON  RCU_CTL_IRC8MEN

#define RCU_IRC8MCALIB_DEFAULT 0x10U

// IRC40KState
#define RCU_IRC40K_OFF 0x00000000U
#define RCU_IRC40K_ON  RCU_RSTSCK_IRC40KEN

// IRC48MState
#define RCU_IRC48M_OFF 0x00000000U
#define RCU_IRC48M_ON  RCU_ADDCTL_IRC48MEN

/* Typedef ----------------------------------------------------------------- */
typedef struct DRV_DRV_RCU_PLLInit {
    uint32_t PLLState;
    uint32_t PLLSource;    /* PLL clock source selection */
    uint32_t PLLMUL;       /* PLL clock multiplication factor */
    uint32_t Predv0Source; /* PREDV0 input clock source selection */
    uint32_t Predv0Value;  /* PREDV0 division factor */
} DRV_RCU_PLLInit_t;

typedef struct DRV_RCU_PLL1Init {
    uint32_t PLL1State;
    uint32_t PLL1MUL; /* PLL1 clock multiplication factor */
} DRV_RCU_PLL1Init_t;

typedef struct DRV_RCU_PLL2Init {
    uint32_t PLL2State;
    uint32_t PLL2MUL; /* PLL2 clock multiplication factor */
} DRV_RCU_PLL2Init_t;

typedef struct DRV_RCU_OscInit {
    uint32_t OscillatorType;

    uint32_t HXTALState;

    uint32_t Predv1Value; /* PREDV1 division factor */

    uint32_t LXTALState;
    uint32_t IRC8MState;
    uint32_t IRC8MCalibValue;

    uint32_t IRC40KState;
    uint32_t IRC48MState;

    DRV_RCU_PLLInit_t PLL;
    DRV_RCU_PLL1Init_t PLL1;
} DRV_RCU_OscInit_t;

typedef struct DRV_RCU_ClockInit {
    uint32_t ClockType;
    uint32_t SYSCLKSource;   /* system clock source select status */
    uint32_t AHBCLKDivider;  /* AHB prescaler selection */
    uint32_t APB1CLKDivider; /* APB1 prescaler selection */
    uint32_t APB2CLKDivider; /* APB2 prescaler selection */
} DRV_RCU_ClockInit_t;

/* Function ---------------------------------------------------------------- */
/* system clock source select status */
static inline uint32_t DRV_RCU_GetSysClkSource(void)
{
    return (RCU_CFG0 & RCU_CFG0_SCSS);
}

static inline uint32_t DRV_RCU_GetPLLOscSource(void)
{
    return (RCU_CFG0 & RCU_CFG0_PLLSEL);
}

static inline uint32_t DRV_RCU_GetPLLPreSource(void)
{
    return (RCU_CFG1 & RCU_CFG1_PLLPRESEL);
}

MDS_Err_t DRV_RCU_HXTALConfig(uint32_t HXTALState);

MDS_Err_t DRV_RCU_IRC8MConfig(uint32_t IRC8MState, uint32_t IRC8MCalibValue);

MDS_Err_t DRV_RCU_IRC40KConfig(uint32_t IRC40KState);

MDS_Err_t DRV_RCU_LXTALConfig(uint32_t LXTALState);

MDS_Err_t DRV_RCU_IRC48MConfig(uint32_t IRC48MState);

MDS_Err_t DRV_RCU_PLLConfig(DRV_RCU_PLLInit_t *pllInitStruct);

MDS_Err_t DRV_RCU_PLL1Config(DRV_RCU_PLL1Init_t *pll1InitStruct, uint32_t predv1Value);

MDS_Err_t DRV_RCU_PLL2Config(DRV_RCU_PLL2Init_t *pll2InitStruct, uint32_t predv1Value);

MDS_Err_t DRV_RCU_OscConfig(DRV_RCU_OscInit_t *oscInitStruct);

MDS_Err_t DRV_RCU_ClockConfig(DRV_RCU_ClockInit_t *clkInitStruct);

uint32_t DRV_RCU_GetPLL1ClockFreq(void);

uint32_t DRV_RCU_GetPLL2ClockFreq(void);

uint32_t DRV_RCU_GetPLLClockFreq(void);

uint32_t DRV_RCU_GetSysClockFreq(void);

uint32_t DRV_RCU_GetCoreClockFreq(void);

MDS_Err_t DRV_RCU_DeInit(void);

/* RTC clock entry selection */
MDS_Err_t DRV_RCU_RTCClockSelection(uint32_t rtcClkSource);

static inline void DRV_RCU_RTCClockEnable(void)
{
    RCU_ADDCTL |= RCU_BDCTL_RTCEN;
}

static inline void DRV_RCU_RTCClockDisable(void)
{
    RCU_ADDCTL &= ~RCU_BDCTL_RTCEN;
}

#ifdef __cplusplus
}
#endif

#endif /* __DRV_RCU_H__ */
