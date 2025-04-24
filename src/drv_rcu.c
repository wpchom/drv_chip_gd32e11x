/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        drv_rcu.c
 * @brief       gd32e11x rcu driver for mds device
 * @date        2024-05-30
 */
/* Include ----------------------------------------------------------------- */
#include "drv_rcu.h"

/* Define ------------------------------------------------------------------ */
#define DBP_TIMEOUT_VALUE         100U
#define PLL_TIMEOUT_VALUE         2U
#define HXTAL_TIMEOUT_VALUE       100U
#define IRC8M_TIMEOUT_VALUE       2U
#define IRC40K_TIMEOUT_VALUE      2U
#define LXTAL_TIMEOUT_VALUE       5000U
#define CLOCKSWITCH_TIMEOUT_VALUE 5000U
#define IRC48M_TIMEOUT_VALUE      2U

#define IS_RCU_HXTAL_STATE(HXTALState)                                                                                 \
    ((HXTALState == RCU_HXTAL_OFF) || (HXTALState == RCU_HXTAL_ON) || (HXTALState == RCU_HXTAL_BYPASS))

#define IS_RCU_IRC8M_STATE(IRC8MState) ((IRC8MState == RCU_IRC8M_OFF) || (IRC8MState == RCU_IRC8M_ON))

#define IS_RCU_IRC40K_STATE(IRC40KState) ((IRC40KState == RCU_IRC40K_OFF) || (IRC40KState == RCU_IRC40K_ON))

#define IS_RCU_LXTAL_STATE(LXTALState)                                                                                 \
    ((LXTALState == RCU_LXTAL_OFF) || (LXTALState == RCU_LXTAL_ON) || (LXTALState == RCU_LXTAL_BYPASS))

#define IS_RCU_IRC48M_STATE(IRC48MState) ((IRC48MState == RCU_IRC48M_OFF) || (IRC48MState == RCU_IRC48M_ON))

#define IS_RCU_PLL_STATE(PLLState) ((PLLState == RCU_PLL_NONE) || (PLLState == RCU_PLL_ON) || (PLLState == RCU_PLL_OFF))

#define IS_RCU_PLL_SOURCE(PLLSource) ((PLLSource == RCU_PLLSRC_IRC8M_DIV2) || (PLLSource == RCU_PLLSRC_HXTAL_IRC48M))

#define IS_RCU_PLL_MUL(PLLMul)                                                                                         \
    ((PLLMul == RCU_PLL_MUL2) || (PLLMul == RCU_PLL_MUL3) || (PLLMul == RCU_PLL_MUL4) || (PLLMul == RCU_PLL_MUL5) ||   \
     (PLLMul == RCU_PLL_MUL6) || (PLLMul == RCU_PLL_MUL7) || (PLLMul == RCU_PLL_MUL8) || (PLLMul == RCU_PLL_MUL9) ||   \
     (PLLMul == RCU_PLL_MUL10) || (PLLMul == RCU_PLL_MUL11) || (PLLMul == RCU_PLL_MUL12) ||                            \
     (PLLMul == RCU_PLL_MUL13) || (PLLMul == RCU_PLL_MUL14) || (PLLMul == RCU_PLL_MUL6_5) ||                           \
     (PLLMul == RCU_PLL_MUL16) || (PLLMul == RCU_PLL_MUL17) || (PLLMul == RCU_PLL_MUL18) ||                            \
     (PLLMul == RCU_PLL_MUL19) || (PLLMul == RCU_PLL_MUL20) || (PLLMul == RCU_PLL_MUL21) ||                            \
     (PLLMul == RCU_PLL_MUL22) || (PLLMul == RCU_PLL_MUL23) || (PLLMul == RCU_PLL_MUL24) ||                            \
     (PLLMul == RCU_PLL_MUL25) || (PLLMul == RCU_PLL_MUL26) || (PLLMul == RCU_PLL_MUL27) ||                            \
     (PLLMul == RCU_PLL_MUL28) || (PLLMul == RCU_PLL_MUL29) || (PLLMul == RCU_PLL_MUL30) || (PLLMul == RCU_PLL_MUL31))

#define IS_RCU_PREDV0_SOURCE(Predv0Source)                                                                             \
    ((Predv0Source == RCU_PREDV0SRC_HXTAL_IRC48M) || (Predv0Source == RCU_PREDV0SRC_CKPLL1))

#define IS_RCU_PREDV0_VALUE(Predv0Value)                                                                               \
    ((Predv0Value == RCU_PREDV0_DIV1) || (Predv0Value == RCU_PREDV0_DIV2) || (Predv0Value == RCU_PREDV0_DIV3) ||       \
     (Predv0Value == RCU_PREDV0_DIV4) || (Predv0Value == RCU_PREDV0_DIV5) || (Predv0Value == RCU_PREDV0_DIV6) ||       \
     (Predv0Value == RCU_PREDV0_DIV7) || (Predv0Value == RCU_PREDV0_DIV8) || (Predv0Value == RCU_PREDV0_DIV9) ||       \
     (Predv0Value == RCU_PREDV0_DIV10) || (Predv0Value == RCU_PREDV0_DIV11) || (Predv0Value == RCU_PREDV0_DIV12) ||    \
     (Predv0Value == RCU_PREDV0_DIV13) || (Predv0Value == RCU_PREDV0_DIV14) || (Predv0Value == RCU_PREDV0_DIV15) ||    \
     (Predv0Value == RCU_PREDV0_DIV16))

#define IS_RCU_PLL1_MUL(PLL1Mul)                                                                                       \
    ((PLL1Mul == RCU_PLL1_MUL8) || (PLL1Mul == RCU_PLL1_MUL9) || (PLL1Mul == RCU_PLL1_MUL10) ||                        \
     (PLL1Mul == RCU_PLL1_MUL11) || (PLL1Mul == RCU_PLL1_MUL12) || (PLL1Mul == RCU_PLL1_MUL13) ||                      \
     (PLL1Mul == RCU_PLL1_MUL14) || (PLL1Mul == RCU_PLL1_MUL16) || (PLL1Mul == RCU_PLL1_MUL20))

#define IS_RCU_PREDV1_VALUE(Predv1Value)                                                                               \
    ((Predv1Value == RCU_PREDV1_DIV1) || (Predv1Value == RCU_PREDV1_DIV2) || (Predv1Value == RCU_PREDV1_DIV3) ||       \
     (Predv1Value == RCU_PREDV1_DIV4) || (Predv1Value == RCU_PREDV1_DIV5) || (Predv1Value == RCU_PREDV1_DIV6) ||       \
     (Predv1Value == RCU_PREDV1_DIV7) || (Predv1Value == RCU_PREDV1_DIV8) || (Predv1Value == RCU_PREDV1_DIV9) ||       \
     (Predv1Value == RCU_PREDV1_DIV10) || (Predv1Value == RCU_PREDV1_DIV11) || (Predv1Value == RCU_PREDV1_DIV12) ||    \
     (Predv1Value == RCU_PREDV1_DIV13) || (Predv1Value == RCU_PREDV1_DIV14) || (Predv1Value == RCU_PREDV1_DIV15) ||    \
     (Predv1Value == RCU_PREDV1_DIV16))

#define IS_RCU_PLL2_MUL(PLL2Mul)                                                                                       \
    ((PLL2Mul == RCU_PLL2_MUL8) || (PLL2Mul == RCU_PLL2_MUL9) || (PLL2Mul == RCU_PLL2_MUL10) ||                        \
     (PLL2Mul == RCU_PLL2_MUL11) || (PLL2Mul == RCU_PLL2_MUL12) || (PLL2Mul == RCU_PLL2_MUL13) ||                      \
     (PLL2Mul == RCU_PLL2_MUL14) || (PLL2Mul == RCU_PLL2_MUL16) || (PLL2Mul == RCU_PLL2_MUL20))

#define IS_RCU_SYSCLKSOURCE(SysClkSrc)                                                                                 \
    ((SysClkSrc == RCU_CKSYSSRC_IRC8M) || (SysClkSrc == RCU_CKSYSSRC_HXTAL) || (SysClkSrc == RCU_CKSYSSRC_PLL))

#define IS_RCU_AHB_PRESCALE(AHBPrescale)                                                                               \
    ((AHBPrescale == RCU_AHB_CKSYS_DIV1) || (AHBPrescale == RCU_AHB_CKSYS_DIV2) ||                                     \
     (AHBPrescale == RCU_AHB_CKSYS_DIV4) || (AHBPrescale == RCU_AHB_CKSYS_DIV8) ||                                     \
     (AHBPrescale == RCU_AHB_CKSYS_DIV16) || (AHBPrescale == RCU_AHB_CKSYS_DIV64) ||                                   \
     (AHBPrescale == RCU_AHB_CKSYS_DIV128) || (AHBPrescale == RCU_AHB_CKSYS_DIV256) ||                                 \
     (AHBPrescale == RCU_AHB_CKSYS_DIV512))

#define IS_RCU_APB1_PRESCALE(APB1Prescale)                                                                             \
    ((APB1Prescale == RCU_APB1_CKAHB_DIV1) || (APB1Prescale == RCU_APB1_CKAHB_DIV2) ||                                 \
     (APB1Prescale == RCU_APB1_CKAHB_DIV4) || (APB1Prescale == RCU_APB1_CKAHB_DIV8) ||                                 \
     (APB1Prescale == RCU_APB1_CKAHB_DIV16))

#define IS_RCU_APB2_PRESCALE(APB2Prescale)                                                                             \
    ((APB2Prescale == RCU_APB2_CKAHB_DIV1) || (APB2Prescale == RCU_APB2_CKAHB_DIV2) ||                                 \
     (APB2Prescale == RCU_APB2_CKAHB_DIV4) || (APB2Prescale == RCU_APB2_CKAHB_DIV8) ||                                 \
     (APB2Prescale == RCU_APB2_CKAHB_DIV16))

/* Function ---------------------------------------------------------------- */
MDS_Err_t DRV_RCU_HXTALConfig(uint32_t HXTALState)
{
    MDS_ASSERT(IS_RCU_HXTAL_STATE(HXTALState));

    if ((DRV_RCU_GetSysClkSource() == RCU_SCSS_HXTAL) ||
        ((DRV_RCU_GetSysClkSource() == RCU_SCSS_PLL) && (DRV_RCU_GetPLLOscSource() == RCU_PLLSRC_HXTAL_IRC48M) &&
         ((RCU_CFG1 & RCU_CFG1_PLLPRESEL) == RCU_PLLPRESRC_HXTAL))) {
        if (((RCU_CTL & RCU_CTL_HXTALSTB) != 0U) && (HXTALState == RCU_HXTAL_OFF)) {
            return (MDS_EBUSY);
        }
    } else {
        MODIFY_REG(RCU_CTL, RCU_CTL_HXTALBPS | RCU_CTL_HXTALEN, HXTALState);

        if (HXTALState != RCU_HXTAL_OFF) {
            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_HXTALSTB) == 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > HXTAL_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        } else {
            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_HXTALSTB) != 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > HXTAL_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        }
    }

    return (MDS_EOK);
}

MDS_Err_t DRV_RCU_IRC8MConfig(uint32_t IRC8MState, uint32_t IRC8MCalibValue)
{
    MDS_ASSERT(IS_RCU_IRC8M_STATE(IRC8MState));

    if ((DRV_RCU_GetSysClkSource() == RCU_SCSS_IRC8M) ||
        ((DRV_RCU_GetSysClkSource() == RCU_SCSS_PLL) && (DRV_RCU_GetPLLOscSource() == RCU_PLLSRC_IRC8M_DIV2))) {
        if (((RCU_CTL & RCU_CTL_IRC8MSTB) != 0U) && (IRC8MState == RCU_IRC8M_OFF)) {
            return (MDS_EBUSY);
        }
    } else {
        if (IRC8MState != RCU_IRC8M_OFF) {
            RCU_CTL |= RCU_CTL_IRC8MEN;

            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_IRC8MSTB) == 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > IRC8M_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        } else {
            RCU_CTL &= ~RCU_CTL_IRC8MEN;

            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_IRC8MSTB) != 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > IRC8M_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        }
    }

    MODIFY_REG(RCU_CTL, RCU_CTL_IRC8MADJ, (uint32_t)(IRC8MCalibValue) << 0x03U);

    return (MDS_EOK);
}

MDS_Err_t DRV_RCU_IRC40KConfig(uint32_t IRC40KState)
{
    MDS_ASSERT(IS_RCU_IRC40K_STATE(IRC40KState));

    if (IRC40KState != RCU_IRC40K_OFF) {
        RCU_RSTSCK |= RCU_RSTSCK_IRC40KEN;

        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_RSTSCK & RCU_RSTSCK_IRC40KSTB) == 0U;) {
            if ((DRV_CHIP_GetTick() - tickstart) > IRC40K_TIMEOUT_VALUE) {
                return (MDS_ETIME);
            }
        }

        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (DRV_CHIP_GetTick() - tickstart) > 1U;) {
        }
    } else {
        RCU_RSTSCK &= ~RCU_RSTSCK_IRC40KEN;

        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_RSTSCK & RCU_RSTSCK_IRC40KSTB) != 0U;) {
            if ((DRV_CHIP_GetTick() - tickstart) > IRC40K_TIMEOUT_VALUE) {
                return (MDS_ETIME);
            }
        }
    }

    return (MDS_EOK);
}

MDS_Err_t DRV_RCU_LXTALConfig(uint32_t LXTALState)
{
    MDS_ASSERT(IS_RCU_LXTAL_STATE(LXTALState));

    bool pwrClkChanged = false;

    while ((RCU_APB1EN & RCU_APB1EN_PMUEN) == 0U) {
        RCU_APB1EN |= RCU_APB1EN_PMUEN;
        pwrClkChanged = true;
    }

    if ((PMU_CTL & PMU_CTL_BKPWEN) == 0U) {
        PMU_CTL |= PMU_CTL_BKPWEN;

        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (PMU_CTL & PMU_CTL_BKPWEN) == 0U;) {
            if ((DRV_CHIP_GetTick() - tickstart) > DBP_TIMEOUT_VALUE) {
                return (MDS_ETIME);
            }
        }
    }

    MODIFY_REG(RCU_BDCTL, RCU_BDCTL_LXTALBPS | RCU_BDCTL_LXTALEN, LXTALState);

    if (LXTALState != RCU_LXTAL_OFF) {
        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_BDCTL & RCU_BDCTL_LXTALSTB) == 0U;) {
            if ((DRV_CHIP_GetTick() - tickstart) > LXTAL_TIMEOUT_VALUE) {
                return (MDS_ETIME);
            }
        }
    } else {
        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_BDCTL & RCU_BDCTL_LXTALSTB) != 0U;) {
            if ((DRV_CHIP_GetTick() - tickstart) > LXTAL_TIMEOUT_VALUE) {
                return (MDS_ETIME);
            }
        }
    }

    if (pwrClkChanged) {
        PMU_CTL &= ~PMU_CTL_BKPWEN;
    }

    return (MDS_EOK);
}

MDS_Err_t DRV_RCU_IRC48MConfig(uint32_t IRC48MState)
{
    MDS_ASSERT(IS_RCU_IRC48M_STATE(IRC48MState));

    if ((DRV_RCU_GetSysClkSource() == RCU_SCSS_PLL) && (DRV_RCU_GetPLLOscSource() == RCU_PLLSRC_HXTAL_IRC48M) &&
        ((RCU_CFG1 & RCU_CFG1_PLLPRESEL) == RCU_PLLPRESRC_IRC48M)) {
        if (((RCU_ADDCTL & RCU_ADDCTL_IRC48MSTB) != 0U) && (IRC48MState == RCU_IRC48M_OFF)) {
            return (MDS_EBUSY);
        }
    } else {
        if (IRC48MState != RCU_IRC48M_OFF) {
            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_ADDCTL & RCU_ADDCTL_IRC48MSTB) == 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > IRC48M_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        } else {
            RCU_ADDCTL &= ~RCU_ADDCTL_IRC48MEN;
            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_ADDCTL & RCU_ADDCTL_IRC48MSTB) != 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > IRC48M_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        }
    }

    return (MDS_EOK);
}

MDS_Err_t DRV_RCU_PLLConfig(DRV_RCU_PLLInit_t *pllInitStruct)
{
    MDS_ASSERT(pllInitStruct != NULL);
    MDS_ASSERT(IS_RCU_PLL_STATE(pllInitStruct->PLLState));

    if (pllInitStruct->PLLState == RCU_PLL_NONE) {
        return (MDS_EOK);
    }

    if (DRV_RCU_GetSysClkSource() != RCU_SCSS_PLL) {
        if (pllInitStruct->PLLState == RCU_PLL_ON) {
            MDS_ASSERT(IS_RCU_PLL_SOURCE(pllInitStruct->PLLSource));
            MDS_ASSERT(IS_RCU_PLL_MUL(pllInitStruct->PLLMUL));

            RCU_CTL &= ~RCU_CTL_PLLEN;

            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_PLLSTB) != 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }

            if (pllInitStruct->PLLSource == RCU_PLLSRC_HXTAL_IRC48M) {
                MDS_ASSERT(IS_RCU_PREDV0_SOURCE(pllInitStruct->Predv0Source));
                MDS_ASSERT(IS_RCU_PREDV0_VALUE(pllInitStruct->Predv0Value));

                MODIFY_REG(RCU_CFG1, RCU_CFG1_PREDV0SEL, pllInitStruct->Predv0Source);
                MODIFY_REG(RCU_CFG1, RCU_CFG1_PREDV0, pllInitStruct->Predv0Value);
            }

            MODIFY_REG(RCU_CFG0, RCU_CFG0_PLLSEL | RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4,
                       pllInitStruct->PLLSource | pllInitStruct->PLLMUL);

            RCU_CTL |= RCU_CTL_PLLEN;

            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_PLLSTB) == 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        } else {
            RCU_CTL &= ~RCU_CTL_PLLEN;

            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_PLLSTB) != 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        }
    } else {
        if ((pllInitStruct->PLLState) == RCU_PLL_OFF) {
            return (MDS_EBUSY);
        } else {
            uint32_t pllConfig = RCU_CFG0;
            if (((pllConfig & RCU_CFG0_PLLSEL) != pllInitStruct->PLLSource) ||
                ((pllConfig & (RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4)) != pllInitStruct->PLLMUL)) {
                return (MDS_EBUSY);
            }
        }
    }

    return (MDS_EOK);
}

MDS_Err_t DRV_RCU_PLL1Config(DRV_RCU_PLL1Init_t *pll1InitStruct, uint32_t predv1Value)
{
    MDS_ASSERT(pll1InitStruct != NULL);
    MDS_ASSERT(IS_RCU_PLL_STATE(pll1InitStruct->PLL1State));

    if (pll1InitStruct->PLL1State == RCU_PLL_NONE) {
        return (MDS_EOK);
    }

    if ((DRV_RCU_GetSysClkSource() == RCU_SCSS_PLL) && (DRV_RCU_GetPLLOscSource() == RCU_PLLSRC_HXTAL_IRC48M) &&
        ((RCU_CFG1 & RCU_CFG1_PREDV0SEL) == RCU_PREDV0SRC_CKPLL1)) {
        return (MDS_EBUSY);
    } else {
        if (pll1InitStruct->PLL1State != RCU_PLL_OFF) {
            MDS_ASSERT(IS_RCU_PLL1_MUL(pll1InitStruct->PLL1MUL));
            MDS_ASSERT(IS_RCU_PREDV1_VALUE(predv1Value));

            if (((RCU_CTL & RCU_CTL_PLL2EN) != 0U) && ((RCU_CFG1 & RCU_CFG1_PREDV1) != predv1Value)) {
                return (MDS_EBUSY);
            }

            RCU_CTL &= ~RCU_CTL_PLL1EN;

            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_PLL1STB) != 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }

            MODIFY_REG(RCU_CFG1, RCU_CFG1_PREDV1, predv1Value);
            MODIFY_REG(RCU_CFG1, RCU_CFG1_PLL1MF, pll1InitStruct->PLL1MUL);
            RCU_CTL |= RCU_CTL_PLL1EN;

            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_PLL1STB) == 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        } else {
            RCU_CFG1 &= ~RCU_CFG1_PREDV0SEL;
            RCU_CTL &= ~RCU_CTL_PLL1EN;

            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_PLL1STB) != 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        }
    }

    return (MDS_EOK);
}

MDS_Err_t DRV_RCU_PLL2Config(DRV_RCU_PLL2Init_t *pll2InitStruct, uint32_t predv1Value)
{
    MDS_ASSERT(pll2InitStruct != NULL);
    MDS_ASSERT(IS_RCU_PLL_STATE(pll2InitStruct->PLL2State));

    if (pll2InitStruct->PLL2State == RCU_PLL_NONE) {
        return (MDS_EOK);
    }

    if (pll2InitStruct->PLL2State != RCU_PLL_OFF) {
        MDS_ASSERT(IS_RCU_PLL2_MUL(pll2InitStruct->PLL2MUL));
        MDS_ASSERT(IS_RCU_PREDV1_VALUE(predv1Value));

        if (((RCU_CTL & RCU_CTL_PLL1EN) != 0U) && ((RCU_CFG1 & RCU_CFG1_PREDV1) != predv1Value)) {
            return (MDS_EBUSY);
        }

        RCU_CTL &= ~RCU_CTL_PLL2EN;

        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_PLL2STB) != 0U;) {
            if ((DRV_CHIP_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
                return (MDS_ETIME);
            }
        }

        MODIFY_REG(RCU_CFG1, RCU_CFG1_PREDV1, predv1Value);
        MODIFY_REG(RCU_CFG1, RCU_CFG1_PLL2MF, pll2InitStruct->PLL2MUL);

        RCU_CTL |= RCU_CTL_PLL2EN;

        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_PLL2STB) == 0U;) {
            if ((DRV_CHIP_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
                return (MDS_ETIME);
            }
        }
    } else {
        if (((RCU_CFG1 & RCU_CFG1_I2S1SEL) != 0U) || ((RCU_CFG1 & RCU_CFG1_I2S2SEL) != 0U)) {
            return (MDS_EBUSY);
        }

        RCU_CTL &= ~RCU_CTL_PLL2EN;

        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_PLL2STB) != 0U;) {
            if ((DRV_CHIP_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
                return (MDS_ETIME);
            }
        }
    }

    return (MDS_EOK);
}

MDS_Err_t DRV_RCU_OscConfig(DRV_RCU_OscInit_t *oscInitStruct)
{
    MDS_ASSERT(oscInitStruct != NULL);

    MDS_Err_t err;

    do {
        if ((oscInitStruct->OscillatorType & RCU_OSCILLATORTYPE_HXTAL) != 0U) {
            err = DRV_RCU_HXTALConfig(oscInitStruct->HXTALState);
            if (err != MDS_EOK) {
                break;
            }
        }

        if ((oscInitStruct->OscillatorType & RCU_OSCILLATORTYPE_IRC48M) != 0U) {
            err = DRV_RCU_IRC48MConfig(oscInitStruct->IRC48MState);
            if (err != MDS_EOK) {
                break;
            }
        }

        if ((oscInitStruct->OscillatorType & RCU_OSCILLATORTYPE_IRC8M) != 0U) {
            err = DRV_RCU_IRC8MConfig(oscInitStruct->IRC8MState, oscInitStruct->IRC8MCalibValue);
            if (err != MDS_EOK) {
                break;
            }
        }

        if ((oscInitStruct->OscillatorType & RCU_OSCILLATORTYPE_IRC40K) != 0U) {
            err = DRV_RCU_IRC40KConfig(oscInitStruct->IRC40KState);
            if (err != MDS_EOK) {
                break;
            }
        }

        if ((oscInitStruct->OscillatorType & RCU_OSCILLATORTYPE_LXTAL) != 0U) {
            err = DRV_RCU_LXTALConfig(oscInitStruct->LXTALState);
            if (err != MDS_EOK) {
                break;
            }
        }

        err = DRV_RCU_PLL1Config(&(oscInitStruct->PLL1), oscInitStruct->Predv1Value);
        if (err != MDS_EOK) {
            break;
        }

        err = DRV_RCU_PLLConfig(&(oscInitStruct->PLL));
    } while (0);

    return (err);
}

MDS_Err_t DRV_RCU_ClockConfig(DRV_RCU_ClockInit_t *clkInitStruct)
{
    MDS_ASSERT(clkInitStruct != NULL);

    if (((clkInitStruct->ClockType) & RCU_CLOCKTYPE_HCLK) != 0U) {
        if (((clkInitStruct->ClockType) & RCU_CLOCKTYPE_PCLK1) != 0U) {
            MODIFY_REG(RCU_CFG0, RCU_CFG0_APB1PSC, RCU_APB1_CKAHB_DIV16);
        }

        if (((clkInitStruct->ClockType) & RCU_CLOCKTYPE_PCLK2) != 0U) {
            MODIFY_REG(RCU_CFG0, RCU_CFG0_APB2PSC, RCU_APB2_CKAHB_DIV16);
        }

        MDS_ASSERT(IS_RCU_AHB_PRESCALE(clkInitStruct->AHBCLKDivider));
        MODIFY_REG(RCU_CFG0, RCU_CFG0_AHBPSC, clkInitStruct->AHBCLKDivider);
    }

    if (((clkInitStruct->ClockType) & RCU_CLOCKTYPE_SYSCLK) != 0U) {
        MDS_ASSERT(IS_RCU_SYSCLKSOURCE(clkInitStruct->SYSCLKSource));

        if (clkInitStruct->SYSCLKSource == RCU_CKSYSSRC_HXTAL) {
            if ((RCU_CTL & RCU_CTL_HXTALSTB) == 0U) {
                return (MDS_ENOENT);
            }
        } else if (clkInitStruct->SYSCLKSource == RCU_CKSYSSRC_PLL) {
            if ((RCU_CTL & RCU_CTL_PLLSTB) == 0U) {
                return (MDS_ENOENT);
            }
        } else {
            if ((RCU_CTL & RCU_CTL_IRC8MSTB) == 0U) {
                return (MDS_ENOENT);
            }
        }

        MODIFY_REG(RCU_CFG0, RCU_CFG0_SCS, clkInitStruct->SYSCLKSource);

        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick();
             DRV_RCU_GetSysClkSource() != (clkInitStruct->SYSCLKSource << 0x2U);) {
            if ((DRV_CHIP_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) {
                return (MDS_ETIME);
            }
        }
    }

    if (((clkInitStruct->ClockType) & RCU_CLOCKTYPE_PCLK1) != 0U) {
        MDS_ASSERT(IS_RCU_APB1_PRESCALE(clkInitStruct->APB1CLKDivider));

        MODIFY_REG(RCU_CFG0, RCU_CFG0_APB1PSC, clkInitStruct->APB1CLKDivider);
    }

    if (((clkInitStruct->ClockType) & RCU_CLOCKTYPE_PCLK2) != 0U) {
        MDS_ASSERT(IS_RCU_APB2_PRESCALE(clkInitStruct->APB2CLKDivider));

        MODIFY_REG(RCU_CFG0, RCU_CFG0_APB2PSC, clkInitStruct->APB2CLKDivider);
    }

    SystemCoreClock = DRV_RCU_GetSysClockFreq();

    if (SysTick_Config(SystemCoreClock / MDS_CLOCK_TICK_FREQ_HZ) != 0) {
        return (MDS_EIO);
    }

    return (MDS_EOK);
}
uint32_t DRV_RCU_GetPLL1ClockFreq(void)
{
    static const uint32_t PLL1MulTable[] = {0, 0, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0, 16, 20};

    uint32_t pll1ClkFreq;
    uint32_t pll1mul = PLL1MulTable[((RCU_CFG1 & RCU_CFG1_PLL1MF) >> 0x08U)];
    uint32_t predv1Div = (((RCU_CFG1 & RCU_CFG1_PREDV1) >> 0x03U) + 1U);

    if (DRV_RCU_GetPLLPreSource() == RCU_PLLPRESRC_HXTAL) {
        pll1ClkFreq = HXTAL_VALUE / predv1Div * pll1mul;
    } else {
        pll1ClkFreq = IRC48M_VALUE / predv1Div * pll1mul;
    }

    return (pll1ClkFreq);
}

uint32_t DRV_RCU_GetPLL2ClockFreq(void)
{
    static const uint32_t PLL2MulTable[] = {0, 0, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0, 16, 20};

    uint32_t pll2ClkFreq;
    uint32_t pll2mul = PLL2MulTable[((RCU_CFG1 & RCU_CFG1_PLL2MF) >> 0x0CU)];
    uint32_t predv1Div = (((RCU_CFG1 & RCU_CFG1_PREDV1) >> 0x03U) + 1U);

    if (DRV_RCU_GetPLLPreSource() == RCU_PLLPRESRC_HXTAL) {
        pll2ClkFreq = HXTAL_VALUE / predv1Div * pll2mul;
    } else {
        pll2ClkFreq = IRC48M_VALUE / predv1Div * pll2mul;
    }

    return (pll2ClkFreq);
}

uint32_t DRV_RCU_GetPLLClockFreq(void)
{
    uint32_t pllsrcClk;

    if (DRV_RCU_GetPLLOscSource() == RCU_PLLSRC_IRC8M_DIV2) {
        pllsrcClk = IRC8M_VALUE >> 1;
    } else {  // RCU_PLLSRC_HXTAL_IRC48M
        uint32_t predv0SrcFreq;
        if ((RCU_CFG1 & RCU_CFG1_PREDV0SEL) == RCU_PREDV0SRC_HXTAL_IRC48M) {
            predv0SrcFreq = ((RCU_CFG1 & RCU_CFG1_PLLPRESEL) == RCU_PLLPRESRC_IRC48M) ? (IRC48M_VALUE) : (HXTAL_VALUE);
        } else {  // RCU_PREDV0SRC_CKPLL1
            predv0SrcFreq = DRV_RCU_GetPLL1ClockFreq();
        }
        pllsrcClk = predv0SrcFreq / (((RCU_CFG1 & RCU_CFG1_PREDV0) >> 0U) + 0x01U);
    }

    uint32_t pllmf = ((RCU_CFG0 & RCU_CFG0_PLLMF) >> 0x12U);

    if ((RCU_CFG0 & RCU_CFG0_PLLMF_4) != 0U) {  // 17~31,31
        return (pllsrcClk * (0x11U + ((pllmf <= 0x0EU) ? (pllmf) : (0x0EU))));
    } else if (pllmf <= 0x0DU) {  // 2~14
        return (pllsrcClk * (pllmf + 0x02U));
    } else if (pllmf == 0x0DU) {
        return (pllsrcClk * 0x0DU >> 0x1U);  // *6.5
    } else {
        return (pllsrcClk * 0x10U);  // *16
    }
}

uint32_t DRV_RCU_GetSysClockFreq(void)
{
    uint32_t sysclkSrc = DRV_RCU_GetSysClkSource();

    if (sysclkSrc == RCU_SCSS_HXTAL) {
        return (HXTAL_VALUE);
    } else if (sysclkSrc == RCU_SCSS_PLL) {
        return (DRV_RCU_GetPLLClockFreq());
    } else {  // RCU_SCSS_IRC8M
        return (IRC8M_VALUE);
    }
}

uint32_t DRV_RCU_GetCoreClockFreq(void)
{
    uint32_t ahbpsc = (RCU_CFG0 & RCU_CFG0_AHBPSC) >> 0x04U;
    uint32_t sysclkFreq = DRV_RCU_GetSysClockFreq();

    if (ahbpsc >= 0x08U) {
        return (sysclkFreq >> (ahbpsc - 0x07U));
    } else {
        return (sysclkFreq);
    }
}

MDS_Err_t DRV_RCU_DeInit(void)
{
    MDS_Err_t err = DRV_RCU_IRC8MConfig(RCU_IRC8M_ON, RCU_IRC8MCALIB_DEFAULT);
    if (err == MDS_EOK) {
        DRV_RCU_ClockInit_t clkInitStruct = {
            .ClockType = RCU_CLOCKTYPE_SYSCLK | RCU_CLOCKTYPE_HCLK | RCU_CLOCKTYPE_PCLK1 | RCU_CLOCKTYPE_PCLK2,
            .SYSCLKSource = RCU_CKSYSSRC_IRC8M,
            .AHBCLKDivider = RCU_AHB_CKSYS_DIV1,
            .APB1CLKDivider = RCU_APB1_CKAHB_DIV1,
            .APB2CLKDivider = RCU_APB2_CKAHB_DIV1,
        };

        err = DRV_RCU_ClockConfig(&clkInitStruct);
    }

    RCU_CFG0 = 0x00U;

    for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); DRV_RCU_GetSysClkSource() != 0x00U;) {
        if ((DRV_CHIP_GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) {
            return (MDS_ETIME);
        }
    }

    SystemCoreClock = IRC8M_VALUE;

    if (SysTick_Config(SystemCoreClock / MDS_CLOCK_TICK_FREQ_HZ) != 0) {
        return (MDS_EIO);
    }

    RCU_CTL &= ~(RCU_CTL_PLLEN | RCU_CTL_PLL1EN | RCU_CTL_PLL2EN);

    for (MDS_Tick_t tickstart = DRV_CHIP_GetTick();
         (RCU_CTL & (RCU_CTL_PLLSTB | RCU_CTL_PLL1STB | RCU_CTL_PLL2STB)) != 0U;) {
        if ((DRV_CHIP_GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
            return (MDS_ETIME);
        }
    }

    RCU_CTL &= ~(RCU_CTL_HXTALEN | RCU_CTL_CKMEN);

    for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_CTL & RCU_CTL_HXTALSTB) != 0U;) {
        if ((DRV_CHIP_GetTick() - tickstart) > HXTAL_TIMEOUT_VALUE) {
            return (MDS_ETIME);
        }
    }

    RCU_CTL &= ~RCU_CTL_HXTALBPS;

    RCU_INT = 0x00U;

    return (MDS_EOK);
}

MDS_Err_t DRV_RCU_RTCClockSelection(uint32_t rtcClkSource)
{
    bool pwrClkChanged = false;

    while ((RCU_APB1EN & RCU_APB1EN_PMUEN) == 0U) {
        RCU_APB1EN |= RCU_APB1EN_PMUEN;
        pwrClkChanged = true;
    }

    if ((PMU_CTL & PMU_CTL_BKPWEN) == 0U) {
        PMU_CTL |= PMU_CTL_BKPWEN;

        for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (PMU_CTL & PMU_CTL_BKPWEN) == 0U;) {
            if ((DRV_CHIP_GetTick() - tickstart) > DBP_TIMEOUT_VALUE) {
                return (MDS_ETIME);
            }
        }
    }

    uint32_t tmpReg = RCU_BDCTL & RCU_BDCTL_RTCSRC;
    if ((tmpReg != RCU_RTCSRC_NONE) && (tmpReg != (rtcClkSource & RCU_BDCTL_RTCSRC))) {
        tmpReg = RCU_BDCTL & ~(RCU_BDCTL_RTCSRC);

        RCU_BDCTL |= RCU_BDCTL_BKPRST;
        RCU_BDCTL &= ~RCU_BDCTL_BKPRST;

        RCU_BDCTL = tmpReg | (rtcClkSource & RCU_BDCTL_RTCSRC);

        if ((tmpReg & RCU_BDCTL_LXTALEN) != 0U) {
            for (MDS_Tick_t tickstart = DRV_CHIP_GetTick(); (RCU_BDCTL & RCU_BDCTL_LXTALSTB) == 0U;) {
                if ((DRV_CHIP_GetTick() - tickstart) > LXTAL_TIMEOUT_VALUE) {
                    return (MDS_ETIME);
                }
            }
        }
    }

    if (pwrClkChanged) {
        RCU_APB1EN &= ~RCU_APB1EN_PMUEN;
    }

    return (MDS_EOK);
}
