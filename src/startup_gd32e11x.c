/**
 * @copyright   Copyright (c) 2024 Pchom & licensed under Mulan PSL v2
 * @file        startup_gd32e11x.c
 * @brief       gd32e11x startup source
 * @date        2024-05-30
 */
/* Include ----------------------------------------------------------------- */
#include "gd32e11x.h"

/* Reference --------------------------------------------------------------- */
void __INITIAL_SP(void);
void __STACK_LIMIT(void);
void Reset_Handler(void);

/* Interrupt --------------------------------------------------------------- */
__attribute__((weak)) void Interrupt_Handler(uintptr_t ipsr)
{
    (void)(ipsr);

    for (;;) {
        __WFI();
    }
}

void Default_Handler(void)
{
    uintptr_t ipsr = __get_IPSR() & IPSR_ISR_Msk;

    Interrupt_Handler(ipsr);
}

__attribute__((weak, alias("Default_Handler"))) void NMI_Handler(void);
__attribute__((weak, alias("Default_Handler"))) void HardFault_Handler(void);
__attribute__((weak, alias("Default_Handler"))) void MemManage_Handler(void);
__attribute__((weak, alias("Default_Handler"))) void BusFault_Handler(void);
__attribute__((weak, alias("Default_Handler"))) void UsageFault_Handler(void);
__attribute__((weak, alias("Default_Handler"))) void SVC_Handler(void);
__attribute__((weak, alias("Default_Handler"))) void DebugMon_Handler(void);
__attribute__((weak, alias("Default_Handler"))) void PendSV_Handler(void);
__attribute__((weak, alias("Default_Handler"))) void SysTick_Handler(void);
#ifndef DRV_CHIP_WITHOUT_INTERRUPT
__attribute__((weak, alias("Default_Handler"))) void WWDGT_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void LVD_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TAMPER_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void RTC_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void FMC_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void RCU_CTC_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void EXTI0_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void EXTI1_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void EXTI2_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void EXTI3_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void EXTI4_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA0_Channel0_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA0_Channel1_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA0_Channel2_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA0_Channel3_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA0_Channel4_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA0_Channel5_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA0_Channel6_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void ADC0_1_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void EXTI5_9_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER0_BRK_TIMER8_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER0_UP_TIMER9_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER0_TRG_CMT_TIMER10_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER0_Channel_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER1_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER2_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER3_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void I2C0_EV_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void I2C0_ER_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void I2C1_EV_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void I2C1_ER_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void SPI0_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void SPI1_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void USART0_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void USART1_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void USART2_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void EXTI10_15_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void RTC_Alarm_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void USBFS_WKUP_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER7_BRK_TIMER11_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER7_UP_TIMER12_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER7_TRG_CMT_TIMER13_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER7_Channel_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void EXMC_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER4_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void SPI2_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void UART3_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void UART4_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER5_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void TIMER6_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Channel0_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Channel1_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Channel2_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Channel3_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void DMA1_Channel4_IRQHandler(void);
__attribute__((weak, alias("Default_Handler"))) void USBFS_IRQHandler(void);
#endif

static void (*__VECTOR_TABLE[])(void) __VECTOR_TABLE_ATTRIBUTE = {
    (void *)(&__INITIAL_SP),
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
#ifndef DRV_CHIP_WITHOUT_INTERRUPT
    WWDGT_IRQHandler,
    LVD_IRQHandler,
    TAMPER_IRQHandler,
    RTC_IRQHandler,
    FMC_IRQHandler,
    RCU_CTC_IRQHandler,
    EXTI0_IRQHandler,
    EXTI1_IRQHandler,
    EXTI2_IRQHandler,
    EXTI3_IRQHandler,
    EXTI4_IRQHandler,
    DMA0_Channel0_IRQHandler,
    DMA0_Channel1_IRQHandler,
    DMA0_Channel2_IRQHandler,
    DMA0_Channel3_IRQHandler,
    DMA0_Channel4_IRQHandler,
    DMA0_Channel5_IRQHandler,
    DMA0_Channel6_IRQHandler,
    ADC0_1_IRQHandler,
    0,
    0,
    0,
    0,
    EXTI5_9_IRQHandler,
    TIMER0_BRK_TIMER8_IRQHandler,
    TIMER0_UP_TIMER9_IRQHandler,
    TIMER0_TRG_CMT_TIMER10_IRQHandler,
    TIMER0_Channel_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    TIMER3_IRQHandler,
    I2C0_EV_IRQHandler,
    I2C0_ER_IRQHandler,
    I2C1_EV_IRQHandler,
    I2C1_ER_IRQHandler,
    SPI0_IRQHandler,
    SPI1_IRQHandler,
    USART0_IRQHandler,
    USART1_IRQHandler,
    USART2_IRQHandler,
    EXTI10_15_IRQHandler,
    RTC_Alarm_IRQHandler,
    USBFS_WKUP_IRQHandler,
    TIMER7_BRK_TIMER11_IRQHandler,
    TIMER7_UP_TIMER12_IRQHandler,
    TIMER7_TRG_CMT_TIMER13_IRQHandler,
    TIMER7_Channel_IRQHandler,
    0,
    EXMC_IRQHandler,
    0,
    TIMER4_IRQHandler,
    SPI2_IRQHandler,
    UART3_IRQHandler,
    UART4_IRQHandler,
    TIMER5_IRQHandler,
    TIMER6_IRQHandler,
    DMA1_Channel0_IRQHandler,
    DMA1_Channel1_IRQHandler,
    DMA1_Channel2_IRQHandler,
    DMA1_Channel3_IRQHandler,
    DMA1_Channel4_IRQHandler,
    0,
    0,
    0,
    0,
    0,
    0,
    USBFS_IRQHandler,
#endif
};

/* Function ---------------------------------------------------------------- */
__attribute__((weak, naked, noreturn)) void _start(void)
{
    __asm volatile("bl     main");
    __asm volatile("b      .");
}

__attribute__((weak, naked, noreturn)) void _exit(int res)
{
    (void)(res);

    __asm volatile("b      .");
}

__attribute__((weak)) void VectorInit(uintptr_t vectorAddress)
{
    SysTick->CTRL = 0U;
    SysTick->LOAD = 0U;
    SysTick->VAL = 0U;

    for (uint32_t idx = 0; idx < (sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0])); idx++) {
        NVIC->ICER[idx] = 0xFFFFFFFF;
    }

    SCB->VTOR = vectorAddress;
}

__attribute__((naked, noreturn)) void Reset_Handler(void)
{
    __disable_irq();

    __set_MSP((uint32_t)(&__INITIAL_SP));

    SystemInit();

    VectorInit((uintptr_t)__VECTOR_TABLE);

    __enable_irq();

    __PROGRAM_START();
}

__attribute__((noreturn)) void DRV_CHIP_JumpIntoVectorAddress(uintptr_t vectorAddress)
{
    typedef void (*vector_t)(void);
    vector_t *vectorTable = (vector_t *)(vectorAddress);

    __disable_irq();

    __set_MSP((uint32_t)(vectorTable[0]));

    VectorInit(vectorAddress);

    __enable_irq();

    vectorTable[1]();

    for (;;) {
    }
}

__attribute__((__noreturn__)) void DRV_CHIP_JumpIntoDFU(void)
{
    DRV_CHIP_JumpIntoVectorAddress(0x1FFFF000);
}

__attribute__((__noreturn__)) void DRV_CHIP_SystemReset(void)
{
    NVIC_SystemReset();
}
