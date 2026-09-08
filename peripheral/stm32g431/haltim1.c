/**
 * @file  haltim1.c
 * @brief TIM1 initialization — 3-phase PWM for FOC
 *
 * Pin mapping:
 *   PA8  — TIM1_CH1  (AF6) — U high-side
 *   PC13 — TIM1_CH1N (AF4) — U low-side
 *   PA9  — TIM1_CH2  (AF6) — V high-side
 *   PA12 — TIM1_CH2N (AF6) — V low-side
 *   PA10 — TIM1_CH3  (AF6) — W high-side
 *   PB15 — TIM1_CH3N (AF4) — W low-side
 *   CH4 = PWM2 mode → OC4REF → ADC injected trigger
 *
 * PWM: 20 kHz center-aligned (170 MHz timer clock, no prescaler),
 * complementary outputs with dead-time, TIM1 break from COMP1/2/4.
 * CH4 触发在 PWM 顶部（低边全导通区）采样。底部翻转实测无效，已还原。
 *
 * Initialization sequence and register layout match reference
 * STOPLL_FOC_2205 (same board, same 2205 gimbal motor).
 */

#include "haltim1.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_bus.h"

/* 170 MHz timer clock (APB2) → 20 kHz center-aligned:
 *   PWM_PERIOD_CYCLES = 170e6 / 20000 = 8500
 *   ARR = 8500 / 2 = 4250
 *   Dead-time: ~750 ns @ 85 MHz DTS (170 MHz / DIV2) → 64 ticks */
#define TIM1_PRESCALER      0U
#define TIM1_PERIOD         4250U   /* center-aligned 20 kHz @ 170 MHz */
#define TIM1_DEAD_TIME      64U     /* ~750 ns @ 85 MHz DTS */
#define TIM1_HTMIN          10U     /* ADC 触发提前量（相对 PWM 顶部，低边导通区） */
#define TIM1_CH4_CCR        ((TIM1_PERIOD) - (TIM1_HTMIN))

static uint32_t s_wDutyU, s_wDutyV, s_wDutyW; /* cache for SetDuty */

void haltim1_Init(void)
{
    /* ---- Clock enable ---- */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);

    /* ---- TIM1 base (matches STOPLL_FOC_2205 reference) ---- */
    LL_TIM_InitTypeDef tim_init = {0};
    tim_init.Prescaler         = TIM1_PRESCALER;
    tim_init.CounterMode       = LL_TIM_COUNTERMODE_CENTER_DOWN;
    tim_init.Autoreload        = TIM1_PERIOD;
    tim_init.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV2;
    tim_init.RepetitionCounter = 1;
    LL_TIM_Init(TIM1, &tim_init);
    LL_TIM_DisableARRPreload(TIM1);

    /* ---- Output compare: CH1-CH3 PWM1 complementary, CH4 PWM2 for ADC ---- */
    LL_TIM_OC_InitTypeDef oc = {0};
    oc.OCMode       = LL_TIM_OCMODE_PWM1;
    oc.OCState      = LL_TIM_OCSTATE_DISABLE;
    oc.OCNState     = LL_TIM_OCSTATE_DISABLE;
    oc.CompareValue = TIM1_PERIOD / 2;
    oc.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity  = LL_TIM_OCPOLARITY_HIGH;
    oc.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
    oc.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;

    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &oc);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &oc);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &oc);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);

    /* CH4: PWM2 with compare = PERIOD - HTMIN (trigger before PWM center) */
    oc.OCMode       = LL_TIM_OCMODE_PWM2;
    oc.CompareValue = TIM1_CH4_CCR;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &oc);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);

    /* TRGO = OC4REF (ADC injected trigger on both ADC1 and ADC2) */
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_OC4REF);
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);

    /* ---- Break inputs from COMP1/2/4 (overcurrent protection) ---- */
    LL_TIM_SetBreakInputSourcePolarity(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP1, LL_TIM_BKIN_POLARITY_HIGH);
    LL_TIM_EnableBreakInputSource(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP1);
    LL_TIM_SetBreakInputSourcePolarity(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP2, LL_TIM_BKIN_POLARITY_HIGH);
    LL_TIM_EnableBreakInputSource(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP2);
    LL_TIM_SetBreakInputSourcePolarity(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP4, LL_TIM_BKIN_POLARITY_HIGH);
    LL_TIM_EnableBreakInputSource(TIM1, LL_TIM_BREAK_INPUT_BKIN,
        LL_TIM_BKIN_SOURCE_BKCOMP4);

    /* ---- BDTR: dead-time, OSSR/OSSI, break ---- */
    LL_TIM_BDTR_InitTypeDef bdtr = {0};
    bdtr.OSSRState       = LL_TIM_OSSR_ENABLE;
    bdtr.OSSIState       = LL_TIM_OSSI_ENABLE;
    bdtr.LockLevel       = LL_TIM_LOCKLEVEL_OFF;
    bdtr.DeadTime        = TIM1_DEAD_TIME;
    bdtr.BreakState      = LL_TIM_BREAK_ENABLE;
    bdtr.BreakPolarity   = LL_TIM_BREAK_POLARITY_HIGH;
    bdtr.BreakFilter     = LL_TIM_BREAK_FILTER_FDIV1_N8;
    bdtr.BreakAFMode     = LL_TIM_BREAK_AFMODE_INPUT;
    bdtr.Break2State     = LL_TIM_BREAK2_DISABLE;
    bdtr.Break2Polarity  = LL_TIM_BREAK2_POLARITY_HIGH;
    bdtr.Break2Filter    = LL_TIM_BREAK2_FILTER_FDIV1_N8;
    bdtr.Break2AFMode    = LL_TIM_BREAK_AFMODE_INPUT;
    bdtr.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &bdtr);

    /* ---- GPIO: PWM outputs ---- */
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLDOWN;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;

    gpio.Alternate = GPIO_AF4_TIM1;
    gpio.Pin = GPIO_PIN_13;             /* CH1N (PC13) */
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Alternate = GPIO_AF4_TIM1;
    gpio.Pin = GPIO_PIN_15;             /* CH3N (PB15) */
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Alternate = GPIO_AF6_TIM1;
    gpio.Pin = GPIO_PIN_8;              /* CH1 (PA8) */
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_9;              /* CH2 (PA9) */
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_10;             /* CH3 (PA10) */
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Alternate = GPIO_AF6_TIM1;
    gpio.Pin = GPIO_PIN_12;             /* CH2N (PA12) */
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Start counter (outputs disabled until haltim1_Start) */
    LL_TIM_GenerateEvent_UPDATE(TIM1);
    LL_TIM_EnableCounter(TIM1);
}

void haltim1_SetDuty(float fU, float fV, float fW)
{
    /* Clamp and scale float [0..1] to [0..TIM1_PERIOD] */
    if (fU < 0.0f) fU = 0.0f; else if (fU > 1.0f) fU = 1.0f;
    if (fV < 0.0f) fV = 0.0f; else if (fV > 1.0f) fV = 1.0f;
    if (fW < 0.0f) fW = 0.0f; else if (fW > 1.0f) fW = 1.0f;

    s_wDutyU = (uint32_t)(fU * (float)TIM1_PERIOD);
    s_wDutyV = (uint32_t)(fV * (float)TIM1_PERIOD);
    s_wDutyW = (uint32_t)(fW * (float)TIM1_PERIOD);

    LL_TIM_OC_SetCompareCH1(TIM1, s_wDutyU);
    LL_TIM_OC_SetCompareCH2(TIM1, s_wDutyV);
    LL_TIM_OC_SetCompareCH3(TIM1, s_wDutyW);
}

void haltim1_Start(void)
{
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableAllOutputs(TIM1);
}

void haltim1_StartAdcTrigger(void)
{
    /* CH4 is an internal trigger; CH1-CH3 remain disabled. */
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH4);
    LL_TIM_EnableAllOutputs(TIM1);
}

void haltim1_Stop(void)
{
    /* 只关三相功率输出（CH1-3），保留 CH4 + MOE：
       高频 FOC ISR 由 OC4REF→ADC 注入触发驱动，若整体关 MOE 会连
       ADC 触发一起停掉，导致后续 Start 的邮箱命令无人消费（电机路径
       启动死锁）。CH1-3 关断 + MOE 保持与 ADC 校准阶段完全一致，
       为已验证的安全关断状态。 */
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2 | LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH3N);
}
