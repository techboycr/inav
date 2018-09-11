/*
 * This file is part of INAV.
 *
 * INAV is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * INAV is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with INAV.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"

static TIM_HandleTypeDef timerHandle[HARDWARE_TIMER_DEFINITION_COUNT];

TIM_HandleTypeDef * timerFindTimerHandle(TIM_TypeDef *tim)
{
    uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex >= HARDWARE_TIMER_DEFINITION_COUNT) {
        return NULL;
    }

    return &timerHandle[timerIndex];
}

void impl_timerNVICConfigure(uint8_t irq, int irqPriority)
{
    HAL_NVIC_SetPriority(irq, NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));
    HAL_NVIC_EnableIRQ(irq);
}

void impl_timerConfigBase(TIM_TypeDef *timer, uint16_t period, uint8_t mhz)
{
    uint8_t timerIndex = lookupTimerIndex(timer);

    if (timerIndex >= HARDWARE_TIMER_DEFINITION_COUNT) {
        return;
    }

    if (timerHandle[timerIndex].Instance == timer)
    {
        // already configured
        return;
    }

    LL_TIM_InitTypeDef init;
    LL_TIM_StructInit(&init);

    init.Prescaler = (timerClock(timer) / ((uint32_t)mhz * 1000000)) - 1;
    init.Autoreload = (period - 1) & 0xffff;
    
    LL_TIM_Init(timer, &init);

    LL_TIM_DisableARRPreload(timer);
    LL_TIM_SetClockSource(timer, LL_TIM_CLOCKSOURCE_INTERNAL);

    if (IS_TIM_BKIN2_INSTANCE(timer)) {
        LL_TIM_DisableBRK2(timer);
    }

    if (IS_TIM_MASTER_INSTANCE(timer)) {
        LL_TIM_DisableMasterSlaveMode(timer);
    }

    if (IS_TIM_BREAK_INSTANCE(timer)) {
        LL_TIM_DisableBRK(timer);
        LL_TIM_EnableAllOutputs(timer);
    }

    LL_TIM_EnableCounter(timer);

    timerHandle[timerIndex].Instance = timer;
}

uint8_t impl_timerLookupChannelLL(uint8_t channelIndex)
{
    const uint8_t lookupTable[] = { LL_TIM_CHANNEL_CH1, LL_TIM_CHANNEL_2, LL_TIM_CHANNEL_3, LL_TIM_CHANNEL_4 };

    if (channelIndex <= CC_CHANNELS_PER_TIMER) {
        return lookupTable[channelIndex];
    }

    return 0;
}

void impl_timerPWMConfigChannel(TIM_TypeDef * timer, uint8_t channelIndex, bool isNChannel, bool inverted, uint16_t value)
{
    UNUSED(isNChannel);

    __IO timCCR_t * ccr = impl_timerCCR(timer, channelIndex);
    uint16_t ll_channel = impl_timerLookupChannelLL(channelIndex);

    LL_TIM_OC_InitTypeDef init;
    LL_TIM_OC_StructInit(&init);

    init.OCMode = LL_TIM_OCMODE_PWM1;
    init.OCState = LL_TIM_OCSTATE_ENABLE;

    if (inverted) {
        init.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    }

    LL_TIM_OC_Init(timer, ll_channel, &init);
    LL_TIM_OC_DisableFast(timer, ll_channel);
    LL_TIM_OC_EnablePreload(timer, ll_channel);

    *ccr = value;
}

uint8_t impl_timerLookupChannel(uint8_t channelIndex)
{
    const uint8_t lookupTable[] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };

    if (channelIndex <= CC_CHANNELS_PER_TIMER) {
        return lookupTable[channelIndex];
    }

    return 0;
}


volatile timCCR_t * impl_timerCCR(TIM_TypeDef *tim, uint8_t channelIndex)
{
    switch (channelIndex) {
        case 0:
            return &tim->CCR1;
            break;
        case 1:
            return &tim->CCR2;
            break;
        case 2:
            return &tim->CCR3;
            break;
        case 3:
            return &tim->CCR4;
            break;
    }
    return NULL;
}

void impl_enableTimer(TIM_TypeDef * tim)
{
    TIM_HandleTypeDef * Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) {
        return;
    }

    HAL_TIM_Base_Start(Handle);
}

void impl_timerPWMStart(TIM_TypeDef * tim, unsigned channel, bool isNChannel)
{
    TIM_HandleTypeDef * Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) {
        return;
    }

    if (isNChannel)
        HAL_TIMEx_PWMN_Start(Handle, channel);
    else
        HAL_TIM_PWM_Start(Handle, channel);
}

void impl_timerEnableIT(TIM_TypeDef * tim, uint32_t interrupt)
{
    TIM_HandleTypeDef * Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) {
        return;
    }

    __HAL_TIM_ENABLE_IT(Handle, interrupt);
}

void impl_timerDisableIT(TIM_TypeDef * tim, uint32_t interrupt)
{
    TIM_HandleTypeDef * Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) {
        return;
    }

    __HAL_TIM_DISABLE_IT(Handle, interrupt);
}

void impl_timerClearFlag(TIM_TypeDef * tim, uint32_t flag)
{
    TIM_HandleTypeDef * Handle = timerFindTimerHandle(tim);
    if (Handle == NULL) {
        return;
    }

    __HAL_TIM_CLEAR_FLAG(Handle, flag);
}

// calculate input filter constant
static unsigned getFilter(unsigned ticks)
{
    static const unsigned ftab[16] = {
        1*1,                 // fDTS !
        1*2, 1*4, 1*8,       // fCK_INT
        2*6, 2*8,            // fDTS/2
        4*6, 4*8,
        8*6, 8*8,
        16*5, 16*6, 16*8,
        32*5, 32*6, 32*8
    };
    for (unsigned i = 1; i < ARRAYLEN(ftab); i++)
        if (ftab[i] > ticks)
            return i - 1;
    return 0x0f;
}

void impl_timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_HandleTypeDef * Handle = timerFindTimerHandle(timHw->tim);
    if (Handle == NULL) {
        return;
    }

    TIM_IC_InitTypeDef TIM_ICInitStructure;

    TIM_ICInitStructure.ICPolarity = polarityRising ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
    TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
    TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.ICFilter = getFilter(inputFilterTicks);
    HAL_TIM_IC_ConfigChannel(Handle,&TIM_ICInitStructure, timHw->channel);
}

void impl_timerCaptureCompareHandler(TIM_TypeDef *tim, timHardwareContext_t *timerCtx)
{
    unsigned tim_status = tim->SR & tim->DIER;

    while (tim_status) {
        // flags will be cleared by reading CCR in dual capture, make sure we call handler correctly
        // currrent order is highest bit first. Code should not rely on specific order (it will introduce race conditions anyway)
        unsigned bit = __builtin_clz(tim_status);
        unsigned mask = ~(0x80000000 >> bit);
        tim->SR = mask;
        tim_status &= mask;

        if (timerCtx) {
            switch (bit) {
                case __builtin_clz(TIM_IT_Update): {
                    const uint16_t capture = tim->ARR;
                    if (timerCtx->ch[0].callbackOvr) {
                        timerCtx->ch[0].callbackOvr(&timerCtx->ch[0], capture);
                    }
                    if (timerCtx->ch[1].callbackOvr) {
                        timerCtx->ch[1].callbackOvr(&timerCtx->ch[1], capture);
                    }
                    if (timerCtx->ch[2].callbackOvr) {
                        timerCtx->ch[2].callbackOvr(&timerCtx->ch[2], capture);
                    }
                    if (timerCtx->ch[3].callbackOvr) {
                        timerCtx->ch[3].callbackOvr(&timerCtx->ch[3], capture);
                    }
                    break;
                }
                case __builtin_clz(TIM_IT_CC1):
                    timerCtx->ch[0].callbackEdge(&timerCtx->ch[0], tim->CCR1);
                    break;
                case __builtin_clz(TIM_IT_CC2):
                    timerCtx->ch[1].callbackEdge(&timerCtx->ch[1], tim->CCR2);
                    break;
                case __builtin_clz(TIM_IT_CC3):
                    timerCtx->ch[2].callbackEdge(&timerCtx->ch[2], tim->CCR3);
                    break;
                case __builtin_clz(TIM_IT_CC4):
                    timerCtx->ch[3].callbackEdge(&timerCtx->ch[3], tim->CCR4);
                    break;
            }
        }
        else {
            // timerConfig == NULL
            volatile uint32_t tmp;

            switch (bit) {
                case __builtin_clz(TIM_IT_UPDATE):
                    tmp = tim->ARR;
                    break;
                case __builtin_clz(TIM_IT_CC1):
                    tmp = tim->CCR1;
                    break;
                case __builtin_clz(TIM_IT_CC2):
                    tmp = tim->CCR2;
                    break;
                case __builtin_clz(TIM_IT_CC3):
                    tmp = tim->CCR3;
                    break;
                case __builtin_clz(TIM_IT_CC4):
                    tmp = tim->CCR4;
                    break;
            }

            (void)tmp;
        }
    }
}

uint16_t impl_timerDmaSource(uint8_t channelIndex)
{
    const uint16_t lookupTable[] = { TIM_DMA_ID_CC1, TIM_DMA_ID_CC2, TIM_DMA_ID_CC3, TIM_DMA_ID_CC4 };

    if (channelIndex <= CC_CHANNELS_PER_TIMER) {
        return lookupTable[channelIndex];
    }

    return 0;
}
