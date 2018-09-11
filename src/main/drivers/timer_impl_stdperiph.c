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

#include "platform.h"

#include "build/atomic.h"

#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/rcc.h"
#include "drivers/time.h"
#include "drivers/nvic.h"
#include "drivers/timer.h"
#include "drivers/timer_impl.h"

void impl_timerNVICConfigure(uint8_t irq, int irqPriority)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(irqPriority);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(irqPriority);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void impl_timerConfigBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = (period - 1) & 0xffff; // AKA TIMx_ARR
    TIM_TimeBaseStructure.TIM_Prescaler = timerGetPrescalerByDesiredMhz(tim, mhz);
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

void impl_enableTimer(TIM_TypeDef * tim)
{
    TIM_Cmd(tim, ENABLE);
}

void impl_timerPWMStart(TIM_TypeDef * tim, unsigned channelIndex, bool isNChannel)
{
    UNUSED(channelIndex);
    UNUSED(isNChannel);
    TIM_CtrlPWMOutputs(tim, ENABLE);
}

void impl_timerEnableIT(TIM_TypeDef * tim, uint32_t interrupt)
{
    TIM_ITConfig(tim, interrupt, ENABLE);
}

void impl_timerDisableIT(TIM_TypeDef * tim, uint32_t interrupt)
{
    TIM_ITConfig(tim, interrupt, DISABLE);
}

void impl_timerClearFlag(TIM_TypeDef * tim, uint32_t flag)
{
    TIM_ClearFlag(tim, flag);
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
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = impl_timerLookupChannel(timHw->channelIndex);
    TIM_ICInitStructure.TIM_ICPolarity = polarityRising ? TIM_ICPolarity_Rising : TIM_ICPolarity_Falling;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = getFilter(inputFilterTicks);

    TIM_ICInit(timHw->tim, &TIM_ICInitStructure);
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
                case __builtin_clz(TIM_IT_Update):
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

void impl_timerPWMConfigChannel(TIM_TypeDef * tim, uint8_t channelIndex, bool isNChannel, bool inverted, uint16_t value)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse = value;

    if (isNChannel) {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_OCNPolarity = inverted ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
    } else {
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
        TIM_OCInitStructure.TIM_OCPolarity = inverted ? TIM_OCPolarity_Low : TIM_OCPolarity_High;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    }

    switch (channelIndex) {
        case 0:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case 1:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case 2:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case 3:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
    }
}

uint8_t impl_timerLookupChannel(uint8_t channelIndex)
{
    const uint8_t lookupTable[] = { TIM_Channel_1, TIM_Channel_2, TIM_Channel_3, TIM_Channel_4 };

    if (channelIndex <= CC_CHANNELS_PER_TIMER) {
        return lookupTable[channelIndex];
    }

    return 0;
}

uint16_t impl_timerDmaSource(uint8_t channelIndex)
{
    const uint16_t lookupTable[] = { TIM_DMA_CC1, TIM_DMA_CC2, TIM_DMA_CC3, TIM_DMA_CC4 };

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
