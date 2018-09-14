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

#pragma once

#if defined(USE_HAL_DRIVER)
#  define IMPL_TIM_IT_UPDATE_INTERRUPT      TIM_IT_UPDATE
#  define TIM_IT_CCx(ch)                    (TIM_IT_CC1 << ((ch) / 4))
#else
#  define IMPL_TIM_IT_UPDATE_INTERRUPT      TIM_IT_Update
#  define TIM_IT_CCx(ch)                    (TIM_IT_CC1 << ((ch) / 4))
#endif

#define _TIM_IRQ_HANDLER2(name, i, j)                                   \
    void name(void)                                                     \
    {                                                                   \
        impl_timerCaptureCompareHandler(TIM ## i, timerCtx[i - 1]); \
        impl_timerCaptureCompareHandler(TIM ## j, timerCtx[j - 1]); \
    } struct dummy

#define _TIM_IRQ_HANDLER(name, i)                                       \
    void name(void)                                                     \
    {                                                                   \
        impl_timerCaptureCompareHandler(TIM ## i, timerCtx[i - 1]); \
    } struct dummy

uint8_t lookupTimerIndex(const TIM_TypeDef *tim);
uint8_t impl_timerLookupChannel(uint8_t channelIndex);
void impl_timerNVICConfigure(uint8_t irq, int irqPriority);
void impl_timerConfigBase(TIM_TypeDef *tim, uint16_t period, uint8_t mhz);
void impl_enableTimer(TIM_TypeDef * tim);
void impl_timerEnableIT(TIM_TypeDef * tim, uint32_t interrupt);
void impl_timerDisableIT(TIM_TypeDef * tim, uint32_t interrupt);
void impl_timerClearFlag(TIM_TypeDef * tim, uint32_t flag);
void impl_timerChConfigIC(const timerHardware_t *timHw, bool polarityRising, unsigned inputFilterTicks);
void impl_timerCaptureCompareHandler(TIM_TypeDef *tim, timHardwareContext_t * timerCtx);
void impl_timerPWMConfigChannel(TIM_TypeDef * tim, uint8_t channelIndex, bool isNChannel, bool inverted, uint16_t value);
void impl_timerPWMStart(TIM_TypeDef * tim, unsigned channelIndex, bool isNChannel);
uint16_t impl_timerDmaSource(uint8_t channelIndex);
volatile timCCR_t * impl_timerCCR(TIM_TypeDef *tim, uint8_t channelIndex);
void impl_timerChCaptureCompareEnable(TCH_t * tch, bool enable);
bool impl_timerPWMConfigChannelDMA(TCH_t * tch, void * dmaBuffer, uint32_t dmaBufferSize);
void impl_timerPWMPrepareDMA(TCH_t * tch, uint32_t dmaBufferSize);
void impl_timerPWMStartDMA(TCH_t * tch);
void impl_timerPWMStopDMA(TCH_t * tch);
