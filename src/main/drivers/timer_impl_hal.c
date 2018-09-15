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

extern uint32_t timerClock(TIM_TypeDef *tim);
const uint16_t lookupDMAIndexTable[] = { TIM_DMA_ID_CC1, TIM_DMA_ID_CC2, TIM_DMA_ID_CC3, TIM_DMA_ID_CC4 };
const uint16_t lookupDMASourceTable[] = { TIM_DMA_CC1, TIM_DMA_CC2, TIM_DMA_CC3, TIM_DMA_CC4 };
const uint8_t lookupTIMChannelTable[] = { TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4 };
static TIM_HandleTypeDef timerHandle[HARDWARE_TIMER_DEFINITION_COUNT];

static TIM_HandleTypeDef * timerFindTimerHandle(TIM_TypeDef *tim)
{
    uint8_t timerIndex = lookupTimerIndex(tim);
    if (timerIndex >= HARDWARE_TIMER_DEFINITION_COUNT) {
        return NULL;
    }

    return &timerHandle[timerIndex];
}

void impl_timerInitContext(timHardwareContext_t * timCtx)
{
    timCtx->timHandle = timerFindTimerHandle(timCtx->timDef->tim);
}

void impl_timerNVICConfigure(TCH_t * tch, int irqPriority)
{
    if (tch->timCtx->timDef->irq) {
        HAL_NVIC_SetPriority(tch->timCtx->timDef->irq, NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));
        HAL_NVIC_EnableIRQ(tch->timCtx->timDef->irq);
    }

    if (tch->timCtx->timDef->secondIrq) {
        HAL_NVIC_SetPriority(tch->timCtx->timDef->secondIrq, NVIC_PRIORITY_BASE(irqPriority), NVIC_PRIORITY_SUB(irqPriority));
        HAL_NVIC_EnableIRQ(tch->timCtx->timDef->secondIrq);
    }
}

void impl_timerConfigBase(TCH_t * tch, uint16_t period, uint8_t mhz)
{
    // Get and verify HAL TIM_Handle object 
    TIM_HandleTypeDef * timHandle = tch->timCtx->timHandle;
    TIM_TypeDef * timer = tch->timCtx->timDef->tim;

    if (timHandle->Instance == timer) {
        return;
    }

    timHandle->Instance = timer;
    timHandle->Init.Prescaler = (timerClock(timer) / ((uint32_t)mhz * 1000000)) - 1;
    timHandle->Init.Period = (period - 1) & 0xffff; // AKA TIMx_ARR
    timHandle->Init.RepetitionCounter = 0;
    timHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
    timHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    HAL_TIM_Base_Init(timHandle);
    if (timer == TIM1 || timer == TIM2 || timer == TIM3 || timer == TIM4 || timer == TIM5 || timer == TIM8 || timer == TIM9) {
        TIM_ClockConfigTypeDef sClockSourceConfig;
        memset(&sClockSourceConfig, 0, sizeof(sClockSourceConfig));
        sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
        if (HAL_TIM_ConfigClockSource(timHandle, &sClockSourceConfig) != HAL_OK) {
            return;
        }
    }

    if (timer == TIM1 || timer == TIM2 || timer == TIM3 || timer == TIM4 || timer == TIM5 || timer == TIM8) {
        TIM_MasterConfigTypeDef sMasterConfig;
        memset(&sMasterConfig, 0, sizeof(sMasterConfig));
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(timHandle, &sMasterConfig) != HAL_OK) {
            return;
        }
    }
}

void impl_timerPWMConfigChannel(TCH_t * tch, uint16_t value)
{
    const bool inverted = tch->timHw->output & TIMER_OUTPUT_INVERTED;

    TIM_OC_InitTypeDef TIM_OCInitStructure;

    TIM_OCInitStructure.OCMode = TIM_OCMODE_PWM1;
    TIM_OCInitStructure.OCIdleState = TIM_OCIDLESTATE_SET;
    TIM_OCInitStructure.OCPolarity = inverted ? TIM_OCPOLARITY_LOW : TIM_OCPOLARITY_HIGH;
    TIM_OCInitStructure.OCNIdleState = TIM_OCNIDLESTATE_SET;
    TIM_OCInitStructure.OCNPolarity = inverted ? TIM_OCNPOLARITY_LOW : TIM_OCNPOLARITY_HIGH;
    TIM_OCInitStructure.Pulse = value;
    TIM_OCInitStructure.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(tch->timCtx->timHandle, &TIM_OCInitStructure, lookupTIMChannelTable[tch->timHw->channelIndex]);
}

volatile timCCR_t * impl_timerCCR(TCH_t * tch)
{
    switch (tch->timHw->channelIndex) {
        case 0:
            return &tch->timHw->tim->CCR1;
            break;
        case 1:
            return &tch->timHw->tim->CCR2;
            break;
        case 2:
            return &tch->timHw->tim->CCR3;
            break;
        case 3:
            return &tch->timHw->tim->CCR4;
            break;
    }
    return NULL;
}

void impl_enableTimer(TCH_t * tch)
{
    HAL_TIM_Base_Start(tch->timCtx->timHandle);
}

void impl_timerPWMStart(TCH_t * tch)
{
    if (tch->timHw->output & TIMER_OUTPUT_N_CHANNEL) {
        HAL_TIMEx_PWMN_Start(tch->timCtx->timHandle, lookupTIMChannelTable[tch->timHw->channelIndex]);
    }
    else {
        HAL_TIM_PWM_Start(tch->timCtx->timHandle, lookupTIMChannelTable[tch->timHw->channelIndex]);
    }
}

void impl_timerEnableIT(TCH_t * tch, uint32_t interrupt)
{
    __HAL_TIM_ENABLE_IT(tch->timCtx->timHandle, interrupt);
}

void impl_timerDisableIT(TCH_t * tch, uint32_t interrupt)
{
    __HAL_TIM_DISABLE_IT(tch->timCtx->timHandle, interrupt);
}

void impl_timerClearFlag(TCH_t * tch, uint32_t flag)
{
    __HAL_TIM_CLEAR_FLAG(tch->timCtx->timHandle, flag);
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

    for (unsigned i = 1; i < ARRAYLEN(ftab); i++) {
        if (ftab[i] > ticks) {
            return i - 1;
        }
    }

    return 0x0f;
}

void impl_timerChConfigIC(TCH_t * tch, bool polarityRising, unsigned inputFilterTicks)
{
    TIM_IC_InitTypeDef TIM_ICInitStructure;

    TIM_ICInitStructure.ICPolarity = polarityRising ? TIM_ICPOLARITY_RISING : TIM_ICPOLARITY_FALLING;
    TIM_ICInitStructure.ICSelection = TIM_ICSELECTION_DIRECTTI;
    TIM_ICInitStructure.ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.ICFilter = getFilter(inputFilterTicks);
    HAL_TIM_IC_ConfigChannel(tch->timCtx->timHandle, &TIM_ICInitStructure, lookupTIMChannelTable[tch->timHw->channelIndex]);
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
                case __builtin_clz(TIM_IT_UPDATE): {
                    const uint16_t capture = tim->ARR;
                    if (timerCtx->ch[0].cb && timerCtx->ch[0].cb->callbackOvr) {
                        timerCtx->ch[0].cb->callbackOvr(&timerCtx->ch[0], capture);
                    }
                    if (timerCtx->ch[1].cb && timerCtx->ch[1].cb->callbackOvr) {
                        timerCtx->ch[1].cb->callbackOvr(&timerCtx->ch[1], capture);
                    }
                    if (timerCtx->ch[2].cb && timerCtx->ch[2].cb->callbackOvr) {
                        timerCtx->ch[2].cb->callbackOvr(&timerCtx->ch[2], capture);
                    }
                    if (timerCtx->ch[3].cb && timerCtx->ch[3].cb->callbackOvr) {
                        timerCtx->ch[3].cb->callbackOvr(&timerCtx->ch[3], capture);
                    }
                    break;
                }
                case __builtin_clz(TIM_IT_CC1):
                    timerCtx->ch[0].cb->callbackEdge(&timerCtx->ch[0], tim->CCR1);
                    break;
                case __builtin_clz(TIM_IT_CC2):
                    timerCtx->ch[1].cb->callbackEdge(&timerCtx->ch[1], tim->CCR2);
                    break;
                case __builtin_clz(TIM_IT_CC3):
                    timerCtx->ch[2].cb->callbackEdge(&timerCtx->ch[2], tim->CCR3);
                    break;
                case __builtin_clz(TIM_IT_CC4):
                    timerCtx->ch[3].cb->callbackEdge(&timerCtx->ch[3], tim->CCR4);
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

void impl_timerChCaptureCompareEnable(TCH_t * tch, bool enable)
{
    TIM_CCxChannelCmd(tch->timHw->tim, lookupTIMChannelTable[tch->timHw->channelIndex], (enable ? TIM_CCx_ENABLE : TIM_CCx_DISABLE));
}

HAL_StatusTypeDef TIM_DMACmd(TIM_HandleTypeDef *htim, uint32_t Channel, FunctionalState NewState)
{
    switch (Channel) {
    case TIM_CHANNEL_1: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 1 DMA request */
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC1);
        } else {
            /* Disable the TIM Capture/Compare 1 DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC1);
        }
    }
        break;

    case TIM_CHANNEL_2: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 2 DMA request */
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC2);
        } else {
            /* Disable the TIM Capture/Compare 2 DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC2);
        }
    }
        break;

    case TIM_CHANNEL_3: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 3 DMA request */
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC3);
        } else {
            /* Disable the TIM Capture/Compare 3 DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC3);
        }
    }
        break;

    case TIM_CHANNEL_4: {
        if (NewState != DISABLE) {
            /* Enable the TIM Capture/Compare 4 DMA request */
            __HAL_TIM_ENABLE_DMA(htim, TIM_DMA_CC4);
        } else {
            /* Disable the TIM Capture/Compare 4 DMA request */
            __HAL_TIM_DISABLE_DMA(htim, TIM_DMA_CC4);
        }
    }
        break;

    default:
        break;
    }
    /* Change the htim state */
    htim->State = HAL_TIM_STATE_READY;
    /* Return function status */
    return HAL_OK;
}

HAL_StatusTypeDef DMA_SetCurrDataCounter(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length)
{
    if ((htim->State == HAL_TIM_STATE_BUSY)) {
        return HAL_BUSY;
    } else if ((htim->State == HAL_TIM_STATE_READY)) {
        if (((uint32_t) pData == 0) && (Length > 0)) {
            return HAL_ERROR;
        } else {
            htim->State = HAL_TIM_STATE_BUSY;
        }
    }
    switch (Channel) {
    case TIM_CHANNEL_1: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TIM_DMA_ID_CC1]->XferCpltCallback = HAL_TIM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC1]->XferErrorCallback = HAL_TIM_DMAError;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC1], (uint32_t) pData, (uint32_t) & htim->Instance->CCR1, Length);
    }
        break;

    case TIM_CHANNEL_2: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TIM_DMA_ID_CC2]->XferCpltCallback = HAL_TIM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC2]->XferErrorCallback = HAL_TIM_DMAError;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC2], (uint32_t) pData, (uint32_t) & htim->Instance->CCR2, Length);
    }
        break;

    case TIM_CHANNEL_3: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TIM_DMA_ID_CC3]->XferCpltCallback = HAL_TIM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC3]->XferErrorCallback = HAL_TIM_DMAError;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC3], (uint32_t) pData, (uint32_t) & htim->Instance->CCR3, Length);
    }
        break;

    case TIM_CHANNEL_4: {
        /* Set the DMA Period elapsed callback */
        htim->hdma[TIM_DMA_ID_CC4]->XferCpltCallback = HAL_TIM_DMADelayPulseCplt;

        /* Set the DMA error callback */
        htim->hdma[TIM_DMA_ID_CC4]->XferErrorCallback = HAL_TIM_DMAError;

        /* Enable the DMA Stream */
        HAL_DMA_Start_IT(htim->hdma[TIM_DMA_ID_CC4], (uint32_t) pData, (uint32_t) & htim->Instance->CCR4, Length);
    }
        break;

    default:
        break;
    }
    /* Return function status */
    return HAL_OK;
}

static void impl_timerDMA_IRQHandler(DMA_t descriptor)
{
    TCH_t * tch = (TCH_t *)descriptor->userParam;
    TIM_HandleTypeDef * timHandle = tch->timCtx->timHandle;
    uint16_t timerDmaIndex = lookupDMAIndexTable[tch->timHw->channelIndex];

    HAL_DMA_IRQHandler(timHandle->hdma[timerDmaIndex]);

    if (tch->timHw->output & TIMER_OUTPUT_N_CHANNEL) {
        HAL_TIMEx_PWMN_Stop_DMA(timHandle, lookupTIMChannelTable[tch->timHw->channelIndex]);
    } else {
        HAL_TIM_PWM_Stop_DMA(timHandle, lookupTIMChannelTable[tch->timHw->channelIndex]);
    }

    tch->dmaState = TCH_DMA_IDLE;
}

bool impl_timerPWMConfigChannelDMA(TCH_t * tch, void * dmaBuffer, uint32_t dmaBufferSize)
{
    (void)dmaBufferSize;

    tch->dma = dmaGetByTag(tch->timHw->dmaTag);
    tch->dmaBuffer = dmaBuffer;
    if (tch->dma == NULL) {
        return false;
    }

    // We assume that timer channels are already initialized by calls to:
    //  timerConfigBase
    //  timerPWMConfigChannel

    TIM_HandleTypeDef * timHandle = tch->timCtx->timHandle;
    DMA_HandleTypeDef * hdma_tim = &tch->dma->hdma;

    /* Set the parameters to be configured */
    hdma_tim->Instance = tch->dma->ref;
    hdma_tim->Init.Channel = dmaGetChannelByTag(tch->timHw->dmaTag);

    hdma_tim->Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_tim->Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_tim->Init.MemInc = DMA_MINC_ENABLE;
    hdma_tim->Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_tim->Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_tim->Init.Mode = DMA_NORMAL;
    hdma_tim->Init.Priority = DMA_PRIORITY_HIGH;
    hdma_tim->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    hdma_tim->Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma_tim->Init.MemBurst = DMA_MBURST_SINGLE;
    hdma_tim->Init.PeriphBurst = DMA_PBURST_SINGLE;

    /* Link hdma_tim to hdma[x] (channelx) */
    int dmaIndex = lookupDMAIndexTable[tch->timHw->channelIndex];
    __HAL_LINKDMA(timHandle, hdma[dmaIndex], *hdma_tim);

    dmaInit(tch->dma, OWNER_TIMER, 0);
    dmaSetHandler(tch->dma, impl_timerDMA_IRQHandler, NVIC_PRIO_WS2811_DMA, (uint32_t)tch);

    /* Initialize TIMx DMA handle */
    HAL_DMA_Init(timHandle->hdma[dmaIndex]);

    return true;
}

void impl_timerPWMPrepareDMA(TCH_t * tch, uint32_t dmaBufferSize)
{
    tch->dmaState = TCH_DMA_READY;
    DMA_SetCurrDataCounter(tch->timCtx->timHandle, lookupTIMChannelTable[tch->timHw->channelIndex], tch->dmaBuffer, dmaBufferSize);
}

void impl_timerPWMStartDMA(TCH_t * tch)
{
    uint16_t dmaSources = 0;
    timHardwareContext_t * timCtx = tch->timCtx;

    if (timCtx->ch[0].dmaState == TCH_DMA_READY) {
        timCtx->ch[0].dmaState = TCH_DMA_ACTIVE;
        dmaSources |= TIM_DMA_CC1;
    }

    if (timCtx->ch[1].dmaState == TCH_DMA_READY) {
        timCtx->ch[1].dmaState = TCH_DMA_ACTIVE;
        dmaSources |= TIM_DMA_CC2;
    }

    if (timCtx->ch[2].dmaState == TCH_DMA_READY) {
        timCtx->ch[2].dmaState = TCH_DMA_ACTIVE;
        dmaSources |= TIM_DMA_CC3;
    }

    if (timCtx->ch[3].dmaState == TCH_DMA_READY) {
        timCtx->ch[3].dmaState = TCH_DMA_ACTIVE;
        dmaSources |= TIM_DMA_CC4;
    }

    if (dmaSources) {
        LL_TIM_SetCounter(tch->timHw->tim, 0);
        tch->timHw->tim->DIER |= dmaSources;
    }
}

void impl_timerPWMStopDMA(TCH_t * tch)
{
    (void)tch;
    // FIXME
}
