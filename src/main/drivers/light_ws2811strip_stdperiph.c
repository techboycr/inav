/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "build/debug.h"

#ifdef USE_LED_STRIP

#include "drivers/io.h"
#include "drivers/nvic.h"

#include "common/color.h"
#include "light_ws2811strip.h"
#include "dma.h"
#include "drivers/system.h"
#include "rcc.h"
#include "timer.h"


static IO_t ws2811IO = IO_NONE;
bool ws2811Initialised = false;
static DMA_t dma = NULL;
static TIM_TypeDef *timer = NULL;

static void WS2811_DMA_IRQHandler(DMA_t descriptor)
{
    if (DMA_GET_FLAG_STATUS(descriptor, DMA_IT_TCIF)) {
        ws2811LedDataTransferInProgress = 0;
        dmaStopTransfer(descriptor);
        DMA_CLEAR_FLAG(descriptor, DMA_IT_TCIF);
    }
}

void ws2811LedStripHardwareInit(void)
{
    const timerHardware_t *timerHardware = timerGetByTag(IO_TAG(WS2811_PIN), TIM_USE_ANY);

    if (timerHardware == NULL) {
        return;
    }

    dma = dmaGetByTag(timerHardware->dmaTag);

    if (dma == NULL) {
        return;
    }
    
    timer = timerHardware->tim;

    ws2811IO = IOGetByTag(IO_TAG(WS2811_PIN));
    IOInit(ws2811IO, OWNER_LED_STRIP, RESOURCE_OUTPUT, 0);
    IOConfigGPIOAF(ws2811IO, IO_CONFIG(GPIO_Mode_AF, GPIO_Speed_50MHz, GPIO_OType_PP, GPIO_PuPd_UP), timerHardware->alternateFunction);

    // Stop timer
    TIM_Cmd(timer, DISABLE);

    /* Compute the prescaler value */
    uint16_t period = 1000000 * WS2811_TIMER_MHZ / WS2811_CARRIER_HZ;

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    /* PWM1 Mode configuration */
    timerConfigBase(timer, period, WS2811_TIMER_MHZ);
    timerPWMConfigChannel(timer, timerHardware->channel, timerHardware->output & TIMER_OUTPUT_N_CHANNEL, timerHardware->output & TIMER_OUTPUT_INVERTED, 0);

    TIM_CtrlPWMOutputs(timer, ENABLE);
    TIM_ARRPreloadConfig(timer, ENABLE);

    TIM_CCxCmd(timer, timerHardware->channel, TIM_CCx_Enable);
    TIM_Cmd(timer, ENABLE);

    dmaInit(dma, OWNER_LED_STRIP, 0);
    dmaSetHandler(dma, WS2811_DMA_IRQHandler, NVIC_PRIO_WS2811_DMA, 0);
    dmaSetupMemoryToPeripheralTransfer(timerHardware->dmaTag, (void *)timerCCR(timer, timerHardware->channel), (void *)ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE);

// #elif defined(STM32F3)
    // DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ledStripDMABuffer;
    // DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    // DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    // DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    // DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
// #endif

    TIM_DMACmd(timer, timerDmaSource(timerHardware->channel), ENABLE);
    ws2811Initialised = true;
}

void ws2811LedStripDMAEnable(void)
{
    if (!ws2811Initialised)
        return;

    // Start DMA transfer
    dmaStartTransfer(dma, WS2811_DMA_BUFFER_SIZE);

    // Enable TIM
    TIM_SetCounter(timer, 0);
    TIM_Cmd(timer, ENABLE);
}

#endif
