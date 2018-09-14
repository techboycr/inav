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

#include "common/color.h"

#include "drivers/io.h"
#include "drivers/nvic.h"
#include "drivers/dma.h"
#include "drivers/rcc.h"
#include "drivers/timer.h"
#include "drivers/system.h"
#include "drivers/light_ws2811strip.h"

static IO_t ws2811IO = IO_NONE;
static TCH_t * ws2811TCH = NULL;
bool ws2811Initialised = false;

void ws2811LedStripHardwareInit(void)
{
    const timerHardware_t * timHw = timerGetByTag(IO_TAG(WS2811_PIN), TIM_USE_ANY);

    if (timHw == NULL) {
        return;
    }

    ws2811TCH = timerGetTCH(timHw);
    if (ws2811TCH == NULL) {
        return;
    }

    /* Compute the prescaler value */
    uint16_t period = 1000000 * WS2811_TIMER_MHZ / WS2811_CARRIER_HZ;

    BIT_COMPARE_1 = period / 3 * 2;
    BIT_COMPARE_0 = period / 3;

    ws2811IO = IOGetByTag(IO_TAG(WS2811_PIN));
    IOInit(ws2811IO, OWNER_LED_STRIP, RESOURCE_OUTPUT, 0);
    IOConfigGPIOAF(ws2811IO, IOCFG_AF_PP_FAST, timHw->alternateFunction);

    timerConfigBase(ws2811TCH, period, WS2811_TIMER_MHZ);
    timerPWMConfigChannel(ws2811TCH, 0);
    timerPWMConfigChannelDMA(ws2811TCH, ledStripDMABuffer, WS2811_DMA_BUFFER_SIZE);

    ws2811Initialised = true;
}

bool ws2811LedStripDMAInProgress(void)
{
    return timerPWMDMAInProgress(ws2811TCH);
}

void ws2811LedStripDMAEnable(void)
{
    if (!ws2811Initialised || !ws2811TCH) {
        return;
    }

    timerPWMPrepareDMA(ws2811TCH, WS2811_DMA_BUFFER_SIZE);
    timerPWMStartDMA(ws2811TCH);
}

#endif
