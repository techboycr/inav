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

#include "drivers/dma.h"

// Macros expand to keep DMA descriptor table compatible with Betaflight
#define DEF_TIM_DMAMAP(variant, timch) CONCAT(DEF_TIM_DMAMAP__, PP_CALL(CONCAT(DEF_TIM_DMAMAP_VARIANT__, variant), CONCAT(DEF_TIM_DMA__BTCH_, timch), DMA_VARIANT_MISSING, DMA_VARIANT_MISSING))
#define DEF_TIM_DMAMAP_VARIANT__0(_0, ...)         _0
#define DEF_TIM_DMAMAP_VARIANT__1(_0, _1, ...)     _1
#define DEF_TIM_DMAMAP_VARIANT__2(_0, _1, _2, ...) _2

#define DEF_TIM_AF(timch, pin)                  CONCAT(DEF_TIM_AF__, DEF_TIM_AF__ ## pin ## __ ## timch)
#define DEF_TIM_AF__D(af_n)                     GPIO_AF_ ## af_n

#if defined(STM32F3)
    #include "timer_def_stm32f3xx.h"
#elif defined(STM32F4)
    #include "timer_def_stm32f4xx.h"
#elif defined(STM32F7)


//#include "timer_def_stm32f7xx.h"


#else
#error "Unknown CPU defined"
#endif

