/** @file HL_sys_main.c 
*   @brief Application main file
*   @date 07-July-2017
*   @version 04.07.00
*
*   This file contains an empty main function,
*   which can be used for the application.
*/

/* 
* Copyright (C) 2009-2016 Texas Instruments Incorporated - www.ti.com  
* 
* 
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*
*    Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the   
*    distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


/* USER CODE BEGIN (0) */
/* USER CODE END */

/* Include Files */

#include "HL_sys_common.h"

/* USER CODE BEGIN (1) */

#include "HL_sys_core.h"
#include "HL_sys_pmm.h"
#include "HL_spi.h"
#include "HL_esm.h"
#include "HL_rti.h"
#include "HL_gio.h"
#include "HL_sci.h"
#include "HL_adc.h"
#include "HL_het.h"
#include "HL_etpwm.h"
#include "HL_sys_dma.h"

#include <stdio.h>
/* USER CODE END */

/** @fn void main(void)
*   @brief Application main function
*   @note This function is empty by default.
*
*   This function is called after startup.
*   The user can use this function to implement the application.
*/

/* USER CODE BEGIN (2) */

void fsw_main(void);

/* USER CODE END */

int main(void)
{
/* USER CODE BEGIN (3) */
    initStackPattern(0xA5A5A5A5);

    // PD2 should not be powered down, because of debug logic (JTAG)
//     pmmTurnOFFLogicPowerDomain(PMM_LOGICPD2);
//     pmmTurnOFFLogicPowerDomain(PMM_LOGICPD3);
    pmmTurnOFFLogicPowerDomain(PMM_LOGICPD4);
    pmmTurnOFFLogicPowerDomain(PMM_LOGICPD5);
//    pmmTurnOFFLogicPowerDomain(PMM_LOGICPD6);

    // Initialize RTI driver
    rtiInit();

    // Start RTI Counters
    rtiStartBothCounters();

    // Reset DMA
    dmaREG->GCTRL = 1;
    dmaDisable();

    sciInit();
    spiInit();
    gioInit();
    adcInit();
    i2cInit();
    hetInit();

    // calibrate ADCs
    adcCalibration(adcREG1);
    adcCalibration(adcREG2);

    // Allow Group 1 Memory Overrun
    adcREG1->GxMODECR[1U] |= 0x00000010;

    // Start ADC1 conversions
    adcStartConversion(adcREG1, adcGROUP1);

    // Allow Group 1 Memory Overrun
    adcREG2->GxMODECR[1U] |= 0x00000010;

    // Start ADC2 conversions
    adcStartConversion(adcREG2, adcGROUP1);

    // Call out to main()
	fsw_main();

//     printf("med_res_timestamp=%u high_res_timestamp=%u seconds=%u\n", rtiGetMedResTimestamp(),rtiGetMedResLow(),rtiGetMedResLow(), rtiGetHighResTimestamp(), rtiGetHighResSeconds());

/* USER CODE END */

    return 0;
}


/* USER CODE BEGIN (4) */
/* USER CODE END */
