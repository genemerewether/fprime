/** @file HL_het.c
*   @brief HET Driver Implementation File
*   @date 07-July-2017
*   @version 04.07.00
*
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


#include "HL_het.h"
#include "het1_prog.h"
#include "HL_sys_vim.h"
/* USER CODE BEGIN (0) */
/* USER CODE END */

/*----------------------------------------------------------------------------*/
/* Global variables                                                           */

static const uint32 s_het1pwmPolarity[8U] =
{
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
    3U,
};




/** @fn void hetInit(void)
*   @brief Initializes the het Driver
*
*   This function initializes the het 1 module.
*/
/* SourceId : HET_SourceId_001 */
/* DesignId : HET_DesignId_001 */
/* Requirements : HL_CONQ_HET_SR10 */
void hetInit(void)
{
    /** @b initialize @b HET */

    /** - Set HET pins default output value */
    hetREG1->DOUT = (uint32)((uint32)0U << 31U)
                  | (uint32)((uint32)0U << 30U)
                  | (uint32)((uint32)0U << 29U)
                  | (uint32)((uint32)0U << 28U)
                  | (uint32)((uint32)0U << 27U)
                  | (uint32)((uint32)0U << 26U)
                  | (uint32)((uint32)0U << 25U)
                  | (uint32)((uint32)0U << 24U)
                  | (uint32)((uint32)0U << 23U)
                  | (uint32)((uint32)0U << 22U)
                  | (uint32)((uint32)0U << 21U)
                  | (uint32)((uint32)0U << 20U)
                  | (uint32)((uint32)0U << 19U)
                  | (uint32)((uint32)0U << 18U)
                  | (uint32)((uint32)0U << 17U)
                  | (uint32)((uint32)0U << 16U)
                  | (uint32)((uint32)0U << 15U)
                  | (uint32)((uint32)0U << 14U)
                  | (uint32)((uint32)0U << 13U)
                  | (uint32)((uint32)0U << 12U)
                  | (uint32)((uint32)0U << 11U)
                  | (uint32)((uint32)0U << 10U)
                  | (uint32)((uint32)0U << 9U)
                  | (uint32)((uint32)0U << 8U)
                  | (uint32)((uint32)0U << 7U)
                  | (uint32)((uint32)0U << 6U)
                  | (uint32)((uint32)0U << 5U)
                  | (uint32)((uint32)0U << 4U)
                  | (uint32)((uint32)0U << 3U)
                  | (uint32)((uint32)0U << 2U)
                  | (uint32)((uint32)0U << 1U)
                  | (uint32)((uint32)0U << 0U);

    /** - Set HET pins direction */
    hetREG1->DIR = (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00008000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00002000U
                 | (uint32) 0x00001000U
                 | (uint32) 0x00000800U
                 | (uint32) 0x00000400U
                 | (uint32) 0x00000200U
                 | (uint32) 0x00000100U
                 | (uint32) 0x00000080U
                 | (uint32) 0x00000040U
                 | (uint32) 0x00000020U
                 | (uint32) 0x00000010U
                 | (uint32) 0x00000008U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000002U
                 | (uint32) 0x00000001U;

    /** - Set HET pins open drain enable */
    hetREG1->PDR = (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U;

    /** - Set HET pins pullup/down enable */
    hetREG1->PULDIS = (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U
                    | (uint32) 0x00000000U;

    /** - Set HET pins pullup/down select */
    hetREG1->PSL = (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U;

    /** - Set HET pins high resolution share */
    hetREG1->HRSH = (uint32) 0x00008000U
                  | (uint32) 0x00004000U
                  | (uint32) 0x00002000U
                  | (uint32) 0x00001000U
                  | (uint32) 0x00000000U
                  | (uint32) 0x00000000U
                  | (uint32) 0x00000000U
                  | (uint32) 0x00000000U
                  | (uint32) 0x00000000U
                  | (uint32) 0x00000000U
                  | (uint32) 0x00000000U
                  | (uint32) 0x00000000U
                  | (uint32) 0x00000008U
                  | (uint32) 0x00000004U
                  | (uint32) 0x00000000U
                  | (uint32) 0x00000000U;

    /** - Set HET pins AND share */
    hetREG1->AND = (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U;

    /** - Set HET pins XOR share */
    hetREG1->XOR = (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U
                 | (uint32) 0x00000000U;

/* USER CODE BEGIN (1) */
/* USER CODE END */

    /** - Setup prescaler values
    *     - Loop resolution prescaler
    *     - High resolution prescaler
    */
    hetREG1->PFR = (uint32)((uint32) 6U << 8U)
                | ((uint32) 0U);


   /** - Parity control register
    *     - Enable/Disable Parity check
    */
    hetREG1->PCR = (uint32) 0x00000005U;

    /** - Fill HET RAM with opcodes and Data */
    /** - "x" in "HET_INITx_PST" denote the HET module Instance
    *      Valid range of x - 0 to 9
    *      For HET1 module x = 0
    *      Refer HET assembler User guide for more Info
    */
    /*SAFETYMCUSW 94 S MR:11.1,11.2,11.4 <APPROVED> "HET RAM Fill from the table - Allowed as per MISRA rule 11.2" */
    /*SAFETYMCUSW 95 S MR:11.1,11.4 <APPROVED> "HET RAM Fill from the table - Allowed as per MISRA rule 11.2" */
    /*SAFETYMCUSW 95 S MR:11.1,11.4 <APPROVED> "HET RAM Fill from the table - Allowed as per MISRA rule 11.2" */
    (void)memcpy((void*)hetRAM1, (void*)HET_INIT0_PST, sizeof(HET_INIT0_PST));

    /** - Setup interrupt priority level */
    hetREG1->PRY = 0xFFFFFFFF;

    /** - Enable interrupts */
    hetREG1->INTENAC = 0xFFFFFFFFU;
    hetREG1->INTENAS = (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U
                     | (uint32) 0x00000000U;


   /** - Setup control register
    *     - Enable output buffers
    *     - Ignore software breakpoints
    *     - Master or Slave Clock Mode
    *     - Enable HET
    */
    hetREG1->GCR = ( 0x00000001U 
                   | (uint32)((uint32)0U << 24U)
	               | (uint32)((uint32)1U << 16U)
                   | (0x00000000U));


/* USER CODE BEGIN (4) */
/* USER CODE END */

}

/** @fn void pwmStart( hetRAMBASE_t * hetRAM, uint32 pwm)
*   @brief Start pwm signal
*   @param[in] hetRAM Pointer to HET RAM:
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*   @param[in] pwm Pwm signal:
*              - pwm0: Pwm 0
*              - pwm1: Pwm 1
*              - pwm2: Pwm 2
*              - pwm3: Pwm 3
*              - pwm4: Pwm 4
*              - pwm5: Pwm 5
*              - pwm6: Pwm 6
*              - pwm7: Pwm 7
*
*   Start the given pwm signal
*/
/* SourceId : HET_SourceId_002 */
/* DesignId : HET_DesignId_002 */
/* Requirements : HL_CONQ_HET_SR11 */
void pwmStart( hetRAMBASE_t * hetRAM, uint32 pwm)
{

   hetRAM->Instruction[(pwm << 1U) + 41U].Control  |= 0x00400000U;
}


/** @fn void pwmStop( hetRAMBASE_t * hetRAM, uint32 pwm)
*   @brief Stop pwm signal
*   @param[in] hetRAM Pointer to HET RAM:
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*   @param[in] pwm Pwm signal:
*              - pwm0: Pwm 0
*              - pwm1: Pwm 1
*              - pwm2: Pwm 2
*              - pwm3: Pwm 3
*              - pwm4: Pwm 4
*              - pwm5: Pwm 5
*              - pwm6: Pwm 6
*              - pwm7: Pwm 7
*
*   Stop the given pwm signal
*/
/* SourceId : HET_SourceId_003 */
/* DesignId : HET_DesignId_003 */
/* Requirements : HL_CONQ_HET_SR12 */
void pwmStop( hetRAMBASE_t * hetRAM, uint32 pwm)
{
    hetRAM->Instruction[(pwm << 1U) + 41U].Control  &= ~(uint32)0x00400000U;
}


/** @fn void pwmSetDuty(hetRAMBASE_t * hetRAM, uint32 pwm, uint32 pwmDuty)
*   @brief Set duty cycle
*   @param[in] hetRAM Pointer to HET RAM:
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*   @param[in] pwm Pwm signal:
*              - pwm0: Pwm 0
*              - pwm1: Pwm 1
*              - pwm2: Pwm 2
*              - pwm3: Pwm 3
*              - pwm4: Pwm 4
*              - pwm5: Pwm 5
*              - pwm6: Pwm 6
*              - pwm7: Pwm 7
*   @param[in] pwmDuty duty cycle in %.
*
*   Sets a new duty cycle on the given pwm signal
*/
/* SourceId : HET_SourceId_004 */
/* DesignId : HET_DesignId_004 */
/* Requirements : HL_CONQ_HET_SR13 */
void pwmSetDuty(hetRAMBASE_t * hetRAM, uint32 pwm, uint32 pwmDuty)
{
    uint32 action;
    uint32 pwmPolarity =0U;
    uint32 pwmPeriod = hetRAM->Instruction[(pwm << 1U) + 42U].Data + 128U;
    pwmPeriod = pwmPeriod >> 7U;

    if(hetRAM == hetRAM1)
    {
        pwmPolarity = s_het1pwmPolarity[pwm];
    }
    else
    {
    }
    if (pwmDuty == 0U)
    {
        action = (pwmPolarity == 3U) ? 0U : 2U;
    }
    else if (pwmDuty >= 100U)
    {
        action = (pwmPolarity == 3U) ? 2U : 0U;
    }
    else
    {
        action = pwmPolarity;
    }

    hetRAM->Instruction[(pwm << 1U) + 41U].Control = ((hetRAM->Instruction[(pwm << 1U) + 41U].Control) & (~(uint32)(0x00000018U))) | (action << 3U);
    hetRAM->Instruction[(pwm << 1U) + 41U].Data = (((pwmPeriod * pwmDuty) / 100U) << 7U) + 128U;
}


/** @fn void pwmSetSignal(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t signal)
*   @brief Set period
*   @param[in] hetRAM Pointer to HET RAM:
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*   @param[in] pwm Pwm signal:
*              - pwm0: Pwm 0
*              - pwm1: Pwm 1
*              - pwm2: Pwm 2
*              - pwm3: Pwm 3
*              - pwm4: Pwm 4
*              - pwm5: Pwm 5
*              - pwm6: Pwm 6
*              - pwm7: Pwm 7
*   @param[in] signal signal
               - duty cycle in %.
*              - period period in us.
*
*   Sets a new pwm signal
*/
/* SourceId : HET_SourceId_005 */
/* DesignId : HET_DesignId_005 */
/* Requirements : HL_CONQ_HET_SR14 */
void pwmSetSignal(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t signal)
{
    uint32 action;
    uint32 pwmPolarity = 0U;
    float64 pwmPeriod = 0.0F;

    if(hetRAM == hetRAM1)
    {
        pwmPeriod = (signal.period * 1000.0F) / 853.333F;
        pwmPolarity = s_het1pwmPolarity[pwm];
    }
    else
    {
    }
    if (signal.duty == 0U)
    {
        action = (pwmPolarity == 3U) ? 0U : 2U;
    }
    else if (signal.duty >= 100U)
    {
        action = (pwmPolarity == 3U) ? 2U : 0U;
    }
    else
    {
        action = pwmPolarity;
    }

    hetRAM->Instruction[(pwm << 1U) + 41U].Control = ((hetRAM->Instruction[(pwm << 1U) + 41U].Control) & (~(uint32)(0x00000018U))) | (action << 3U);
    hetRAM->Instruction[(pwm << 1U) + 41U].Data = ((((uint32)pwmPeriod * signal.duty) / 100U) << 7U ) + 128U;
    hetRAM->Instruction[(pwm << 1U) + 42U].Data = ((uint32)pwmPeriod << 7U) - 128U;

}


/** @fn void pwmGetSignal(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t signal)
*   @brief Get duty cycle
*   @param[in] hetRAM Pointer to HET RAM:
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*   @param[in] pwm Pwm signal:
*              - pwm0: Pwm 0
*              - pwm1: Pwm 1
*              - pwm2: Pwm 2
*              - pwm3: Pwm 3
*              - pwm4: Pwm 4
*              - pwm5: Pwm 5
*              - pwm6: Pwm 6
*              - pwm7: Pwm 7
*   @param[in] signal signal
*              - duty cycle in %.
*              - period period in us.
*
*   Gets current signal of the given pwm signal.
*/
/* SourceId : HET_SourceId_006 */
/* DesignId : HET_DesignId_006 */
/* Requirements : HL_CONQ_HET_SR15 */
void pwmGetSignal(hetRAMBASE_t * hetRAM, uint32 pwm, hetSIGNAL_t* signal)
{
    uint32    pwmDuty   = (hetRAM->Instruction[(pwm << 1U) + 41U].Data - 128U) >> 7U;
    uint32    pwmPeriod = (hetRAM->Instruction[(pwm << 1U) + 42U].Data + 128U) >> 7U;

    signal->duty   = (pwmDuty * 100U) / pwmPeriod;

    if(hetRAM == hetRAM1)
    {
        signal->period = ((float64)pwmPeriod * 853.333F) / 1000.0F;
    }
    else
    {
        signal->period = ((float64)pwmPeriod * 853.333F) / 1000.0F;
    }
}

/** @fn void pwmEnableNotification(hetBASE_t * hetREG, uint32 pwm, uint32 notification)
*   @brief Enable pwm notification
*   @param[in] hetREG Pointer to HET Module:
*              - hetREG1: HET1 Module pointer
*              - hetREG2: HET2 Module pointer
*   @param[in] pwm Pwm signal:
*              - pwm0: Pwm 0
*              - pwm1: Pwm 1
*              - pwm2: Pwm 2
*              - pwm3: Pwm 3
*              - pwm4: Pwm 4
*              - pwm5: Pwm 5
*              - pwm6: Pwm 6
*              - pwm7: Pwm 7
*   @param[in] notification Pwm notification:
*              - pwmEND_OF_DUTY:   Notification on end of duty
*              - pwmEND_OF_PERIOD: Notification on end of end period
*              - pwmEND_OF_BOTH:   Notification on end of both duty and period
*/
/* SourceId : HET_SourceId_007 */
/* DesignId : HET_DesignId_007 */
/* Requirements : HL_CONQ_HET_SR16 */
void pwmEnableNotification(hetBASE_t * hetREG, uint32 pwm, uint32 notification)
{
    hetREG->FLG     = notification << (pwm << 1U);
    hetREG->INTENAS = notification << (pwm << 1U);
}


/** @fn void pwmDisableNotification(hetBASE_t * hetREG, uint32 pwm, uint32 notification)
*   @brief Enable pwm notification
*   @param[in] hetREG Pointer to HET Module:
*              - hetREG1: HET1 Module pointer
*              - hetREG2: HET2 Module pointer
*   @param[in] pwm Pwm signal:
*              - pwm0: Pwm 0
*              - pwm1: Pwm 1
*              - pwm2: Pwm 2
*              - pwm3: Pwm 3
*              - pwm4: Pwm 4
*              - pwm5: Pwm 5
*              - pwm6: Pwm 6
*              - pwm7: Pwm 7
*   @param[in] notification Pwm notification:
*              - pwmEND_OF_DUTY:   Notification on end of duty
*              - pwmEND_OF_PERIOD: Notification on end of end period
*              - pwmEND_OF_BOTH:   Notification on end of both duty and period
*/
/* SourceId : HET_SourceId_008 */
/* DesignId : HET_DesignId_008 */
/* Requirements : HL_CONQ_HET_SR17 */
void pwmDisableNotification(hetBASE_t * hetREG, uint32 pwm, uint32 notification)
{
    hetREG->INTENAC = notification << (pwm << 1U);
}


/** @fn void edgeResetCounter(hetRAMBASE_t * hetRAM, uint32 edge)
*   @brief Resets edge counter to 0
*   @param[in] hetRAM Pointer to HET RAM:
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*   @param[in] edge Edge signal:
*              - edge0: Edge 0
*              - edge1: Edge 1
*              - edge2: Edge 2
*              - edge3: Edge 3
*              - edge4: Edge 4
*              - edge5: Edge 5
*              - edge6: Edge 6
*              - edge7: Edge 7
*
*   Reset edge counter to 0.
*/
/* SourceId : HET_SourceId_009 */
/* DesignId : HET_DesignId_009 */
/* Requirements : HL_CONQ_HET_SR19 */
void edgeResetCounter(hetRAMBASE_t * hetRAM, uint32 edge)
{
    hetRAM->Instruction[edge + 17U].Data = 0U;
}


/** @fn uint32 edgeGetCounter(hetRAMBASE_t * hetRAM, uint32 edge)
*   @brief Get current edge counter value
*   @param[in] hetRAM Pointer to HET RAM:
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*   @param[in] edge Edge signal:
*              - edge0: Edge 0
*              - edge1: Edge 1
*              - edge2: Edge 2
*              - edge3: Edge 3
*              - edge4: Edge 4
*              - edge5: Edge 5
*              - edge6: Edge 6
*              - edge7: Edge 7
*
*   Gets current edge counter value.
*/
/* SourceId : HET_SourceId_010 */
/* DesignId : HET_DesignId_010 */
/* Requirements : HL_CONQ_HET_SR20 */
uint32 edgeGetCounter(hetRAMBASE_t * hetRAM, uint32 edge)
{
    return hetRAM->Instruction[edge + 17U].Data >> 7U;
}


/** @fn void edgeEnableNotification(hetBASE_t * hetREG, uint32 edge)
*   @brief Enable edge notification
*   @param[in] hetREG Pointer to HET Module:
*              - hetREG1: HET1 Module pointer
*              - hetREG2: HET2 Module pointer
*   @param[in] edge Edge signal:
*              - edge0: Edge 0
*              - edge1: Edge 1
*              - edge2: Edge 2
*              - edge3: Edge 3
*              - edge4: Edge 4
*              - edge5: Edge 5
*              - edge6: Edge 6
*              - edge7: Edge 7
*/
/* SourceId : HET_SourceId_011 */
/* DesignId : HET_DesignId_011 */
/* Requirements : HL_CONQ_HET_SR21 */
void edgeEnableNotification(hetBASE_t * hetREG, uint32 edge)
{
    hetREG->FLG     = (uint32)0x20000U << edge;
    hetREG->INTENAS = (uint32)0x20000U << edge;
}


/** @fn void edgeDisableNotification(hetBASE_t * hetREG, uint32 edge)
*   @brief Enable edge notification
*   @param[in] hetREG Pointer to HET Module:
*              - hetREG1: HET1 Module pointer
*              - hetREG2: HET2 Module pointer
*   @param[in] edge Edge signal:
*              - edge0: Edge 0
*              - edge1: Edge 1
*              - edge2: Edge 2
*              - edge3: Edge 3
*              - edge4: Edge 4
*              - edge5: Edge 5
*              - edge6: Edge 6
*              - edge7: Edge 7
*/
/* SourceId : HET_SourceId_012 */
/* DesignId : HET_DesignId_012 */
/* Requirements : HL_CONQ_HET_SR22 */
void edgeDisableNotification(hetBASE_t * hetREG, uint32 edge)
{
    hetREG->INTENAC = (uint32)0x20000U << edge;
}


/** @fn void capGetSignal(hetRAMBASE_t * hetRAM, uint32 cap, hetSIGNAL_t signal)
*   @brief Get capture signal
*   @param[in] hetRAM Pointer to HET RAM:
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*   @param[in] cap captured signal:
*              - cap0: Captured signal 0
*              - cap1: Captured signal 1
*              - cap2: Captured signal 2
*              - cap3: Captured signal 3
*              - cap4: Captured signal 4
*              - cap5: Captured signal 5
*              - cap6: Captured signal 6
*              - cap7: Captured signal 7
*   @param[in] signal signal
*              - duty cycle in %.
*              - period period in us.
*
*   Gets current signal of the given capture signal.
*/
/* SourceId : HET_SourceId_013 */
/* DesignId : HET_DesignId_013 */
/* Requirements : HL_CONQ_HET_SR24 */
void capGetSignal(hetRAMBASE_t * hetRAM, uint32 cap, hetSIGNAL_t *signal)
{
    uint32    pwmDuty   = (hetRAM->Instruction[(cap << 1U) + 25U].Data) >> 7U;
    uint32    pwmPeriod = (hetRAM->Instruction[(cap << 1U) + 26U].Data) >> 7U;

    signal->duty   = (pwmDuty * 100U) / pwmPeriod;

    if( hetRAM == hetRAM1)
    {
        signal->period = ((float64)pwmPeriod * 853.333F) / 1000.0F;
    }
    else
    {
        signal->period = ((float64)pwmPeriod * 853.333F) / 1000.0F;
    }
}


/** @fn void hetResetTimestamp(hetRAMBASE_t *hetRAM)
*   @brief Resets timestamp
*   @param[in] hetRAM Pointer to HET RAM:
*              - hetRAM1: HET1 RAM pointer
*              - hetRAM2: HET2 RAM pointer
*
*   Resets loop count based timestamp.
*/
/* SourceId : HET_SourceId_014 */
/* DesignId : HET_DesignId_014 */
/* Requirements : HL_CONQ_HET_SR25 */
void hetResetTimestamp(hetRAMBASE_t * hetRAM)
{
    hetRAM->Instruction[0U].Data = 0U;
}


/** @fn uint32 hetGetTimestamp(hetRAMBASE_t *hetRAM)
*   @brief Returns timestamp
*
*   Returns loop count based timestamp.
*/
/* SourceId : HET_SourceId_015 */
/* DesignId : HET_DesignId_015 */
/* Requirements : HL_CONQ_HET_SR26 */
uint32 hetGetTimestamp(hetRAMBASE_t * hetRAM)
{
    return hetRAM->Instruction[57U].Data;
}

/* USER CODE BEGIN (5) */
/* USER CODE END */


/** @fn void het1GetConfigValue(het_config_reg_t *config_reg, config_value_type_t type)
*   @brief Get the initial or current values of the HET1 configuration registers
*
*   @param[in] *config_reg: pointer to the struct to which the initial or current
*                           value of the configuration registers need to be stored
*   @param[in] type:    whether initial or current value of the configuration registers need to be stored
*                       - InitialValue: initial value of the configuration registers will be stored
*                                       in the struct pointed by config_reg
*                       - CurrentValue: initial value of the configuration registers will be stored
*                                       in the struct pointed by config_reg
*
*   This function will copy the initial or current value (depending on the parameter 'type')
*   of the configuration registers to the struct pointed by config_reg
*
*/
/* SourceId : HET_SourceId_016 */
/* DesignId : HET_DesignId_016 */
/* Requirements : HL_CONQ_HET_SR29 */
void het1GetConfigValue(het_config_reg_t *config_reg, config_value_type_t type)
{
    if (type == InitialValue)
    {
        config_reg->CONFIG_GCR     = HET1_GCR_CONFIGVALUE;
        config_reg->CONFIG_PFR     = HET1_PFR_CONFIGVALUE;
        config_reg->CONFIG_INTENAS = HET1_INTENAS_CONFIGVALUE;
        config_reg->CONFIG_INTENAC = HET1_INTENAC_CONFIGVALUE;
        config_reg->CONFIG_PRY     = HET1_PRY_CONFIGVALUE;
        config_reg->CONFIG_AND     = HET1_AND_CONFIGVALUE;
        config_reg->CONFIG_HRSH    = HET1_HRSH_CONFIGVALUE;
        config_reg->CONFIG_XOR     = HET1_XOR_CONFIGVALUE;
        config_reg->CONFIG_DIR     = HET1_DIR_CONFIGVALUE;
        config_reg->CONFIG_PDR     = HET1_PDR_CONFIGVALUE;
        config_reg->CONFIG_PULDIS  = HET1_PULDIS_CONFIGVALUE;
        config_reg->CONFIG_PSL     = HET1_PSL_CONFIGVALUE;
        config_reg->CONFIG_PCR     = HET1_PCR_CONFIGVALUE;
    }
    else
    {
    /*SAFETYMCUSW 134 S MR:12.2 <APPROVED> "LDRA Tool issue" */
        config_reg->CONFIG_GCR     = hetREG1->GCR;
        config_reg->CONFIG_PFR     = hetREG1->PFR;
        config_reg->CONFIG_INTENAS = hetREG1->INTENAS;
        config_reg->CONFIG_INTENAC = hetREG1->INTENAC;
        config_reg->CONFIG_PRY     = hetREG1->PRY;
        config_reg->CONFIG_AND     = hetREG1->AND;
        config_reg->CONFIG_HRSH    = hetREG1->HRSH;
        config_reg->CONFIG_XOR     = hetREG1->XOR;
        config_reg->CONFIG_DIR     = hetREG1->DIR;
        config_reg->CONFIG_PDR     = hetREG1->PDR;
        config_reg->CONFIG_PULDIS  = hetREG1->PULDIS;
        config_reg->CONFIG_PSL     = hetREG1->PSL;
        config_reg->CONFIG_PCR     = hetREG1->PCR;
    }
}



