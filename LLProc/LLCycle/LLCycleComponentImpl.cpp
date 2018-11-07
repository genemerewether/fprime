// ====================================================================== 
// \title  LLCycleImpl.cpp
// \author tcanham
// \brief  cpp file for LLCycle component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged. Any commercial use must be negotiated with the Office
// of Technology Transfer at the California Institute of Technology.
// 
// This software may be subject to U.S. export control laws and
// regulations.  By accepting this document, the user agrees to comply
// with all U.S. export laws and regulations.  User has the
// responsibility to obtain export licenses, or other export authority
// as may be required before exporting such information to foreign
// countries or providing access to foreign persons.
// ====================================================================== 


#include <LLProc/LLCycle/LLCycleComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <LLProc/LLCycle/LLCycleComponentImplCfg.hpp>
#include <stdio.h>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)
#define DEBUG_PRINT(x,...)

#define TIME_DEBUG 0

namespace LLProc {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  LLCycleComponentImpl ::
#if FW_OBJECT_NAMES == 1
    LLCycleComponentImpl(
        const char *const compName
    ) : LLCycleComponentBase(compName)
#else
    LLCycleImpl(void)
#endif
    ,m_maxRti(0)
    ,m_minRti(4000) // two cycles
    ,m_overRuns(0)
    ,m_rtiNum(0)
    ,m_firstRti(true)
    ,m_timeSyncStage(TIME_SYNC_DONE) //TODO - implement platform-specific timesync here
  {

  }

  void LLCycleComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    LLCycleComponentBase::init(instance);
  }

  LLCycleComponentImpl ::
    ~LLCycleComponentImpl(void)
  {

  }

  void LLCycleComponentImpl ::
    runCycles(NATIVE_UINT_TYPE cycles) {

      for (NATIVE_UINT_TYPE cycle = 0; cycle < cycles; cycle++) {
          // Wait for RTI
          DEBUG_PRINT("Wait for RTI %d\n",this->m_rtiNum);
          bool alreadySet;
          if (this->isConnected_RtiWait_OutputPort(0)) {
              this->RtiWait_out(0, alreadySet);
          }
          if (this->m_firstRti) {
              this->m_firstRti = false;
              if (this->isConnected_RtiWait_OutputPort(0)) {
                  this->RtiWait_out(0, alreadySet);
              }
              DEBUG_PRINT("Received first RTI %d\n",this->m_rtiNum);
              this->m_rtiNum++;
          } else {
              DEBUG_PRINT("Received RTI %d\n",this->m_rtiNum);
          }

          // check the RTI time to see if we exceeded the max
          if (TIME_SYNC_DONE == this->m_timeSyncStage) {
              this->m_rtiEndTimer.take();
              U32 diff = this->m_rtiEndTimer.diffUSec(m_rtiStartTimer);
              DEBUG_PRINT("RTI diff time: %d\n",diff);
              if (diff > this->m_maxRti) {
                  this->m_maxRti = diff;
                  //this->TlmOut_out(0,LL_TLM_MAX_RTI,diff);
              }
              if (diff < this->m_minRti) {
                  this->m_minRti = diff;
                  //this->TlmOut_out(0,LL_TLM_MIN_RTI,diff);
              }
              // time synced
              //this->TlmOut_out(0,LL_TLM_TIME_SYNC,0xFF);
          } else {
              // time is not yet synced
              //this->TlmOut_out(0,LL_TLM_TIME_SYNC,0);
          }
          this->m_rtiStartTimer.take();
          // execute time sync if necessary
          switch (this->m_timeSyncStage) {
              case TIME_SYNC_START: {
                  //TODO - implement platform-specific timesync here
                  break;
              }
              case TIME_SYNC_WAIT_FOR_SYNCH_RTI:
                  // start time running again with new values
                  this->TimeStartPort_out(0, Fw::START_CLOCK);
                  // go to complete stage
                  this->m_timeSyncStage = TIME_SYNC_DONE;
                  break;
              case TIME_SYNC_DONE:
                  // done, do nothing
                  break;
              default:
                  FW_ASSERT(0,this->m_timeSyncStage);
                  break;
          }
          // cycle rate group

          // grab interval timer time
          this->m_rgTimer.take();
          // check for overrun
          if (alreadySet) {
              //this->TlmOut_out(0,LL_TLM_CYCLE_SLIPS,++this->m_overRuns);
          }

          this->RateGroupOut_out(0,this->m_rgTimer);
          this->m_rtiNum++;

          DEBUG_PRINT("Rate Group Done\n");
      } // end for each cycle
  }

} // end namespace LLProc
