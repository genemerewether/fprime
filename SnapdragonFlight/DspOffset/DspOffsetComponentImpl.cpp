// ======================================================================
// \title  DspOffsetComponentImpl.cpp
// \author mereweth
// \brief  cpp file for DspOffset component implementation class
//
// \copyright
// Copyright 2009-2015, by the California Institute of Technology.
// ALL RIGHTS RESERVED.  United States Government Sponsorship
// acknowledged.
//
// ======================================================================


#include <SnapdragonFlight/DspOffset/DspOffsetComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <Os/File.hpp>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

namespace SnapdragonFlight {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  DspOffsetComponentImpl ::
#if FW_OBJECT_NAMES == 1
    DspOffsetComponentImpl(
        const char *const compName
    ) :
      DspOffsetComponentBase(compName)
#else
    DspOffsetComponentImpl(void)
#endif
  {

  }

  void DspOffsetComponentImpl ::
    init(
        const NATIVE_INT_TYPE queueDepth,
        const NATIVE_INT_TYPE instance
    )
  {
    DspOffsetComponentBase::init(queueDepth, instance);
  }

  DspOffsetComponentImpl ::
    ~DspOffsetComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void DspOffsetComponentImpl ::
    SchedIn_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
      Fw::Time hlosTime = this->getTime();
      const I64 usecHLOS = (I64) hlosTime.getSeconds() * 1000LL * 1000LL + (I64) hlosTime.getUSeconds();
        
      Os::File::Status stat = Os::File::OTHER_ERROR;
      Os::File file;
      stat = file.open("/sys/kernel/dsp_offset/walltime_dsp_diff", Os::File::OPEN_READ);
      if (stat != Os::File::OP_OK) {
          // TODO(mereweth) - EVR
          printf("Unable to read DSP diff at /sys/kernel/dsp_offset/walltime_dsp_diff\n");
          return;
      }
      char buff[255];
      NATIVE_INT_TYPE size = sizeof(buff);
      stat = file.read(buff, size, false);
      file.close();
      if ((stat != Os::File::OP_OK) ||
          !size) {
          // TODO(mereweth) - EVR
          printf("Unable to read DSP diff at /sys/kernel/dsp_offset/walltime_dsp_diff\n");
          return;
      }
      // Make sure buffer is null-terminated:
      buff[sizeof(buff)-1] = 0;
      const I64 walltimeDspLeadUs = strtoll(buff, NULL, 10);

      if (walltimeDspLeadUs > usecHLOS) {
          // TODO(mereweth) - EVR; can't have difference greater than time
          printf("linux-dsp diff %lld greater than message time %lu\n",
                 walltimeDspLeadUs, usecHLOS);
          return;
      }
      const I64 usecDsp = usecHLOS - walltimeDspLeadUs;
      Fw::Time dspTime(TB_PROC_TIME,
                       0,
                       (U32) (usecDsp / 1000 / 1000),
                       (U32) (usecDsp % (1000 * 1000)));

      this->ClockTimes_out(0, hlosTime, dspTime);
      this->tlmWrite_DSPOFF_HLOSLeadDsp((F64) walltimeDspLeadUs * 0.001 * 0.001);
  }

} // end namespace SnapdragonFlight
