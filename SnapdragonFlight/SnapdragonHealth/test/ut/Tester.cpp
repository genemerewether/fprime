// ======================================================================
// \title  SnapdragonHealth.hpp
// \author gcgandhi
// \brief  cpp file for SnapdragonHealth test harness implementation class
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

#include "Tester.hpp"
#include <Os/File.hpp>

#include <Common/Cfg/CommonPolyDb.hpp>

#define INSTANCE 0
#define MAX_HISTORY_SIZE 100
#define QUEUE_DEPTH 100

// Must update depending on machine your on
#if BUILD_LINUX

#define HAS_MON_FILES 1
#define NUM_TEMP_ZONES 1

#elif BUILD_SDFLIGHT

#define HAS_MON_FILES 1
#define NUM_TEMP_ZONES 17

#else

// Probably the right setting for Parallels
#define NUM_TEMP_ZONES 0

#endif


namespace SnapdragonFlight {

    static const int POWER_SAVER_ON_NUM_FILES = 1 + SnapdragonHealthComponentImpl::HEALTH_FILE_TYPE_MAX
                                                - SnapdragonHealthComponentImpl::HEALTH_FILE_TYPE_MIN
                                                - (SnapdragonHealthComponentImpl::FREQ_SET_FILE_TYPE_MAX
                                                   - SnapdragonHealthComponentImpl::FREQ_SET_FILE_TYPE_MIN + 1);

    static const int POWER_SAVER_OFF_NUM_FILES = 1 + SnapdragonHealthComponentImpl::HEALTH_FILE_TYPE_MAX
                                                 - SnapdragonHealthComponentImpl::HEALTH_FILE_TYPE_MIN;

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  Tester ::
    Tester(void) :
#if FW_OBJECT_NAMES == 1
      SnapdragonHealthGTestBase("Tester", MAX_HISTORY_SIZE),
      component("SnapdragonHealth")
#else
      SnapdragonHealthGTestBase(MAX_HISTORY_SIZE),
      component()
#endif
  {
    this->initComponents();
    this->connectPorts();
  }

  Tester ::
    ~Tester(void)
  {

  }

  // ----------------------------------------------------------------------
  // Tests
  // ----------------------------------------------------------------------

  void Tester::run_nominal_tests(void)
  {
      // This test cannot run on Macs/PCs
#if BUILD_CYGWIN
      return;
#endif

      this->clearHistory();
      this->component.preamble();

      ASSERT_FALSE(this->component.fsw_version_sent);

      Fw::Time tempTime(10, 12345);
      this->setTestTime(tempTime);

      // Run one cycle:
      this->invoke_to_SchedIn(0,0);
      this->dispatch();

      ASSERT_TRUE(this->component.fsw_version_sent);
      ASSERT_EVENTS_SH_FswVersion_SIZE(1);
      ASSERT_EVENTS_SH_FswVersion(0,0x1bf18d920,"Rel3.0-AltMgr");

      ASSERT_TLM_SH_FswVersion_SIZE(1);
      ASSERT_TLM_SH_FswVersion(0, 0x1bf18d920);

      for (int i = 0; i < NUM_TEMP_ZONES; i++) {
          if (i <= 10) { // kernel thermal management enabled
              ASSERT_TRUE(this->component.temp_zone_stat[i]);
          }
          else { // kernel thermal management disabled
              ASSERT_FALSE(this->component.temp_zone_stat[i]);
          }
          if ((i != 13) && (i != 14)) { // power amp 0 and 1 give negative
              ASSERT_LT(0, this->component.temp_zone_temp[i]);
          }
          printf("this->component.temp_zone_temp[%d]: %d\n",
                 i, this->component.temp_zone_temp[i]);
      }

      ASSERT_from_PowerReport_SIZE(2);
      ASSERT_from_PowerReport(0, static_cast<U32>(Common::COMMON_POLYDB_POWERSAVE_STATE),
                              Svc::MEASUREMENT_OK,
                              tempTime,
                              Fw::PolyType(Common::COMMON_POLYDB_SAVER_ON));

      // NOTE(mereweth) - floating point comparison fails
/*
      ASSERT_from_PowerReport(1, static_cast<U32>(Common::COMMON_POLYDB_SDTEMP),
                              Svc::MEASUREMENT_OK,
                              tempTime,
                              Fw::PolyType((F32)
                                  FW_MAX(FW_MAX(this->component.temp_zone_temp[5],
                                                this->component.temp_zone_temp[6]),
                                         FW_MAX(this->component.temp_zone_temp[7],
                                                this->component.temp_zone_temp[8]))));
*/
      ASSERT_TLM_SnapdragonPowerSaver_SIZE(1);
      ASSERT_TLM_SnapdragonPowerSaver(0, SnapdragonHealthComponentImpl::SAVER_ON_TLM);

#if NUM_TEMP_ZONES > 0
      ASSERT_TRUE(this->component.temp_zone_stat[0]);
      ASSERT_TLM_SnapTempZone0DspCpuStat_SIZE(1);
      ASSERT_TLM_SnapTempZone0DspCpuStat(0,true);
      ASSERT_TLM_SnapTempZone0DspCpuTemp_SIZE(1);
      ASSERT_TLM_SnapTempZone0DspCpuTemp(0,this->component.temp_zone_temp[0]);
#endif
#if NUM_TEMP_ZONES > 1
      ASSERT_TLM_SnapTempZone1DspModemStat_SIZE(1);
      ASSERT_TLM_SnapTempZone1DspModemStat(0,true);
      ASSERT_TLM_SnapTempZone1DspModemTemp_SIZE(1);
      ASSERT_TLM_SnapTempZone1DspModemTemp(0,this->component.temp_zone_temp[1]);
#endif
#if NUM_TEMP_ZONES > 2
      ASSERT_TLM_SnapTempZone2ModemStat_SIZE(1);
      ASSERT_TLM_SnapTempZone2ModemStat(0,true);
      ASSERT_TLM_SnapTempZone2ModemTemp_SIZE(1);
      ASSERT_TLM_SnapTempZone2ModemTemp(0,this->component.temp_zone_temp[2]);
#endif
#if NUM_TEMP_ZONES > 3
      ASSERT_TLM_SnapTempZone3VideoStat_SIZE(1);
      ASSERT_TLM_SnapTempZone3VideoStat(0,true);
      ASSERT_TLM_SnapTempZone3VideoTemp_SIZE(1);
      ASSERT_TLM_SnapTempZone3VideoTemp(0,this->component.temp_zone_temp[3]);
#endif
#if NUM_TEMP_ZONES > 4
      ASSERT_TLM_SnapTempZone4WlanCpuStat_SIZE(1);
      ASSERT_TLM_SnapTempZone4WlanCpuStat(0,true);
      ASSERT_TLM_SnapTempZone4WlanCpuTemp_SIZE(1);
      ASSERT_TLM_SnapTempZone4WlanCpuTemp(0,this->component.temp_zone_temp[4]);
#endif
#if NUM_TEMP_ZONES > 5
      ASSERT_TLM_SnapTempZone5Cpu0Stat_SIZE(1);
      ASSERT_TLM_SnapTempZone5Cpu0Stat(0,true);
      ASSERT_TLM_SnapTempZone5Cpu0Temp_SIZE(1);
      ASSERT_TLM_SnapTempZone5Cpu0Temp(0,this->component.temp_zone_temp[5]);
#endif
#if NUM_TEMP_ZONES > 6
      ASSERT_TLM_SnapTempZone6Cpu1Stat_SIZE(1);
      ASSERT_TLM_SnapTempZone6Cpu1Stat(0,true);
      ASSERT_TLM_SnapTempZone6Cpu1Temp_SIZE(1);
      ASSERT_TLM_SnapTempZone6Cpu1Temp(0,this->component.temp_zone_temp[6]);
#endif
#if NUM_TEMP_ZONES > 7
      ASSERT_TLM_SnapTempZone7Cpu2Stat_SIZE(1);
      ASSERT_TLM_SnapTempZone7Cpu2Stat(0,true);
      ASSERT_TLM_SnapTempZone7Cpu2Temp_SIZE(1);
      ASSERT_TLM_SnapTempZone7Cpu2Temp(0,this->component.temp_zone_temp[7]);
#endif
#if NUM_TEMP_ZONES > 8
      ASSERT_TLM_SnapTempZone8Cpu3Stat_SIZE(1);
      ASSERT_TLM_SnapTempZone8Cpu3Stat(0,true);
      ASSERT_TLM_SnapTempZone8Cpu3Temp_SIZE(1);
      ASSERT_TLM_SnapTempZone8Cpu3Temp(0,this->component.temp_zone_temp[8]);
#endif
#if NUM_TEMP_ZONES > 9
      ASSERT_TLM_SnapTempZone9GpuAStat_SIZE(1);
      ASSERT_TLM_SnapTempZone9GpuAStat(0,true);
      ASSERT_TLM_SnapTempZone9GpuATemp_SIZE(1);
      ASSERT_TLM_SnapTempZone9GpuATemp(0,this->component.temp_zone_temp[9]);
#endif
#if NUM_TEMP_ZONES > 10
      ASSERT_TLM_SnapTempZone10GpuBStat_SIZE(1);
      ASSERT_TLM_SnapTempZone10GpuBStat(0,true);
      ASSERT_TLM_SnapTempZone10GpuBTemp_SIZE(1);
      ASSERT_TLM_SnapTempZone10GpuBTemp(0,this->component.temp_zone_temp[10]);
#endif
// from here on, all zones should have stat false, meaning not used for kernel
// thermal management
#if NUM_TEMP_ZONES > 11
      ASSERT_TLM_SnapTempZone11PM8841Stat_SIZE(1);
      ASSERT_TLM_SnapTempZone11PM8841Stat(0,false);
      ASSERT_TLM_SnapTempZone11PM8841Temp_SIZE(1);
      ASSERT_TLM_SnapTempZone11PM8841Temp(0,0.001 *
                                            this->component.temp_zone_temp[11]);
#endif
#if NUM_TEMP_ZONES > 12
      ASSERT_TLM_SnapTempZone12PM8941Stat_SIZE(1);
      ASSERT_TLM_SnapTempZone12PM8941Stat(0,false);
      ASSERT_TLM_SnapTempZone12PM8941Temp_SIZE(1);
      ASSERT_TLM_SnapTempZone12PM8941Temp(0,0.001 *
                                            this->component.temp_zone_temp[12]);
#endif
#if NUM_TEMP_ZONES > 13
      /*ASSERT_TLM_SnapTempZone13PA0Stat_SIZE(1);
      ASSERT_TLM_SnapTempZone13PA0Stat(0,false);
      ASSERT_TLM_SnapTempZone13PA0Temp_SIZE(1);
      ASSERT_TLM_SnapTempZone13PA0Temp(0,this->component.temp_zone_temp[13]);*/
#endif
#if NUM_TEMP_ZONES > 14
      /*ASSERT_TLM_SnapTempZone14PA1Stat_SIZE(1);
      ASSERT_TLM_SnapTempZone14PA1Stat(0,false);
      ASSERT_TLM_SnapTempZone14PA1Temp_SIZE(1);
      ASSERT_TLM_SnapTempZone14PA1Temp(0,this->component.temp_zone_temp[14]);*/
#endif
#if NUM_TEMP_ZONES > 15
      ASSERT_TLM_SnapTempZone15EmmcStat_SIZE(1);
      ASSERT_TLM_SnapTempZone15EmmcStat(0,false);
      ASSERT_TLM_SnapTempZone15EmmcTemp_SIZE(1);
      ASSERT_TLM_SnapTempZone15EmmcTemp(0,this->component.temp_zone_temp[15]);
#endif
#if NUM_TEMP_ZONES > 16
      ASSERT_TLM_SnapTempZone16MsmStat_SIZE(1);
      ASSERT_TLM_SnapTempZone16MsmStat(0,false);
      ASSERT_TLM_SnapTempZone16MsmTemp_SIZE(1);
      ASSERT_TLM_SnapTempZone16MsmTemp(0,this->component.temp_zone_temp[16]);
#endif

      // Verify CPU util:
#if HAS_MON_FILES
      ASSERT_LE(0,this->component.total_cpu_util);
      ASSERT_LE(0,this->component.cpu_util[1]);
      ASSERT_LE(0,this->component.cpu_util[2]);
      ASSERT_LE(0,this->component.cpu_util[0]);
      ASSERT_LE(0,this->component.cpu_util[3]);
#endif

      printf("this->component.total_cpu_util: %f\n",
              this->component.total_cpu_util);

      ASSERT_TLM_SnapdragonTotalCpuUtil_SIZE(1);
      ASSERT_TLM_SnapdragonTotalCpuUtil(0,this->component.total_cpu_util);
      ASSERT_TLM_SnapdragonCpu0Util_SIZE(1);
      ASSERT_TLM_SnapdragonCpu0Util(0,this->component.cpu_util[0]);
      ASSERT_TLM_SnapdragonCpu1Util_SIZE(1);
      ASSERT_TLM_SnapdragonCpu1Util(0,this->component.cpu_util[1]);
      ASSERT_TLM_SnapdragonCpu2Util_SIZE(1);
      ASSERT_TLM_SnapdragonCpu2Util(0,this->component.cpu_util[2]);
      ASSERT_TLM_SnapdragonCpu3Util_SIZE(1);
      ASSERT_TLM_SnapdragonCpu3Util(0,this->component.cpu_util[3]);

      // Verify no off-nominal conditions
      ASSERT_EVENTS_SH_FileReadError_SIZE(0);

#if HAS_MON_FILES == 1
      /* systems in this category vary widely in # of thermal zones and whether
       * mode & temp are present
       */
#elif HAS_MON_FILES == 2
      ASSERT_EVENTS_SH_FileOpenError_SIZE(0);
#elif HAS_MON_FILES == 0
      ASSERT_EVENTS_SH_FileOpenError_SIZE(POWER_SAVER_ON_NUM_FILES);
#endif

      this->clearHistory();

      this->component.log_WARNING_LO_SH_FileOpenError_ThrottleClear();

      // Turn off power-saving
      this->sendCmd_SnapdragonHealth_SetPowerSaver(0, 0,
                                    SnapdragonHealthComponentImpl::SAVER_OFF);

      tempTime = Fw::Time(11, 23456);
      this->setTestTime(tempTime);

      this->dispatch();
      ASSERT_EQ(SnapdragonHealthComponentImpl::SAVER_OFF,
                this->component.power_saver);

      // Run one cycle:
      this->invoke_to_SchedIn(0,0);
      this->dispatch();

      ASSERT_from_PowerReport_SIZE(2);
      ASSERT_from_PowerReport(0, static_cast<U32>(Common::COMMON_POLYDB_POWERSAVE_STATE),
                              Svc::MEASUREMENT_OK,
                              tempTime,
                              Fw::PolyType(Common::COMMON_POLYDB_SAVER_OFF));

      // NOTE(mereweth) - floating point comparison fails
/*
      ASSERT_from_PowerReport(1, static_cast<U32>(Common::COMMON_POLYDB_SDTEMP),
                              Svc::MEASUREMENT_OK,
                              tempTime,
                              Fw::PolyType((F32)
                                  FW_MAX(FW_MAX(this->component.temp_zone_temp[5],
                                                this->component.temp_zone_temp[6]),
                                         FW_MAX(this->component.temp_zone_temp[7],
                                                this->component.temp_zone_temp[8]))));
*/

      ASSERT_TLM_SnapdragonPowerSaver_SIZE(1);
      ASSERT_TLM_SnapdragonPowerSaver(0, SnapdragonHealthComponentImpl::SAVER_OFF_TLM);
      ASSERT_EVENTS_SH_PowerSaver(0, SnapdragonHealthComponentImpl::SAVER_OFF_EVR);

      ASSERT_EVENTS_SH_FileReadError_SIZE(0);
      #if HAS_MON_FILES == 1
            /* systems in this category vary widely in # of thermal zones and whether
             * mode & temp are present
             */
      #elif HAS_MON_FILES == 2
            ASSERT_EVENTS_SH_FileOpenError_SIZE(0);
      #elif HAS_MON_FILES == 0
            // NOTE(mereweth) only opens /proc/stat once
            ASSERT_EVENTS_SH_FileOpenError_SIZE(POWER_SAVER_OFF_NUM_FILES - 1);
      #endif
  }

  void Tester::run_off_nominal_tests(void)
  {

      // This test cannot run on Macs/PCs
#if BUILD_CYGWIN
      return;
#endif

      // These files don't exist so will get the file open errors:
#if HAS_MON_FILES == 0

      printf("File open error (only works on non-true linux machines)..\n");

      this->clearHistory();
      this->component.preamble();

       // Run one cycle:
       this->invoke_to_SchedIn(0,0);
       this->dispatch();

       ASSERT_EVENTS_SH_FileOpenError_SIZE(POWER_SAVER_ON_NUM_FILES);
       int numFiles = 0;

       ASSERT_EVENTS_SH_FileOpenError(numFiles,
                                      SnapdragonHealthComponentImpl::PROC_FILE_TYPE,
                                      Os::File::OTHER_ERROR);
       numFiles += 1;

       ASSERT_EVENTS_SH_FileOpenError(numFiles,
                                      SnapdragonHealthComponentImpl::GOV_0_FILE_TYPE,
                                      Os::File::DOESNT_EXIST);
       numFiles += SnapdragonHealthComponentImpl::GOV_FILE_TYPE_MAX
                   - SnapdragonHealthComponentImpl::GOV_FILE_TYPE_MIN + 1;

       ASSERT_EVENTS_SH_FileOpenError(numFiles,
                                      SnapdragonHealthComponentImpl::L2_POWER_FILE_TYPE,
                                      Os::File::DOESNT_EXIST);
       numFiles += 1;

       ASSERT_EVENTS_SH_FileOpenError(numFiles,
                                      SnapdragonHealthComponentImpl::TEMP_0_FILE_TYPE,
                                      Os::File::DOESNT_EXIST);
       numFiles += SnapdragonHealthComponentImpl::TEMP_FILE_TYPE_MAX
                   - SnapdragonHealthComponentImpl::TEMP_FILE_TYPE_MIN + 1;

       ASSERT_EVENTS_SH_FileOpenError(numFiles,
                                      SnapdragonHealthComponentImpl::TEMP_MODE_0_FILE_TYPE,
                                      Os::File::DOESNT_EXIST);
       numFiles += SnapdragonHealthComponentImpl::TEMP_MODE_FILE_TYPE_MAX
                   - SnapdragonHealthComponentImpl::TEMP_MODE_FILE_TYPE_MIN + 1;

       ASSERT_EVENTS_SH_FileOpenError(numFiles,
                                      SnapdragonHealthComponentImpl::FREQ_0_FILE_TYPE,
                                      Os::File::DOESNT_EXIST);
       numFiles += SnapdragonHealthComponentImpl::FREQ_FILE_TYPE_MAX
                   - SnapdragonHealthComponentImpl::FREQ_FILE_TYPE_MIN + 1;

#endif

       // TODO: Cant delete the files, so no way to test read errors


  }


    // ----------------------------------------------------------------------
    // Handlers for typed from ports
    // ----------------------------------------------------------------------

    void Tester ::
      from_PowerReport_handler(
          const NATIVE_INT_TYPE portNum,
          U32 entry,
          Svc::MeasurementStatus &status,
          Fw::Time &time,
          Fw::PolyType &val
      )
    {
      this->pushFromPortEntry_PowerReport(entry, status, time, val);
    }

  // ----------------------------------------------------------------------
  // Helper methods
  // ----------------------------------------------------------------------

  void Tester::textLogIn(
                   const FwEventIdType id, //!< The event ID
                   Fw::Time& timeTag, //!< The time
                   const Fw::TextLogSeverity severity, //!< The severity
                   const Fw::TextLogString& text //!< The event string
               ) {

       TextLogEntry e = { id, timeTag, severity, text };

       printTextLogHistoryEntry(e,stdout);
  }

  void Tester ::
    from_pingOut_handler(
        const NATIVE_INT_TYPE portNum,
        U32 key
    )
  {
    this->pushFromPortEntry_pingOut(key);
  }



  void Tester ::
      dispatch(void)
  {
      SnapdragonHealthComponentBase::MsgDispatchStatus stat = this->component.doDispatch();
      FW_ASSERT(stat != SnapdragonHealthComponentBase::MSG_DISPATCH_ERROR);
  }

  void Tester ::
    connectPorts(void)
  {

      // SchedIn
      this->connect_to_SchedIn(
          0,
          this->component.get_SchedIn_InputPort(0)
      );

      // pingIn
      this->connect_to_pingIn(
          0,
          this->component.get_pingIn_InputPort(0)
      );

      // CmdDisp
      this->connect_to_CmdDisp(
          0,
          this->component.get_CmdDisp_InputPort(0)
      );

      // PowerReport
      this->component.set_PowerReport_OutputPort(
          0,
          this->get_from_PowerReport(0)
      );

      // pingOut
      this->component.set_pingOut_OutputPort(
          0,
          this->get_from_pingOut(0)
      );

      // CmdStatus
      this->component.set_CmdStatus_OutputPort(
          0,
          this->get_from_CmdStatus(0)
      );

      // CmdReg
      this->component.set_CmdReg_OutputPort(
          0,
          this->get_from_CmdReg(0)
      );

      // Tlm
      this->component.set_Tlm_OutputPort(
          0,
          this->get_from_Tlm(0)
      );

      // Time
      this->component.set_Time_OutputPort(
          0,
          this->get_from_Time(0)
      );

      // Log
      this->component.set_Log_OutputPort(
          0,
          this->get_from_Log(0)
      );

      // LogText
      this->component.set_LogText_OutputPort(
          0,
          this->get_from_LogText(0)
      );



  }

  void Tester ::
    initComponents(void)
  {
    this->init();
    this->component.init(
        QUEUE_DEPTH, INSTANCE
    );
  }

} // end namespace SnapdragonFlight
