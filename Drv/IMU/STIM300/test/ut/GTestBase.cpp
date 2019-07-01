// ======================================================================
// \title  STIM300/test/ut/GTestBase.cpp
// \author Auto-generated
// \brief  cpp file for STIM300 component Google Test harness base class
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

#include "GTestBase.hpp"

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction and destruction
  // ----------------------------------------------------------------------

  STIM300GTestBase ::
    STIM300GTestBase(
#if FW_OBJECT_NAMES == 1
        const char *const compName,
        const U32 maxHistorySize
#else
        const U32 maxHistorySize
#endif
    ) : 
        STIM300TesterBase (
#if FW_OBJECT_NAMES == 1
            compName,
#endif
            maxHistorySize
        )
  {

  }

  STIM300GTestBase ::
    ~STIM300GTestBase(void)
  {

  }

  // ----------------------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertTlm_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->tlmSize)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Total size of all telemetry histories\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmSize << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: NumPackets
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertTlm_NumPackets_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_NumPackets->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel NumPackets\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_NumPackets->size() << "\n";
  }

  void STIM300GTestBase ::
    assertTlm_NumPackets(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_NumPackets->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel NumPackets\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_NumPackets->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_NumPackets& e =
      this->tlmHistory_NumPackets->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel NumPackets\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: ImuPacket
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertTlm_ImuPacket_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_ImuPacket->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel ImuPacket\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_ImuPacket->size() << "\n";
  }

  void STIM300GTestBase ::
    assertTlm_ImuPacket(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const ROS::sensor_msgs::ImuNoCov& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_ImuPacket->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel ImuPacket\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_ImuPacket->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_ImuPacket& e =
      this->tlmHistory_ImuPacket->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel ImuPacket\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Channel: TimeSyncStatus
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertTlm_TimeSyncStatus_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(this->tlmHistory_TimeSyncStatus->size(), size)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for telemetry channel TimeSyncStatus\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->tlmHistory_TimeSyncStatus->size() << "\n";
  }

  void STIM300GTestBase ::
    assertTlm_TimeSyncStatus(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const STIM300ComponentBase::STIM300TimeSync& val
    )
    const
  {
    ASSERT_LT(__index, this->tlmHistory_TimeSyncStatus->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of telemetry channel TimeSyncStatus\n"
      << "  Expected: Less than size of history (" 
      << this->tlmHistory_TimeSyncStatus->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const TlmEntry_TimeSyncStatus& e =
      this->tlmHistory_TimeSyncStatus->at(__index);
    ASSERT_EQ(val, e.arg)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value at index "
      << __index
      << " on telmetry channel TimeSyncStatus\n"
      << "  Expected: " << val << "\n"
      << "  Actual:   " << e.arg << "\n";
  }

  // ----------------------------------------------------------------------
  // Events
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertEvents_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Total size of all event histories\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: BufferFull
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertEvents_BufferFull_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_BufferFull)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event BufferFull\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_BufferFull << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: UartError
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertEvents_UartError_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_UartError)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event UartError\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_UartError << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: NoEvents
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertEvents_NoEvents_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_NoEvents)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event NoEvents\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_NoEvents << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: InvalidCounter
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertEvents_InvalidCounter_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_InvalidCounter->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event InvalidCounter\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_InvalidCounter->size() << "\n";
  }

  void STIM300GTestBase ::
    assertEvents_InvalidCounter(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 actualCount,
        const U32 expectedCount
    ) const
  {
    ASSERT_GT(this->eventHistory_InvalidCounter->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event InvalidCounter\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_InvalidCounter->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_InvalidCounter& e =
      this->eventHistory_InvalidCounter->at(__index);
    ASSERT_EQ(actualCount, e.actualCount)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument actualCount at index "
      << __index
      << " in history of event InvalidCounter\n"
      << "  Expected: " << actualCount << "\n"
      << "  Actual:   " << e.actualCount << "\n";
    ASSERT_EQ(expectedCount, e.expectedCount)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument expectedCount at index "
      << __index
      << " in history of event InvalidCounter\n"
      << "  Expected: " << expectedCount << "\n"
      << "  Actual:   " << e.expectedCount << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: TooManyEvents
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertEvents_TooManyEvents_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventHistory_TooManyEvents->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event TooManyEvents\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventHistory_TooManyEvents->size() << "\n";
  }

  void STIM300GTestBase ::
    assertEvents_TooManyEvents(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 __index,
        const U32 maxEvents
    ) const
  {
    ASSERT_GT(this->eventHistory_TooManyEvents->size(), __index)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Index into history of event TooManyEvents\n"
      << "  Expected: Less than size of history (" 
      << this->eventHistory_TooManyEvents->size() << ")\n"
      << "  Actual:   " << __index << "\n";
    const EventEntry_TooManyEvents& e =
      this->eventHistory_TooManyEvents->at(__index);
    ASSERT_EQ(maxEvents, e.maxEvents)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Value of argument maxEvents at index "
      << __index
      << " in history of event TooManyEvents\n"
      << "  Expected: " << maxEvents << "\n"
      << "  Actual:   " << e.maxEvents << "\n";
  }

  // ----------------------------------------------------------------------
  // Event: BadTimeSync
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertEvents_BadTimeSync_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->eventsSize_BadTimeSync)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for event BadTimeSync\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->eventsSize_BadTimeSync << "\n";
  }

  // ----------------------------------------------------------------------
  // From ports
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assertFromPortHistory_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistorySize)
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Total size of all from port histories\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistorySize << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: IMU
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assert_from_IMU_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_IMU->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_IMU\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_IMU->size() << "\n";
  }

  // ----------------------------------------------------------------------
  // From port: packetTime
  // ----------------------------------------------------------------------

  void STIM300GTestBase ::
    assert_from_packetTime_size(
        const char *const __callSiteFileName,
        const U32 __callSiteLineNumber,
        const U32 size
    ) const
  {
    ASSERT_EQ(size, this->fromPortHistory_packetTime->size())
      << "\n"
      << "  File:     " << __callSiteFileName << "\n"
      << "  Line:     " << __callSiteLineNumber << "\n"
      << "  Value:    Size of history for from_packetTime\n"
      << "  Expected: " << size << "\n"
      << "  Actual:   " << this->fromPortHistory_packetTime->size() << "\n";
  }

} // end namespace Drv
