// ======================================================================
// \title  Errors.hpp
// \author bocchino, mereweth
// \brief  Test errors
//
// \copyright
// Copyright (c) 2017 California Institute of Technology.
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

#include "Errors.hpp"

namespace Svc {

  namespace Errors {

    // ----------------------------------------------------------------------
    // Tests
    // ----------------------------------------------------------------------


    void Tester ::
      PartialDrain(void)
    {
        ASSERT_EQ(BufferAccumulator::DRAIN, this->component.mode);
        this->sendCmd_BA_DrainBuffers(0, 0, 1, BufferAccumulator::BLOCK);
        this->component.doDispatch(); // will fail - we are still in DRAIN mode
        ASSERT_EQ(BufferAccumulator::DRAIN, this->component.mode);
        ASSERT_FROM_PORT_HISTORY_SIZE(0);
        ASSERT_EVENTS_BA_AlreadyDraining_SIZE(1);
        ASSERT_EQ(0u, this->component.numDrained);
        ASSERT_EQ(0u, this->component.numToDrain);

        this->sendCmd_BA_SetMode(0, 0, BufferAccumulator::ACCUMULATE);
        this->component.doDispatch();
        ASSERT_EQ(BufferAccumulator::ACCUMULATE, this->component.mode);
        ASSERT_FROM_PORT_HISTORY_SIZE(0);

        this->sendCmd_BA_DrainBuffers(0, 0, 10, BufferAccumulator::BLOCK);
        this->component.doDispatch(); // will succeed - now we are in ACCUMULATE
        ASSERT_EQ(BufferAccumulator::ACCUMULATE, this->component.mode);
        ASSERT_FROM_PORT_HISTORY_SIZE(0); // would be first buffer out, but we are empty
        ASSERT_EVENTS_BA_DrainStalled_SIZE(1);
        ASSERT_EVENTS_BA_DrainStalled(0, 0u, 10u);
        ASSERT_EVENTS_BA_PartialDrainDone_SIZE(0); // partial drain not done
        ASSERT_EQ(true, this->component.send);
        ASSERT_EQ(0u, this->component.numDrained);
        ASSERT_EQ(10u, this->component.numToDrain);

        this->sendCmd_BA_DrainBuffers(0, 0, 1, BufferAccumulator::BLOCK);
        this->component.doDispatch(); // will fail - we are still doing a partial drain
        ASSERT_EVENTS_BA_StillDraining_SIZE(1);
        ASSERT_EVENTS_BA_StillDraining(0, 0u, 10u);
        ASSERT_EVENTS_BA_PartialDrainDone_SIZE(0); // partial drain not done
        ASSERT_EQ(BufferAccumulator::ACCUMULATE, this->component.mode);
        ASSERT_FROM_PORT_HISTORY_SIZE(0);
        ASSERT_EQ(true, this->component.send);
        ASSERT_EQ(0u, this->component.numDrained);
        ASSERT_EQ(10u, this->component.numToDrain);
    }

    void Tester ::
      QueueFull(void)
    {

      Fw::Buffer buffer;

      // Go to Accumulate mode
      ASSERT_EQ(BufferAccumulator::DRAIN, this->component.mode);
      this->sendCmd_BA_SetMode(0, 0, BufferAccumulator::ACCUMULATE);
      this->component.doDispatch();
      ASSERT_EQ(BufferAccumulator::ACCUMULATE, this->component.mode);
      ASSERT_FROM_PORT_HISTORY_SIZE(0);

      // Fill up the buffer queue
      for (U32 i = 0; i < MAX_NUM_BUFFERS; ++i) {
        this->invoke_to_bufferSendInFill(0, buffer);
        this->component.doDispatch();
        ASSERT_FROM_PORT_HISTORY_SIZE(0);
      }

      // Send another buffer and expect an event
      this->invoke_to_bufferSendInFill(0, buffer);
      this->component.doDispatch();
      ASSERT_EVENTS_SIZE(1);
      ASSERT_EVENTS_BA_QueueFull_SIZE(1);

      // Send another buffer and expect no new event
      this->invoke_to_bufferSendInFill(0, buffer);
      this->component.doDispatch();
      ASSERT_EVENTS_SIZE(1);

      // Drain one buffer
      this->sendCmd_BA_SetMode(0, 0, BufferAccumulator::DRAIN);
      this->component.doDispatch();
      ASSERT_FROM_PORT_HISTORY_SIZE(1);
      ASSERT_from_bufferSendOutDrain_SIZE(1);
      ASSERT_from_bufferSendOutDrain(0, buffer);

      // Send another buffer and expect an event
      this->invoke_to_bufferSendInFill(0, buffer);
      this->component.doDispatch();
      ASSERT_EVENTS_SIZE(2);
      ASSERT_EVENTS_BA_BufferAccepted_SIZE(1);

      // Return the original buffer in order to drain one buffer
      this->invoke_to_bufferSendInReturn(0, buffer);
      this->component.doDispatch();
      ASSERT_FROM_PORT_HISTORY_SIZE(3);
      ASSERT_from_bufferSendOutDrain_SIZE(2);
      ASSERT_from_bufferSendOutDrain(1, buffer);

      // Send another buffer and expect no new event
      this->invoke_to_bufferSendInFill(0, buffer);
      this->component.doDispatch();
      ASSERT_EVENTS_SIZE(2);
      ASSERT_EVENTS_BA_BufferAccepted_SIZE(1);

      // Send another buffer and expect an event
      this->invoke_to_bufferSendInFill(0, buffer);
      this->component.doDispatch();
      ASSERT_EVENTS_SIZE(3);
      ASSERT_EVENTS_BA_QueueFull_SIZE(2);

    }

  }

}