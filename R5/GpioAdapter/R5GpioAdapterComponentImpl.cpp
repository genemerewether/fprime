// ======================================================================
// \title  R5GpioAdapterImpl.cpp
// \author mereweth
// \brief  cpp file for R5GpioAdapter component implementation class
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


#include <R5/GpioAdapter/R5GpioAdapterComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  R5GpioAdapterComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5GpioAdapterComponentImpl(
        const char *const compName
    ) :
      R5GpioAdapterComponentBase(compName)
#else
    R5GpioAdapterImpl(void)
#endif
  {

  }

  void R5GpioAdapterComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    R5GpioAdapterComponentBase::init(instance);
  }

  R5GpioAdapterComponentImpl ::
    ~R5GpioAdapterComponentImpl(void)
  {

  }

  void R5GpioAdapterComponentImpl ::
    waitMapping(GpioWaitBank bank, U32 bit) {
      this->m_waitBank = bank;
      this->m_waitBit = bit;
      this->m_waitValid = true;
  }

  void R5GpioAdapterComponentImpl ::
    getMapping(GpioGetBank bank, U32 bit) {
      this->m_getBank = bank;
      this->m_getBit = bit;
      this->m_getValid = true;
  }

  void R5GpioAdapterComponentImpl ::
    setMapping(GpioSetBank bank, U32 bit) {
      this->m_setBank = bank;
      this->m_setBit = bit;
      this->m_setValid = true;
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5GpioAdapterComponentImpl ::
    setIn_handler(
        const NATIVE_INT_TYPE portNum,
        bool state
    )
  {
      if (m_setValid) {
          this->setOut_out(0, m_setBank, m_setBit,
                           state ? GPIO_SET_SET : GPIO_SET_CLEAR);
      }
  }

  void R5GpioAdapterComponentImpl ::
    getIn_handler(
        const NATIVE_INT_TYPE portNum,
        bool &state
    )
  {
      if (m_getValid) {
          GpioGetVal getState = GPIO_GET_CLEAR;
          this->getOut_out(0, m_getBank, m_getBit, getState);
          state = (GPIO_GET_SET == getState) ? true : false;
      }
  }

  void R5GpioAdapterComponentImpl ::
    waitIn_handler(
        const NATIVE_INT_TYPE portNum,
        GpioWaitBank bank,
        U32 bit,
        bool &alreadySet
    )
  {
      if (m_waitValid) {
          this->waitOut_out(0, m_waitBank, m_waitBit, alreadySet);
      }
  }

} // end namespace R5
