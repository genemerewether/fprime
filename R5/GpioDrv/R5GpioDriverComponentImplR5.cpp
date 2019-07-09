// ======================================================================
// \title  R5GpioDriverImpl.cpp
// \author tcanham
// \brief  cpp file for R5GpioDriver component implementation class
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

#include <R5/GpioDrv/R5GpioDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include <Fw/Types/Assert.hpp>

#include <R5/TiHal/include/HL_gio.h>
#include <R5/TiHal/include/HL_het.h>

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  R5GpioDriverComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5GpioDriverComponentImpl(
        const char *const compName
    ) :
      R5GpioDriverComponentBase(compName)
#else
    R5GpioDriverImpl(void)
#endif
  {

  }

  void R5GpioDriverComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    R5GpioDriverComponentBase::init(instance);
  }

  R5GpioDriverComponentImpl ::
    ~R5GpioDriverComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5GpioDriverComponentImpl ::
    set_handler(
        const NATIVE_INT_TYPE portNum,
        GpioSetBank bank,
        U32 bit,
        GpioSetVal val
    )
  {
      gioPORT_t* port;

      switch (bank) {
          case GPIO_SET_BANK_A:
              port = gioPORTA;
              break;
          case GPIO_SET_BANK_B:
              port = gioPORTB;
              break;
          case GPIO_SET_HET_1:
              port = hetPORT1;
              break;
          case GPIO_SET_HET_2:
              port = hetPORT2;
              break;
          default:
              FW_ASSERT(0,bank);
              break;
      }

      U32 value = (GPIO_SET_CLEAR == val) ? 0 : 1;

      gioSetBit(port, bit, value);
  }

  void R5GpioDriverComponentImpl ::
    get_handler(
        const NATIVE_INT_TYPE portNum,
        GpioGetBank bank,
        U32 bit,
        GpioGetVal &val
    )
  {
      gioPORT_t* port;

      switch (bank) {
          case GPIO_GET_BANK_A:
              port = gioPORTA;
              break;
          case GPIO_GET_BANK_B:
              port = gioPORTB;
              break;
          case GPIO_GET_HET_1:
              port = hetPORT1;
              break;
          case GPIO_GET_HET_2:
              port = hetPORT2;
              break;
          default:
              FW_ASSERT(0,bank);
              break;
      }

      val = (0 == gioGetBit(port, bit)) ? GPIO_GET_CLEAR : GPIO_GET_SET;
  }

  void R5GpioDriverComponentImpl ::
    wait_handler(
        const NATIVE_INT_TYPE portNum,
        GpioWaitBank bank,
        U32 bit,
        bool &alreadySet
    )
  {
      FW_ASSERT((bit < 8), bit);

      U32 shift = (GPIO_SET_BANK_A == bank) ? 0 : 8;
      U32 mask = ((1 << bit) << shift);
      U32 gio_flag_reg = (mask & gioREG->FLG);

      alreadySet = (0 != gio_flag_reg);

      volatile int i;
      for (i = 0; i < 500; i++);

      //while(0 == gio_flag_reg) {
          //gio_flag_reg = (mask & gioREG->FLG);
      //}

      gioREG->FLG = gio_flag_reg;
  }

} // end namespace R5
