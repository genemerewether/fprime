// ======================================================================
// \title  UartDrv.c
// \author Igor Uchenik
// \brief  Uart Driver component implementation class
//
// \copyright
// Copyright 2009-2017, by the California Institute of Technology.
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

#include "UartDrv.hpp"

void UartDrvInit(NATIVE_INT_TYPE instance, U8* transmitDmaBuffer, U32 transmitBufferSize, U8* receiveDmaBuffer, U32  receiveBufferSize) {

}

void UartDrvSend(NATIVE_INT_TYPE instance, U8* transmitPtr, U32 transmitSize) {

}

NATIVE_INT_TYPE UartDrvReceive(NATIVE_INT_TYPE instance, U8* receivePtr, U32 receiveSize) {
    return 0;
}
