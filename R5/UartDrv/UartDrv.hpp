// ======================================================================
// \title  UartDrv.h
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

#ifndef UARTDRV_HPP_
#define UARTDRV_HPP_

#include <Fw/Types/BasicTypes.hpp>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

    void UartDrvInit(NATIVE_INT_TYPE instance, U8* transmitDmaBufferPtr, U32 transmitBufferSize, U8* receiveDmaBufferPtr, U32  receiveBufferSize);
    void UartDrvSend(NATIVE_INT_TYPE instance, U8* transmitBufPtr, U32 transmitBufSize);
    NATIVE_INT_TYPE UartDrvReceive(NATIVE_INT_TYPE instance, U8* receiveBufPtr, U32 receiveBufSize);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  // UARTDRV_HPP_
