// ======================================================================
// \title  SpiSlaveDrv.h
// \author Igor Uchenik
// \brief  SpiSlave Driver component implementation class
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

#ifndef SPISLAVEDRV_HPP_
#define SPISLAVEDRV_HPP_

#include <Fw/Types/BasicTypes.hpp>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

void SpiSlaveDrvInit(U16* receiveDmaBufferPtr, U32 receiveBufferSize, U16 syncByte, U32 packetSize);
NATIVE_INT_TYPE SpiSlaveDrvReceive(U8* receiveBufPtr, U32 receiveBufSize);
void SpiSlaveRestartReceiver(void);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  // SPISLAVEDRV_HPP_
