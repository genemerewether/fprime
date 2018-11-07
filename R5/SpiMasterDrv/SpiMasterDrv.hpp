// ======================================================================
// \title  SpiMasterDrv.h
// \author Igor Uchenik
// \brief  SpiMaster Driver component implementation class
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

#ifndef SPIMASTERDRV_HPP_
#define SPIMASTERDRV_HPP_

#include <Fw/Types/BasicTypes.hpp>

#include <R5/TiHal/include/HL_spi.h>

#ifdef __cplusplus
extern "C" {
#endif  // __cplusplus

enum {
    SPI_TX_CONFIG = 0x10000000 | ((U32)SPI_FMT_0 << 24) | ((U32)SPI_CS_0 << 16),
    SPI_TX_CONFIG_LAST =  0x04000000 | ((U32)SPI_FMT_0 << 24U) | ((U32)SPI_CS_0 << 16U),

    SPI_DMA_REQ_EN = 0x00010000
};

// transmitBufferSize is a number of entries in the transmitDmaBufferPtr array
// receiveBufferSize is a number of entries in the receiveDmaBufferPtr array
void SpiMasterDrvInit(U32* transmitDmaBufferPtr, U32 transmitBufferSize, U16* receiveDmaBufferPtr, U32  receiveBufferSize);

// transmitBufSize is a number of entries in the transmitBufPtr array
void SpiMasterDrvSend(U16* transmitBufPtr, U32 transmitBufSize, U32 receiveBufSize, bool lastBuff);

// receiveBufSize is a number of entries in the receiveBufPtr array
NATIVE_INT_TYPE SpiMasterDrvReceive(U8* receiveBufPtr, U32 receiveBufSize);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus

#endif  // SPIMASTERDRV_HPP_
