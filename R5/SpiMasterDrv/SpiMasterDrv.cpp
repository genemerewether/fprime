// ======================================================================
// \title  SpiMasterDrv.c
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

#include "SpiMasterDrv.hpp"

#include <R5/TiHal/include/HL_sys_dma.h>

#include <Fw/Types/Assert.hpp>

#include <string.h>
// #include <stdio.h>  // for printf

// Addresses of SPI Tx/Rx data
#define SPI1_RX_ADDR ((uint32_t)(&(spiREG1->BUF)) + 2)
#define SPI1_TX_ADDR ((uint32_t)(&(spiREG1->DAT1)))

#define DMA_SPI1_RX  DMA_REQ0
#define DMA_SPI1_TX  DMA_REQ1

#define DMA_CH2_MASK  (1 << 2)
#define DMA_CH3_MASK  (1 << 3)

void configSpiMasterDrvDmaReceive(U32 spi_rx_buf, U16* rx_ptr, U32 rx_size, dmaChannel_t dma_channel)
{
    g_dmaCTRL g_dmaCTRLPKT;

    // Configure control packet
    g_dmaCTRLPKT.SADD      = spi_rx_buf;             // Initial source address
    g_dmaCTRLPKT.DADD      = (uint32_t)rx_ptr;       // Initial destination address
    g_dmaCTRLPKT.CHCTRL    = 0;                      // Next channel to be triggered + 1
    g_dmaCTRLPKT.FRCNT     = rx_size;                // Frame   count
    g_dmaCTRLPKT.ELCNT     = 1;                      // Element count
    g_dmaCTRLPKT.ELDOFFSET = 0;                      // Element destination offset
    g_dmaCTRLPKT.ELSOFFSET = 0;                      // Element source offset
    g_dmaCTRLPKT.FRDOFFSET = 0;                      // Frame destination offset
    g_dmaCTRLPKT.FRSOFFSET = 0;                      // Frame source offset
    g_dmaCTRLPKT.PORTASGN  = PORTB_READ_PORTA_WRITE; // DMA port
    g_dmaCTRLPKT.RDSIZE    = ACCESS_16_BIT;          // Read element size
    g_dmaCTRLPKT.WRSIZE    = ACCESS_16_BIT;          // Write element size
    g_dmaCTRLPKT.TTYPE     = FRAME_TRANSFER;         // Trigger type - frame/block
    g_dmaCTRLPKT.ADDMODERD = ADDR_FIXED;             // Addressing mode for source
    g_dmaCTRLPKT.ADDMODEWR = ADDR_INC1;              // Addressing mode for destination
    g_dmaCTRLPKT.AUTOINIT  = AUTOINIT_OFF;           // Auto-init mode

    dmaSetCtrlPacket(dma_channel, g_dmaCTRLPKT);
}


void configSpiMasterDrvDmaTransmit(U32 spi_tx_buf, U32* tx_ptr, U32 tx_size, dmaChannel_t dma_channel)
{
    g_dmaCTRL g_dmaCTRLPKT;

    // Configure control packet
    g_dmaCTRLPKT.SADD      = (uint32_t)tx_ptr;       // Initial source address
    g_dmaCTRLPKT.DADD      = spi_tx_buf;             // Initial destination address
    g_dmaCTRLPKT.CHCTRL    = 0;                      // Next channel to be triggered + 1
    g_dmaCTRLPKT.FRCNT     = tx_size;                // Frame   count
    g_dmaCTRLPKT.ELCNT     = 1;                      // Element count
    g_dmaCTRLPKT.ELDOFFSET = 0;                      // Element destination offset
    g_dmaCTRLPKT.ELSOFFSET = 0;                      // Element source offset
    g_dmaCTRLPKT.FRDOFFSET = 0;                      // Frame destination offset
    g_dmaCTRLPKT.FRSOFFSET = 0;                      // Frame source offset
    g_dmaCTRLPKT.PORTASGN  = PORTA_READ_PORTB_WRITE; // DMA port
    g_dmaCTRLPKT.RDSIZE    = ACCESS_32_BIT;          // Read element size
    g_dmaCTRLPKT.WRSIZE    = ACCESS_32_BIT;          // Write element size
    g_dmaCTRLPKT.TTYPE     = FRAME_TRANSFER;         // Trigger type - frame/block
    g_dmaCTRLPKT.ADDMODERD = ADDR_INC1;              // Addressing mode for source
    g_dmaCTRLPKT.ADDMODEWR = ADDR_FIXED;             // Addressing mode for destination
    g_dmaCTRLPKT.AUTOINIT  = AUTOINIT_OFF;           // Auto-init mode

    dmaSetCtrlPacket(dma_channel, g_dmaCTRLPKT);
}


typedef struct {
    U32* transmitDmaBufferPtr;
    U32 transmitDmaBufferSize;   // Number of U16
    U32 lastTransmitSize;        // Number of U8
    U32 currTransmitBufferSize;  // Number of U16

    U16* receiveDmaBufferPtr;
    U32 receiveDmaBufferSize;   // Number of U16

    U32 transmitFlag;
} SpiMasterDrvState;

SpiMasterDrvState spiMasterDrvState;

void SpiMasterDrvInit(U32* transmitDmaBufferPtr, U32 transmitBufferSize, U16* receiveDmaBufferPtr, U32  receiveBufferSize)
{
    spiMasterDrvState.transmitDmaBufferPtr = transmitDmaBufferPtr;
    spiMasterDrvState.transmitDmaBufferSize = transmitBufferSize;
    spiMasterDrvState.currTransmitBufferSize = 0;

    spiMasterDrvState.receiveDmaBufferPtr = receiveDmaBufferPtr;
    spiMasterDrvState.receiveDmaBufferSize = receiveBufferSize;

    // Master receive DMA configuration
    configSpiMasterDrvDmaReceive(SPI1_RX_ADDR, spiMasterDrvState.receiveDmaBufferPtr, spiMasterDrvState.receiveDmaBufferSize, DMA_CH2);

    // Assign DMA request SPI1 receive to channel 2
    dmaReqAssign(DMA_CH2, DMA_SPI1_RX);

    configSpiMasterDrvDmaTransmit(SPI1_TX_ADDR, spiMasterDrvState.transmitDmaBufferPtr, spiMasterDrvState.transmitDmaBufferSize, DMA_CH3);

    // Assign DMA request SPI1 transmit to channel 3
    dmaReqAssign(DMA_CH3, DMA_SPI1_TX);
}


void SpiMasterDrvSend(U16* transmitBufPtr, U32 transmitBufSize, bool lastBuff)
{
    if(spiMasterDrvState.transmitFlag) {
        // Wait for the DMA BTC interrupt status (DMA channel 2 receive done)
        U32 reg_btc;
        do {
            reg_btc = ((DMA_CH3_MASK | DMA_CH2_MASK) & dmaREG->BTCFLAG);
        } while((DMA_CH3_MASK | DMA_CH2_MASK) != reg_btc);

        // Clear the DMA BTC interrupt status bit
        dmaREG->BTCFLAG = reg_btc;

        // Disable SPI1 DMA Request
        spiREG1->INT0 &= ~SPI_DMA_REQ_EN;

        spiMasterDrvState.transmitFlag = 0;
    }

    FW_ASSERT(((0 < transmitBufSize) && ((spiMasterDrvState.currTransmitBufferSize + transmitBufSize) <= spiMasterDrvState.transmitDmaBufferSize)), spiMasterDrvState.currTransmitBufferSize, transmitBufSize, spiMasterDrvState.transmitDmaBufferSize);

    U32* curTrBuffPtr = &spiMasterDrvState.transmitDmaBufferPtr[spiMasterDrvState.currTransmitBufferSize];
    spiMasterDrvState.currTransmitBufferSize += transmitBufSize;

    U32 i = 0;
    --transmitBufSize;
    for(; i < transmitBufSize; ++i) {
        // Copy transmit data into the DMA buffer
        curTrBuffPtr[i] = SPI_TX_CONFIG | transmitBufPtr[i];
    }

    // Copy the last half-word into DMA buffer
    curTrBuffPtr[i] = SPI_TX_CONFIG_LAST | transmitBufPtr[i];

    if(lastBuff) {
        spiMasterDrvState.lastTransmitSize = sizeof(U16) * spiMasterDrvState.currTransmitBufferSize;

        configSpiMasterDrvDmaReceive(SPI1_RX_ADDR, spiMasterDrvState.receiveDmaBufferPtr, spiMasterDrvState.currTransmitBufferSize, DMA_CH2);
        configSpiMasterDrvDmaTransmit(SPI1_TX_ADDR, spiMasterDrvState.transmitDmaBufferPtr, spiMasterDrvState.currTransmitBufferSize, DMA_CH3);

        spiMasterDrvState.currTransmitBufferSize = 0;

        dmaSetChEnable(DMA_CH2, DMA_HW);
        dmaSetChEnable(DMA_CH3, DMA_HW);

        // Enable SPI1 receive and transmit DMA requests
        spiREG1->INT0 |= SPI_DMA_REQ_EN;

        spiMasterDrvState.transmitFlag = 1;
    }
}


NATIVE_INT_TYPE SpiMasterDrvReceive(U8* receiveBufPtr, U32 receiveBufSize)
{
    if(spiMasterDrvState.transmitFlag) {
        // Wait for the DMA BTC interrupt status (DMA channel 2 receive done)
        U32 reg_btc;
        do {
            reg_btc = ((DMA_CH3_MASK | DMA_CH2_MASK) & dmaREG->BTCFLAG);
        } while((DMA_CH3_MASK | DMA_CH2_MASK) != reg_btc);
    }

    FW_ASSERT((spiMasterDrvState.lastTransmitSize <= receiveBufSize), spiMasterDrvState.lastTransmitSize, receiveBufSize);

    // Copy received data from the DMA buffer
    memcpy(receiveBufPtr, spiMasterDrvState.receiveDmaBufferPtr, spiMasterDrvState.lastTransmitSize);

    return spiMasterDrvState.lastTransmitSize;
}
