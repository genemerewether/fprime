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

#include <R5/TiHal/include/HL_sci.h>
#include <R5/TiHal/include/HL_sys_dma.h>

#include <Fw/Types/Assert.hpp>

#include <string.h>
// #include <stdio.h>  // for printf

#define UART_NUM  3 // number of UARTs

#define SCI_TX_DMA          (1 << 16)
#define SCI_SET_RX_DMA      (1 << 17)
#define SCI_SET_RX_DMA_ALL  (1 << 18)


static void setupUartDrvDmaReceive(uint32_t rx_addr, U8* rx_ptr, U32 rx_size, dmaChannel_t channel)
{
    g_dmaCTRL g_dmaCTRLPKT;

    // Configure control packet for receive channel
    g_dmaCTRLPKT.SADD      = rx_addr;                // Initial source address
    g_dmaCTRLPKT.DADD      = (uint32_t)rx_ptr;       // Initial destination address
    g_dmaCTRLPKT.CHCTRL    = 0;                      // Next channel to be triggered + 1
    g_dmaCTRLPKT.FRCNT     = rx_size;                // Frame   count
    g_dmaCTRLPKT.ELCNT     = 1;                      // Element count
    g_dmaCTRLPKT.ELDOFFSET = 0;                      // Element destination offset
    g_dmaCTRLPKT.ELSOFFSET = 0;                      // Element source offset
    g_dmaCTRLPKT.FRDOFFSET = 0;                      // Frame destination offset
    g_dmaCTRLPKT.FRSOFFSET = 0;                      // Frame source offset
    g_dmaCTRLPKT.PORTASGN  = PORTB_READ_PORTA_WRITE; // DMA port
    g_dmaCTRLPKT.RDSIZE    = ACCESS_8_BIT;           // Read element size
    g_dmaCTRLPKT.WRSIZE    = ACCESS_8_BIT;           // Write element size
    g_dmaCTRLPKT.TTYPE     = FRAME_TRANSFER;         // Trigger type - frame/block
    g_dmaCTRLPKT.ADDMODERD = ADDR_FIXED;             // Addressing mode for source
    g_dmaCTRLPKT.ADDMODEWR = ADDR_INC1;              // Addressing mode for destination
    g_dmaCTRLPKT.AUTOINIT  = AUTOINIT_ON;            // Auto-init mode

    dmaSetCtrlPacket(channel, g_dmaCTRLPKT);
}


static void configUartDrvDmaTransmit(uint32_t tx_addr, U8* tx_ptr, U32 tx_size, dmaChannel_t channel)
{
    g_dmaCTRL g_dmaCTRLPKT;

    // Configure control packet for transmit channel
    g_dmaCTRLPKT.SADD      = (uint32_t)tx_ptr;       // Initial source address
    g_dmaCTRLPKT.DADD      = tx_addr;           // Initial destination address
    g_dmaCTRLPKT.CHCTRL    = 0;                      // Next channel to be triggered + 1
    g_dmaCTRLPKT.FRCNT     = tx_size;                // Frame   count
    g_dmaCTRLPKT.ELCNT     = 1;                      // Element count
    g_dmaCTRLPKT.ELDOFFSET = 0;                      // Element destination offset
    g_dmaCTRLPKT.ELSOFFSET = 0;                      // Element source offset
    g_dmaCTRLPKT.FRDOFFSET = 0;                      // Frame destination offset
    g_dmaCTRLPKT.FRSOFFSET = 0;                      // Frame source offset
    g_dmaCTRLPKT.PORTASGN  = PORTA_READ_PORTB_WRITE; // DMA port
    g_dmaCTRLPKT.RDSIZE    = ACCESS_8_BIT;           // Read element size
    g_dmaCTRLPKT.WRSIZE    = ACCESS_8_BIT;           // Write element size
    g_dmaCTRLPKT.TTYPE     = FRAME_TRANSFER;         // Trigger type - frame/block
    g_dmaCTRLPKT.ADDMODERD = ADDR_INC1;              // Addressing mode for source
    g_dmaCTRLPKT.ADDMODEWR = ADDR_FIXED;             // Addressing mode for destination
    g_dmaCTRLPKT.AUTOINIT  = AUTOINIT_OFF;           // Auto-init mode

    dmaSetCtrlPacket(channel, g_dmaCTRLPKT);
}


typedef struct {
    U8* transmitDmaBufferPtr;
    U32 transmitDmaBufferSize;

    U8* receiveDmaBufferPtr;
    U32 receiveDmaBufferSize;

    sciBASE_t* p_sci_reg;

    dmaChannel_t rx_dma_ch;
    dmaChannel_t tx_dma_ch;

    U8* receiveHeadPtr;
    U32 transmitFlag;
    U32 ch_CDADDR;
} UartDrvState;

UartDrvState uartDrvState[UART_NUM];

sciBASE_t* uartRegFramePtr[] = {sciREG1, sciREG2, sciREG3, sciREG4};

dmaChannel_t uartDrvDmaRxCh[UART_NUM] = {DMA_CH4, DMA_CH6, DMA_CH8};
dmaChannel_t uartDrvDmaTxCh[UART_NUM] = {DMA_CH5, DMA_CH7, DMA_CH9};

dmaRequest_t uartDrvDmaRx[] = {DMA_REQ28, DMA_REQ40, DMA_REQ30, DMA_REQ42};
dmaRequest_t uartDrvDmaTx[] = {DMA_REQ29, DMA_REQ41, DMA_REQ31, DMA_REQ43};

void UartDrvInit(NATIVE_INT_TYPE instance, U8* transmitDmaBufferPtr, U32 transmitBufferSize, U8* receiveDmaBufferPtr, U32  receiveBufferSize)
{
    FW_ASSERT(instance < FW_NUM_ARRAY_ELEMENTS(uartDrvState));
    UartDrvState* uartDrvStatePtr = &uartDrvState[instance];

    if(0 < transmitBufferSize) {
        FW_ASSERT(0 != transmitDmaBufferPtr);
    }
    else {
        FW_ASSERT(0 == transmitDmaBufferPtr);
    }
    uartDrvStatePtr->transmitDmaBufferPtr = transmitDmaBufferPtr;
    uartDrvStatePtr->transmitDmaBufferSize = transmitBufferSize;

    if(0 < receiveBufferSize) {
        FW_ASSERT(0 != receiveDmaBufferPtr);
    }
    else {
        FW_ASSERT(0 == receiveDmaBufferPtr);
    }
    uartDrvStatePtr->receiveDmaBufferPtr = receiveDmaBufferPtr;
    uartDrvStatePtr->receiveDmaBufferSize = receiveBufferSize;

    uartDrvStatePtr->p_sci_reg = uartRegFramePtr[instance];

    uartDrvStatePtr->rx_dma_ch = uartDrvDmaRxCh[instance];
    uartDrvStatePtr->tx_dma_ch = uartDrvDmaTxCh[instance];

    uartDrvStatePtr->receiveHeadPtr = receiveDmaBufferPtr;
    uartDrvStatePtr->transmitFlag = 0;

    // Initial value of the WCP[DMA_CH*].CDADDR
    uartDrvStatePtr->ch_CDADDR = dmaRAMREG->WCP[uartDrvStatePtr->rx_dma_ch].CDADDR;

    setupUartDrvDmaReceive(((uint32_t)(&(uartDrvStatePtr->p_sci_reg->RD)) + 3), uartDrvStatePtr->receiveDmaBufferPtr, uartDrvStatePtr->receiveDmaBufferSize, uartDrvStatePtr->rx_dma_ch);

    // Assign DMA request for SCI receive
    dmaReqAssign(uartDrvStatePtr->rx_dma_ch, uartDrvDmaRx[instance]);

    configUartDrvDmaTransmit(((uint32_t)(&(uartDrvStatePtr->p_sci_reg->TD)) + 3), uartDrvStatePtr->transmitDmaBufferPtr, uartDrvStatePtr->transmitDmaBufferSize, uartDrvStatePtr->tx_dma_ch);

    // Assign DMA request for SCI transmit
    dmaReqAssign(uartDrvStatePtr->tx_dma_ch, uartDrvDmaTx[instance]);

    // Set receive DMA channel to trigger on hardware request
    dmaSetChEnable(uartDrvStatePtr->rx_dma_ch, DMA_HW);

    // Enable SCI Receive DMA Request
    uartDrvStatePtr->p_sci_reg->SETINT = SCI_SET_RX_DMA | SCI_SET_RX_DMA_ALL;
}


void UartDrvSend(NATIVE_INT_TYPE instance, U8* transmitBufPtr, U32 transmitBufSize)
{
    FW_ASSERT(instance < FW_NUM_ARRAY_ELEMENTS(uartDrvState));
    UartDrvState* uartDrvStatePtr = &uartDrvState[instance];
    FW_ASSERT(0 != uartDrvStatePtr->transmitDmaBufferPtr);

    if(uartDrvStatePtr->transmitFlag) {
        // Wait for the DMA BTC interrupt status (DMA transmit channel done)
        U32 reg_btc;
        do {
            reg_btc = ((1 << uartDrvStatePtr->tx_dma_ch) & dmaREG->BTCFLAG);
        } while(0 == reg_btc);

        // Clear the DMA BTC interrupt status bit
        dmaREG->BTCFLAG = reg_btc;

        // Disable SCI1 Transmit DMA Request
        uartDrvStatePtr->p_sci_reg->CLEARINT = SCI_TX_DMA;
    }

    FW_ASSERT(((0 < transmitBufSize) && (transmitBufSize <= uartDrvStatePtr->transmitDmaBufferSize)), transmitBufSize, uartDrvStatePtr->transmitDmaBufferSize);

    // Copy transmit data into the DMA buffer
    memcpy(uartDrvStatePtr->transmitDmaBufferPtr, transmitBufPtr, transmitBufSize);

    configUartDrvDmaTransmit(((uint32_t)(&(uartDrvStatePtr->p_sci_reg->TD)) + 3), uartDrvStatePtr->transmitDmaBufferPtr, transmitBufSize, uartDrvStatePtr->tx_dma_ch);

    dmaSetChEnable(uartDrvStatePtr->tx_dma_ch, DMA_HW);

    // Enable SCI Transmit DMA Request
    uartDrvStatePtr->p_sci_reg->SETINT = SCI_TX_DMA;

    uartDrvStatePtr->transmitFlag = 1;
}


NATIVE_INT_TYPE UartDrvReceive(NATIVE_INT_TYPE instance, U8* receiveBufPtr, U32 receiveBufSize)
{
    FW_ASSERT(instance < FW_NUM_ARRAY_ELEMENTS(uartDrvState));
    UartDrvState* uartDrvStatePtr = &uartDrvState[instance];
    FW_ASSERT(0 != uartDrvStatePtr->receiveHeadPtr);

    // Get the RX DMA current destination address
    // Note: the tailPtr is valid if in the range of [receiveDmaBufferPtr+1,receiveDmaBufferPtr+DmaBufferSize], as it always points to where the next byte should be written.
    U8* tailPtr = (U8*)dmaREG->FAACDADDR;
    U32 cdaddr = dmaRAMREG->WCP[uartDrvStatePtr->rx_dma_ch].CDADDR;
    if((tailPtr <= uartDrvStatePtr->receiveDmaBufferPtr) || (&uartDrvStatePtr->receiveDmaBufferPtr[uartDrvStatePtr->receiveDmaBufferSize] < tailPtr)) {
        tailPtr = (U8*)dmaREG->FBACDADDR;
        if((tailPtr <= uartDrvStatePtr->receiveDmaBufferPtr) || (&uartDrvStatePtr->receiveDmaBufferPtr[uartDrvStatePtr->receiveDmaBufferSize] < tailPtr)) {
            if(cdaddr != uartDrvStatePtr->ch_CDADDR) {
                tailPtr = (U8*)cdaddr;
            }
            else {
                tailPtr = 0;
            }
        }
    }

    if((tailPtr <= uartDrvStatePtr->receiveDmaBufferPtr) || (&uartDrvStatePtr->receiveDmaBufferPtr[uartDrvStatePtr->receiveDmaBufferSize] < tailPtr)) {
        return 0;
    }

    // GG: Storing the previous value of this register, so that if check above will set
    //     tailPtr = 0 if the register value did not change.  This was causing a bug where
    //     this register value was stale, and so more data was read than valid.
    //uartDrvStatePtr->ch_CDADDR = 0;
    uartDrvStatePtr->ch_CDADDR = cdaddr;

    NATIVE_INT_TYPE rxBytes = tailPtr - uartDrvStatePtr->receiveHeadPtr;

    if(0 != rxBytes) {
        if(0 < rxBytes) {
            FW_ASSERT((rxBytes <= receiveBufSize), rxBytes, receiveBufSize);

            memcpy(receiveBufPtr, uartDrvStatePtr->receiveHeadPtr, rxBytes);
        }
        else {
            rxBytes += uartDrvStatePtr->receiveDmaBufferSize;
            FW_ASSERT((rxBytes <= receiveBufSize), rxBytes, receiveBufSize);

            U32 numBytes = &uartDrvStatePtr->receiveDmaBufferPtr[uartDrvStatePtr->receiveDmaBufferSize] - uartDrvStatePtr->receiveHeadPtr;
            memcpy(receiveBufPtr, uartDrvStatePtr->receiveHeadPtr, numBytes);

            receiveBufPtr += numBytes;
            numBytes = tailPtr - uartDrvStatePtr->receiveDmaBufferPtr;
            memcpy(receiveBufPtr, uartDrvStatePtr->receiveDmaBufferPtr, numBytes);
        }

        // Update the receive head pointer
        uartDrvStatePtr->receiveHeadPtr = tailPtr;
    }

    return rxBytes;
}

