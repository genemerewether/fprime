// ======================================================================
// \title  SpiSlaveDrv.c
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

#include "SpiSlaveDrv.hpp"

#include <R5/TiHal/include/HL_spi.h>
#include <R5/TiHal/include/HL_sys_dma.h>

#include <Fw/Types/Assert.hpp>

#include <string.h>
 #include <stdio.h>  // for printf

// Addresses of SPI Tx/Rx data
#define SPI3_RX_ADDR  ((uint32_t)(&(spiREG3->BUF)) + 2)

#define DMA_SPI3_RX  DMA_REQ14

enum{
    SPI_DMA_REQ_EN = 0x00010000
};


void SetupSpiSlaveDrvDmaReceive(U32 spi_rx_buf, U16* rx_ptr, U32 rx_size, dmaChannel_t dma_channel)
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
    g_dmaCTRLPKT.AUTOINIT  = AUTOINIT_ON;            // Auto-init mode

    dmaSetCtrlPacket(dma_channel, g_dmaCTRLPKT);
}


typedef struct {
    U16* receiveDmaBufferPtr;
    U32 receiveDmaBufferSize;

    U8* receiveHeadPtr;
    U32 ch0_CDADDR;

    U16 syncByte;
    U32 packetSize;

    U32 maxNumPackets;

} SpiSlaveDrvState;

SpiSlaveDrvState spiSlaveDrvState;

void SpiSlaveDrvInit(U16* receiveDmaBufferPtr, U32 receiveBufferSize, U16 syncByte, U32 packetSize)
{
    spiSlaveDrvState.receiveHeadPtr = (U8*)receiveDmaBufferPtr;

    // Choose a better syncByte. We are going to clear things to zero
    FW_ASSERT((syncByte != 0), syncByte);

    spiSlaveDrvState.syncByte = syncByte;
    spiSlaveDrvState.packetSize = packetSize;

    spiSlaveDrvState.receiveDmaBufferPtr = receiveDmaBufferPtr;

    // receiveBufferSize is the number of U16s, which is twice the number of U8s
    // Make sure the dmaBuffer is a multiple of the packet size
    U32 alignedBufferSize = ((receiveBufferSize * 2) / packetSize) * packetSize;

    // cause packets to walk to resync
    alignedBufferSize -= 2;

    FW_ASSERT((alignedBufferSize > 0), receiveBufferSize * 2);

    spiSlaveDrvState.receiveDmaBufferSize = alignedBufferSize;

    spiSlaveDrvState.maxNumPackets = alignedBufferSize / packetSize;

    // Initial value of the WCP[DMA_CH0].CDADDR
    spiSlaveDrvState.ch0_CDADDR = dmaRAMREG->WCP[DMA_CH0].CDADDR;

    memset(spiSlaveDrvState.receiveDmaBufferPtr, 0, spiSlaveDrvState.receiveDmaBufferSize);

    // Slave receive DMA configurations
    SetupSpiSlaveDrvDmaReceive(SPI3_RX_ADDR, spiSlaveDrvState.receiveDmaBufferPtr, spiSlaveDrvState.receiveDmaBufferSize / 2, DMA_CH0);

    // Assign DMA request SPI3 receive to Channel 0
    dmaReqAssign(DMA_CH0, DMA_SPI3_RX);

    // Enable slave RX dma channel (0) with trigger on hardware request
    dmaSetChEnable(DMA_CH0, DMA_HW);

    // Enable slave (SPI3) receive DMA request
    spiREG3->INT0 |= SPI_DMA_REQ_EN;
}


NATIVE_INT_TYPE SpiSlaveDrvReceive(U8* receiveBufPtr, U32 receiveBufSize)
{
    U32 numFoundPackets;

    //U8* tempPtr = (U8*)spiSlaveDrvState.receiveDmaBufferPtr;
    //printf("Mem ");
    //for (int i = 0; i < spiSlaveDrvState.receiveDmaBufferSize; i++) {
        //printf("%.2X ", tempPtr[i]);
    //}
    //printf("\n");

    U8* tailPtr = NULL;
    U8* packetPtr = spiSlaveDrvState.receiveHeadPtr;

    // Locate all potential packets
    for (numFoundPackets = 0; numFoundPackets < spiSlaveDrvState.maxNumPackets; numFoundPackets++) {

        // Compare both Sync bytes
        if (packetPtr[0] != ((spiSlaveDrvState.syncByte >> 8) & 0xFF) ||
            packetPtr[1] != (spiSlaveDrvState.syncByte & 0xFF)) {
            break;
        }

        tailPtr = packetPtr;

        // Increase the packet pointer
        packetPtr += spiSlaveDrvState.packetSize;
        
        if (packetPtr >= ((U8*)spiSlaveDrvState.receiveDmaBufferPtr) + spiSlaveDrvState.receiveDmaBufferSize) {
            packetPtr -= spiSlaveDrvState.receiveDmaBufferSize;
        }
    }

    if (numFoundPackets >= spiSlaveDrvState.maxNumPackets) {
        // Handle this case properly
        // I don't think this will cause any issues, but there may be incomplete/corrupted packets
        //FW_ASSERT(false);
    }

    // tailPtr now points to the beginning of the last packet found. Every packet between
    // head and tail is copied into the receive buffer. The final packet pointed to by tail
    // is not copied as we cannot prove that it is a complete packet

    NATIVE_INT_TYPE rxBytes = 0;

//    if (tailPtr) {
//        printf("tp: 0x%08X 0x%08X\n",tailPtr,spiSlaveDrvState.receiveDmaBufferPtr);
//    }

    if (tailPtr != NULL) {
        // check for rollover
        rxBytes = (U32)(tailPtr) - (U32)(spiSlaveDrvState.receiveHeadPtr);

        if(0 != rxBytes) {
            if(tailPtr > spiSlaveDrvState.receiveHeadPtr) {
                FW_ASSERT((rxBytes <= receiveBufSize), rxBytes, receiveBufSize);

                memcpy(receiveBufPtr, spiSlaveDrvState.receiveHeadPtr, rxBytes);
            }
            else {
                rxBytes = spiSlaveDrvState.receiveDmaBufferSize - ((U32)spiSlaveDrvState.receiveHeadPtr - (U32)tailPtr);
                //rxBytes += (spiSlaveDrvState.receiveDmaBufferSize);
                FW_ASSERT((rxBytes <= receiveBufSize), rxBytes, receiveBufSize);


                U32 numBytes = (U32)(spiSlaveDrvState.receiveDmaBufferPtr) + spiSlaveDrvState.receiveDmaBufferSize - (U32)(spiSlaveDrvState.receiveHeadPtr);
                memcpy(receiveBufPtr, spiSlaveDrvState.receiveHeadPtr, numBytes);

                receiveBufPtr += numBytes;
                U32 numBytes2 = (U32)(tailPtr) - (U32)(spiSlaveDrvState.receiveDmaBufferPtr);
                memcpy(receiveBufPtr, spiSlaveDrvState.receiveDmaBufferPtr, numBytes2);
//                printf("Bytes: %u %u %u\n", rxBytes, numBytes, numBytes2);
            }
#if 1
            // Clear the sync bytes
            U8* tempHeadPtr = spiSlaveDrvState.receiveHeadPtr;

            // Is there any way that this could in up as an infinite loop?
            while (tempHeadPtr != tailPtr) {
                tempHeadPtr[0] = 0;
                tempHeadPtr[1] = 0;

                tempHeadPtr += spiSlaveDrvState.packetSize;

                if (tempHeadPtr >= ((U8*)spiSlaveDrvState.receiveDmaBufferPtr) + spiSlaveDrvState.receiveDmaBufferSize) {
                    tempHeadPtr -= spiSlaveDrvState.receiveDmaBufferSize;
                }
            }
#endif
            // Update the receive head pointer
            spiSlaveDrvState.receiveHeadPtr = tailPtr;
        }
    } else {
        // Handle this case properly
        rxBytes = 0;
    }

//    return 0;
    return rxBytes;
}

