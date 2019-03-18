// ======================================================================
// \title  I2CDrv.c
// \author Gerik Kubiak
// \brief  I2C Driver component implementation class
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

#include "I2CDrv.hpp"

#include <R5/TiHal/include/HL_i2c.h>
#include <R5/TiHal/include/HL_reg_i2c.h>
#include <R5/TiHal/include/HL_sys_dma.h>

#include <Fw/Types/Assert.hpp>

#include <string.h>
// #include <stdio.h>  // for printf

#define I2C_NUM  2 // number of I2C peripherals
#define I2C_MAX_NUM_PORTS 16

#define I2C_MIN_TX_BUFF_SIZE (32)
#define I2C_MIN_RX_BUFF_SIZE (32 * 4) // See DMA Oddities


/* **************** *
 * I2C Driver Usage *
 * **************** *
 *
 * Each F' component has a unique numbered port into the I2C driver.
 * This port number is used as an index throughout the driver. Each
 * port may communicate to a single I2C Slave. If a component wishes
 * to talk with multiple I2C Slaves, multiple Config and ReadWrite ports
 * must be used.
 *
 * The Driver must be initialization in the Topology by calling I2CDrvInit
 * for each individual I2C peripheral instance to use. The TMS570LC4357 has
 * two I2C peripherals. I2C1 is mapped to instance 0 and I2C2 is mapped to
 * instance 1. Attempting to use another instance value will result in an
 * assert.
 *
 * Each component that would like to use the driver must configure it's
 * specific settings through the Config port. This step configures both
 * the bus speed and the I2C slave address to command with WriteRead calls.
 * The I2C driver for the TMS570LC4357 ignores the timeout field of the
 * Config port.
 *
 * Once the port has been configured, transaction can be created with the
 * I2CDrvAddTxn function. Each port is allowed to have a single transaction
 * in progress at a given time. A transaction consist of both an optional
 * I2C write and and optional I2C read call. A buffer size of 0 indicates that
 * the write or read call should be skipped. If both read and write buffers
 * have a size of zero, the I2C driver will attempt to address the slave
 * through its read address, but will not read any data. This provides a
 * means to check that a device exists without commanding the device.
 *
 * Transaction are not executed until the I2CDrvService function is called.
 * This function checks the status of existing transactions, cleans up
 * completed transactions and starts executing new transactions if possible.
 * I2CDrvService should be called at a regular rate for the best performance.
 * At most one I2C read or write sequence can occur during servicing. If
 * I2CDrvService rs called once per RTI, then it will take a minimum of two
 * RTIs for a WriteRead transaction to complete. I2CDrvService may be called
 * multiple times in an RTI, but it is recommended that these calls be placed
 * as far apart as possible.
 *
 * A transaction is completed with a call to I2CDrvCheckTxn. This function
 * checks if a transaction has completed, and if so, returns that status of
 * the transaction. This status can be one of the following:
 *   BUSY: Still processing the transaction. Check again later
 *   OK: Transaction completed successfully. The Rx passed into the driver
 *       will have the read data if applicable.
 *   TX_NACK: Received a NACK when attempting a write
 *   RX_NACK: Received a NACK when attempting a read
 * The I2CDrvCheckTxn call may optionally block until the transaction completes.
 * This is accomplished by calling I2CDrvService in a loop until the driver
 * notices the transaction has completed. Because of the speed of the I2C bus,
 * this may take several microseconds. If I2CDrvCheckTxn is called in non-blocking
 * mode, the wouldBlock reference will be set to true if the transaction would
 * have blocked.
 */

/*
 * *********************** *
 * Executing a transaction *
 * *********************** *
 *
 * There are three phases of transactions, Tx, Rx and Nack. Each is executed with
 * a call to I2CDrvExecuteTxn.
 *
 * A Tx transaction addresses the write address of the specified slave. If the slaves
 * acks then the entire txDmaBuffer is sent across the wire byte by byte. This process
 * is entirely autonomous using the DMA to grab bytes from the I2C peripheral and store
 * them in the drivers allocated DMA memory. Calls to I2CService check on the status of
 * the DMA to determine when the transaction has completed.
 *
 * An Rx transaction addresses the read address of the specified slave and reads the
 * requested number of bytes into the supplied receive buffer. This is done in two
 * stages. First, the DMA receives data directly from the I2C peripheral into a DMA
 * memory buffer. When the transaction completes the memory is then transfered
 * and packed into the supplied user buffer.
 *
 * A Nack transaction is a special state to handle I2C slave nacks. The Nack flag is
 * first cleared then a STOP_CONDITION command is sent to the I2C peripheral. Subsequent
 * calls to I2CService poll the status of the Stop Condition Detect (SCD) flag. This
 * guarentees that a Nack condition is properly cleared in the I2C peripheral.
 */

/* 
 * ************ *
 * DMA Oddities *
 * ************ *
 *
 * Transmit:
 * It seems as though the transmit address is sensitive to endianness issues. An offset
 * of 3 bytes is added to the address of the I2C DXR register to compensate for this.
 *
 *
 * Receive:
 * It seems as though I2C data can only be read through 32-bit reads from the I2C
 * peripheral. This means that the DMA Rx buffer needs room to store four times the
 * number of necessary bytes. This data then needs to be packed before sending it
 * back to the user.
 */

/*
 * ********************** *
 * TI Undocument Behavior *
 * ********************** *
 *
 * * DMA port writes
 *   * If seems as though normal 8-bit reads/writes to the I2C Tx and Rx registers
 *     does not work as expected. See DMA Oddities
 *
 * * Must clear NACK flag before writing Stop bit
 *   * The NACK status flag must be cleared after an address NACK before a stop
 *     condition can be sent. Failure to do so will leave the I2C SCL line low
 *     and the bus busy indefinately
 *
 * * Arduino-like pinout on launchpad documentation incorrect about I2C pin placement
 *   * Do not use the provided pinout for connecting an I2C peripheral to the TI Hercules
 *     Launchpad XL board. Use the provided schematics on TI's website instead.
 */

static void setupI2CDrvDmaReceive(uint32_t rx_addr, U8* rx_ptr, U32 rx_size, dmaChannel_t channel)
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
    g_dmaCTRLPKT.RDSIZE    = ACCESS_32_BIT;           // Read element size
    g_dmaCTRLPKT.WRSIZE    = ACCESS_32_BIT;           // Write element size
    g_dmaCTRLPKT.TTYPE     = FRAME_TRANSFER;         // Trigger type - frame/block
    g_dmaCTRLPKT.ADDMODERD = ADDR_FIXED;             // Addressing mode for source
    g_dmaCTRLPKT.ADDMODEWR = ADDR_INC1;              // Addressing mode for destination
    g_dmaCTRLPKT.AUTOINIT  = AUTOINIT_OFF;            // Auto-init mode

    dmaSetCtrlPacket(channel, g_dmaCTRLPKT);
}


static void setupI2CDrvDmaTransmit(uint32_t tx_addr, const U8* tx_ptr, const U32 tx_size, const dmaChannel_t channel)
{
    g_dmaCTRL g_dmaCTRLPKT;

    // Configure control packet for transmit channel
    g_dmaCTRLPKT.SADD      = (uint32_t)tx_ptr;       // Initial source address
    g_dmaCTRLPKT.DADD      = tx_addr + 3;            // Initial destination address
    g_dmaCTRLPKT.CHCTRL    = 0;                      // Next channel to be triggered + 1
    g_dmaCTRLPKT.FRCNT     = tx_size;                // Frame   count
    g_dmaCTRLPKT.ELCNT     = 1;                      // Element count
    g_dmaCTRLPKT.ELDOFFSET = 0;                      // Element destination offset
    g_dmaCTRLPKT.ELSOFFSET = 0;                      // Element source offset
    g_dmaCTRLPKT.FRDOFFSET = 0;                      // Frame destination offset
    g_dmaCTRLPKT.FRSOFFSET = 4;                      // Frame source offset
    g_dmaCTRLPKT.PORTASGN  = PORTA_READ_PORTB_WRITE; // DMA port
    g_dmaCTRLPKT.RDSIZE    = ACCESS_8_BIT;           // Read element size
    g_dmaCTRLPKT.WRSIZE    = ACCESS_8_BIT;           // Write element size
    g_dmaCTRLPKT.TTYPE     = FRAME_TRANSFER;         // Trigger type - frame/block
    g_dmaCTRLPKT.ADDMODERD = ADDR_OFFSET;            // Addressing mode for source
    g_dmaCTRLPKT.ADDMODEWR = ADDR_FIXED;             // Addressing mode for destination
    g_dmaCTRLPKT.AUTOINIT  = AUTOINIT_OFF;           // Auto-init mode

    dmaSetCtrlPacket(channel, g_dmaCTRLPKT);
}

enum TransactionState {
    TXN_IDLE,
    TXN_TX,
    TXN_RX,
    TXN_NACK,
    TXN_DONE
};

typedef struct {
    const U8* txDmaBufferPtr;
    U32 txDmaBufferSize;

    U8* rxDmaBufferPtr;
    U32 rxDmaBufferSize;

    TransactionState txnState;
    I2CTransactionStatus txnStatus;

} I2CPortTransaction;

typedef struct {
    U8 i2cAddr;
    I2CBusSpeed speed;
    bool valid;
} I2CPortConfig;


static I2CPortTransaction portTransaction[I2C_NUM][I2C_MAX_NUM_PORTS];
static I2CPortConfig portConfig[I2C_NUM][I2C_MAX_NUM_PORTS];

static NATIVE_INT_TYPE currentTxn[I2C_NUM];

static i2cBASE_t* i2cRegFramePtr[] = {i2cREG1, i2cREG2};

static U8* i2cRxDmaMem[I2C_NUM] = {NULL,NULL};
static U8* i2cTxDmaMem[I2C_NUM] = {NULL,NULL};

static U32 i2cRxDmaMemSize[I2C_NUM] = {0, 0};
static U32 i2cTxDmaMemSize[I2C_NUM] = {0, 0};

static const dmaChannel_t i2cDrvDmaRxCh[I2C_NUM] = {DMA_CH10, DMA_CH12};
static const dmaChannel_t i2cDrvDmaTxCh[I2C_NUM] = {DMA_CH11, DMA_CH13};

static const dmaRequest_t i2cDrvDmaRx[] = {DMA_REQ10, DMA_REQ32};
static const dmaRequest_t i2cDrvDmaTx[] = {DMA_REQ11, DMA_REQ33};

void I2CDrvInit(const NATIVE_INT_TYPE instance,
                U8* rxDmaMem, U32 rxDmaMemSize,
                U8* txDmaMem, U32 txDmaMemSize)
{
    FW_ASSERT(instance >= 0 && instance < I2C_NUM, instance);

    FW_ASSERT(rxDmaMemSize >= I2C_MIN_RX_BUFF_SIZE, rxDmaMemSize);
    FW_ASSERT(txDmaMemSize >= I2C_MIN_TX_BUFF_SIZE, txDmaMemSize);

    FW_ASSERT(rxDmaMem != NULL, reinterpret_cast<U32>(rxDmaMem));
    FW_ASSERT(txDmaMem != NULL, reinterpret_cast<U32>(txDmaMem));

    NATIVE_UINT_TYPE port;

    for (port = 0; port < I2C_MAX_NUM_PORTS; port++) {
        portConfig[instance][port].valid = false;
        portTransaction[instance][port].txnState = TXN_IDLE;
    }

    currentTxn[instance] = -1;

    i2cRxDmaMem[instance] = rxDmaMem;
    i2cTxDmaMem[instance] = txDmaMem;

    i2cRxDmaMemSize[instance] = rxDmaMemSize;
    i2cTxDmaMemSize[instance] = txDmaMemSize;
}

void I2CDrvConfigPort(const NATIVE_INT_TYPE instance,
                      const NATIVE_INT_TYPE port,
                      const I2CBusSpeed speed,
                      const U32 timeoutUs,
                      const U8 i2cAddr)
{
    FW_ASSERT(instance >= 0 && instance < I2C_NUM, instance);
    FW_ASSERT(port >= 0 && port < I2C_MAX_NUM_PORTS, port);

    FW_ASSERT(i2cAddr < 128, i2cAddr);

    (void)timeoutUs;

    I2CPortConfig* config_p = &portConfig[instance][port];

    config_p->i2cAddr = i2cAddr;
    config_p->speed = speed;
    config_p->valid = true;
}

void I2CDrvAddTxn(const NATIVE_INT_TYPE instance,
                  const NATIVE_INT_TYPE port,
                  const U8* txDmaBufferPtr,
                  U32 txDmaBufferSize,
                  U8* rxDmaBufferPtr,
                  U32 rxDmaBufferSize)
{
    FW_ASSERT(instance >= 0 && instance < I2C_NUM, instance);
    FW_ASSERT(port >= 0 && port < I2C_MAX_NUM_PORTS, port);

    FW_ASSERT(portConfig[instance][port].valid == true, instance, port);

    FW_ASSERT(rxDmaBufferSize < i2cRxDmaMemSize[instance], rxDmaBufferSize, i2cRxDmaMemSize[instance]);
    FW_ASSERT(txDmaBufferSize < i2cTxDmaMemSize[instance], txDmaBufferSize, i2cTxDmaMemSize[instance]);

    I2CPortTransaction* txn_p = &portTransaction[instance][port];

    txn_p->txDmaBufferPtr = txDmaBufferPtr;
    txn_p->txDmaBufferSize = txDmaBufferSize;
    txn_p->rxDmaBufferPtr = rxDmaBufferPtr;
    txn_p->rxDmaBufferSize = rxDmaBufferSize;

    if (txDmaBufferSize > 0) {
        txn_p->txnState = TXN_TX;
    } else {
        txn_p->txnState = TXN_RX;
    }

    txn_p->txnStatus = TXN_STATUS_BUSY;

    I2CDrvService(instance);
}

static void I2CDrvExecuteTxn(const NATIVE_INT_TYPE instance, const NATIVE_INT_TYPE port)
{
    FW_ASSERT(instance >= 0 && instance < I2C_NUM, instance);
    FW_ASSERT(port >= 0 && port < I2C_MAX_NUM_PORTS, port);

    FW_ASSERT(portConfig[instance][port].valid == true, instance, port);

    FW_ASSERT(currentTxn[instance] == -1, currentTxn[instance]);

    I2CPortTransaction* txn_p = &portTransaction[instance][port];
    I2CPortConfig* config_p = &portConfig[instance][port];
    i2cBASE_t* i2cReg = i2cRegFramePtr[instance];

    FW_ASSERT(txn_p->txnState == TXN_TX ||
              txn_p->txnState == TXN_RX ||
              txn_p->txnState == TXN_NACK,
              txn_p->txnState);

    // Set this as the current transaction
    currentTxn[instance] = port;

    if (txn_p->txnState == TXN_NACK) {

        i2cReg->STR = I2C_NACK;
        i2cReg->MDR |= I2C_STOP_COND;

    } else {
        // Configure port settings
        i2cSetBaudrate(i2cReg, config_p->speed);
        i2cSetSlaveAdd(i2cReg, config_p->i2cAddr);

        // Clear the status register
        i2cReg->STR = I2C_SCD |
            I2C_NACKSNT;

        if (txn_p->txnState == TXN_TX) {

            // Transfer memory to DMA buffer
            memcpy(i2cTxDmaMem[instance],
                    txn_p->txDmaBufferPtr,
                    txn_p->txDmaBufferSize);

            // Start a Tx transaction

            // Setup DMA
            dmaReqAssign(i2cDrvDmaTxCh[instance], i2cDrvDmaTx[instance]);

            setupI2CDrvDmaTransmit(reinterpret_cast<U32>(&i2cReg->DXR),
                    i2cTxDmaMem[instance],
                    txn_p->txDmaBufferSize,
                    i2cDrvDmaTxCh[instance]);

            dmaSetChEnable(i2cDrvDmaTxCh[instance], DMA_HW);

            // Set the number of Tx bytes
            i2cReg->CNT = txn_p->txDmaBufferSize;

            // Setup I2C Mode
            i2cReg->MDR = I2C_MASTER      | // Master Mode
                I2C_TRANSMITTER | // Transmit
                I2C_7BIT_AMODE  | // 7 bit address
                I2C_8_BIT       | // 8 bit data mode
                I2C_RESET_OUT;    // Remain out of reset
        } else {
            // Start an Rx transaction

            // Setup DMA
            dmaReqAssign(i2cDrvDmaRxCh[instance], i2cDrvDmaRx[instance]);

            setupI2CDrvDmaReceive(reinterpret_cast<U32>(&i2cReg->DRR),
                    i2cRxDmaMem[instance],
                    txn_p->rxDmaBufferSize,
                    i2cDrvDmaRxCh[instance]);
            dmaSetChEnable(i2cDrvDmaRxCh[instance], DMA_HW);

            // Start to I2C transaction

            // Set the number of Rx bytes
            i2cReg->CNT = txn_p->rxDmaBufferSize;

            // Setup I2C Mode
            i2cReg->MDR = I2C_MASTER     | // Master Mode
                I2C_RECEIVER   | // Receive
                I2C_7BIT_AMODE | // 7 bit address
                I2C_8_BIT      | // 8 bit data mode
                I2C_RESET_OUT;   // Remain out of reset
        }

        // Set the Start bit flag to kickoff I2C Transaction
        i2cReg->MDR |= I2C_START_COND;

        // Enable Tx/Rx Dma triggers
        i2cReg->DMACR = 0x3;

        // Send a Stop bit after all bytes have been sent
        i2cReg->MDR |= I2C_STOP_COND;
    }
}

static void I2CDrvEndTxn(const NATIVE_INT_TYPE instance, const NATIVE_INT_TYPE port)
{
    FW_ASSERT(instance >= 0 && instance < I2C_NUM, instance);
    FW_ASSERT(port >= 0 && port < I2C_MAX_NUM_PORTS, port);

    I2CPortTransaction* txn_p = &portTransaction[instance][port];

    FW_ASSERT(txn_p->txnState == TXN_TX ||
              txn_p->txnState == TXN_RX ||
              txn_p->txnState == TXN_NACK,
              txn_p->txnState);

    i2cBASE_t* i2cReg = i2cRegFramePtr[instance];

    bool errDetected = false;

    // Clear the current transaction
    currentTxn[instance] = -1;

    if (txn_p->txnState == TXN_TX) {
        // We just performed a TX transaction

        // Check for errors
        U32 i2cStatus = i2cReg->STR;

        if (i2cStatus & I2C_NACK) {
            // NACK detected

            errDetected = true;

            txn_p->txnStatus = TXN_STATUS_TX_NACK;
            txn_p->txnState = TXN_NACK;
        }

        if (!errDetected) {
            // Setup for a Rx transaction
            if (txn_p->rxDmaBufferSize > 0) {
                txn_p->txnState = TXN_RX;
            } else {
                txn_p->txnState = TXN_DONE;
                txn_p->txnStatus = TXN_STATUS_OK;
            }
        }
    } else if (txn_p->txnState == TXN_RX) {
        // We just performed an Rx transaction

        // Check for errors
        U32 i2cStatus = i2cReg->STR;

        if (i2cStatus & I2C_NACK) {
            // NACK detected

            errDetected = true;

            txn_p->txnState = TXN_NACK;
            txn_p->txnStatus = TXN_STATUS_RX_NACK;
        }

        if (!errDetected) {
            // Copy memory from DMA memory back to transaction buffer. See note about i2c dma buffers
            U32 idx;
            for (idx = 0; idx < txn_p->rxDmaBufferSize; idx++) {
                txn_p->rxDmaBufferPtr[idx] = i2cRxDmaMem[instance][(idx * 4) + 3];
            }

            txn_p->txnState = TXN_DONE;
            txn_p->txnStatus = TXN_STATUS_OK;
        }
    } else {
        // Nack transaction

        // txnStatus already set during EndTxn for Tx/Rx
        txn_p->txnState = TXN_DONE;
    }
}

static bool I2CDrvTxnInProgress(const NATIVE_INT_TYPE instance)
{
    FW_ASSERT(instance >= 0 && instance < I2C_NUM, instance);

    return currentTxn[instance] != -1;
}

static bool I2CDrvIsTxnDone(const NATIVE_INT_TYPE instance)
{
    FW_ASSERT(instance >= 0 && instance < I2C_NUM, instance);

    FW_ASSERT(currentTxn[instance] != -1, instance, currentTxn[instance]);

    i2cBASE_t* i2cReg = i2cRegFramePtr[instance];

    NATIVE_INT_TYPE port = currentTxn[instance];

    I2CPortTransaction* txn_p = &portTransaction[instance][port];

    FW_ASSERT(txn_p->txnState == TXN_TX ||
              txn_p->txnState == TXN_RX ||
              txn_p->txnState == TXN_NACK,
              txn_p->txnState);

    if (txn_p->txnState == TXN_NACK) {
        // On Nack transaction, return status based on the BusBusy and Stop flag state 
        return (i2cReg->STR & I2C_BUSBUSY) == 0 &&
               (i2cReg->STR & I2C_SCD) == I2C_SCD;
    } else {

        if (i2cReg->STR & I2C_ARDY) {
            // Only check for NACK if we are ready to read registers.
            // Not entirely sure why this isn't necessary and breaks checking
            // for TX transaction completes
            if (i2cReg->STR & I2C_NACK) {
                return true;
            }
        }

        dmaChannel_t chan;

        // Criteria for trasaction complete is different depeding on the transaction state
        if (txn_p->txnState == TXN_TX) {
            // On Tx transaction, return status based on the BusBusy, Master mode state
            // and DMA channel complete

            chan = i2cDrvDmaTxCh[instance];
            return (i2cReg->STR & I2C_BUSBUSY) == 0 &&
                   (i2cReg->MDR & I2C_MASTER) == 0 &&
                   (dmaREG->BTCFLAG & (1 << chan)) == (1 << chan);
        } else {
            // On Rx transaction, return status based on the BusBusy, Stop flag state
            // and DMA channel complete

            chan = i2cDrvDmaTxCh[instance];
            return (i2cReg->STR & I2C_BUSBUSY) == 0 &&
                   (i2cReg->STR & I2C_SCD) == I2C_SCD &&
                   (dmaREG->BTCFLAG & (1 << chan)) == (1 << chan);
        }
    }
}

void I2CDrvService(const NATIVE_INT_TYPE instance)
{
    FW_ASSERT(instance >= 0 && instance < I2C_NUM, instance);

    NATIVE_UINT_TYPE port;

    if (I2CDrvTxnInProgress(instance)) {
        // We have started a transaction. Check if it has completed


        if (!I2CDrvIsTxnDone(instance)) {
            // Transaction active. Nothing to do
            return;
        }

        // Transaction has complete.
        I2CDrvEndTxn(instance, currentTxn[instance]);
    }

    // We either did not have a transaction in progress, or we just completed the old one
    // Check for a new transaction

    for (port = 0; port < I2C_MAX_NUM_PORTS; port++) {

        if (portTransaction[instance][port].txnState == TXN_TX ||
            portTransaction[instance][port].txnState == TXN_RX ||
            portTransaction[instance][port].txnState == TXN_NACK) {
            // We found an available transaction

            break;
        }
    }

    if (port < I2C_MAX_NUM_PORTS) {
        // Start the transaction
        I2CDrvExecuteTxn(instance, port);
    }
}

I2CTransactionStatus I2CDrvCheckTxn(const NATIVE_INT_TYPE instance, const NATIVE_INT_TYPE port, bool block)
{
    FW_ASSERT(instance >= 0 && instance < I2C_NUM, instance);
    FW_ASSERT(port >= 0 && port < I2C_MAX_NUM_PORTS, port);

    I2CPortTransaction* txn_p = &portTransaction[instance][port];

    FW_ASSERT(txn_p->txnState != TXN_IDLE);

    if (txn_p->txnState != TXN_DONE &&
        block) {

        // Service the drive until the transaction is done
        while (txn_p->txnState != TXN_DONE) {
            do {
                I2CDrvService(instance);
            } while (I2CDrvTxnInProgress(instance));
        }
    }

    return txn_p->txnStatus;
}

