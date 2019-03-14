// ======================================================================
// \title  I2CDrv.h
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

#ifndef I2CDRV_HPP_
#define I2CDRV_HPP_

#include <Fw/Types/BasicTypes.hpp>

#ifdef __cplusplus
extern "C" {
#endif


    enum I2CBusSpeed {
        I2C_BUS_SPEED_100K = 100,
        I2C_BUS_SPEED_400K = 400
    };

    enum I2CTransactionStatus {
        TXN_STATUS_BUSY,
        TXN_STATUS_OK,
        TXN_STATUS_TX_NACK,
        TXN_STATUS_RX_NACK,
    };


    // Initialize the I2C device for a specific peripheral instance
    void I2CDrvInit(const NATIVE_INT_TYPE instance,
                    U8* rxDmaMem, const U32 rxDmaMemSize,
                    U8* txDmaMem, const U32 txDmaMemSize);

    // Configure the per-F' port I2C settings.
    // Must be called before adding a transaction
    void I2CDrvConfigPort(const NATIVE_INT_TYPE instance,
                          const NATIVE_INT_TYPE port,
                          const I2CBusSpeed speed,
                          const U32 timeoutUs,
                          const U8 i2cAddr);

    // Add a WriteRead transaction to the I2C driver.
    // If either the Write or Read buffer has a size of 0
    // the driver will skip that portion of the transaction.
    // If both Read and Write sizes are 0, the driver will
    // start a Read transaction to check for an Ack from the
    // device.
    void I2CDrvAddTxn(const NATIVE_INT_TYPE instance,
                  const NATIVE_INT_TYPE port,
                  const U8* txDmaBufferPtr,
                  U32 txDmaBufferSize,
                  U8* rxDmaBufferPtr,
                  U32 rxDmaBufferSize);

    // Service the I2C driver. Advances the state of transactions
    // in the driver.
    // Must be called at a regular rate.
    void I2CDrvService(const NATIVE_INT_TYPE instance);

    // Check the status of a transaction. If block is true,
    // don't return until that transaction is complete
    I2CTransactionStatus I2CDrvCheckTxn(const NATIVE_INT_TYPE instance,
                                     const NATIVE_INT_TYPE port,
                                     bool block);

#ifdef __cplusplus
}  // extern "C"
#endif  // __cplusplus


#endif
