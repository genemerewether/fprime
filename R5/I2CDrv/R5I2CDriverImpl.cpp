// ====================================================================== 
// \title  R5I2CDriverImpl.cpp
// \author kubiak
// \brief  cpp file for R5I2CDriver component implementation class
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


#include <R5/I2CDrv/R5I2CDriverImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"

#include <R5/I2CDrv/I2CDrv.hpp>
#include <Fw/Types/MemAllocator.hpp>
#include <Fw/Types/Assert.hpp>

namespace R5 {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  R5I2CDriverComponentImpl ::
#if FW_OBJECT_NAMES == 1
    R5I2CDriverComponentImpl(
        const char *const compName
    ) :
      R5I2CDriverComponentBase(compName)
#else
    R5I2CDriverImpl(void)
#endif
  {

  }

  void R5I2CDriverComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    R5I2CDriverComponentBase::init(instance);

    this->m_instance = instance;
  }

  R5I2CDriverComponentImpl ::
    ~R5I2CDriverComponentImpl(void)
  {

  }

  void R5I2CDriverComponentImpl ::
    initDriver(
        NATIVE_UINT_TYPE rxSize,
        NATIVE_UINT_TYPE txSize,
        Fw::MemAllocator& allocator)
  {
    U8* rxDmaMem = static_cast<U8*>(allocator.allocate(0, rxSize));
    U8* txDmaMem = static_cast<U8*>(allocator.allocate(0, txSize));

    FW_ASSERT(rxDmaMem != NULL, reinterpret_cast<POINTER_CAST>(rxDmaMem));
    FW_ASSERT(txDmaMem != NULL, reinterpret_cast<POINTER_CAST>(txDmaMem));

    I2CDrvInit(this->m_instance,
               rxDmaMem, rxSize,
               txDmaMem, txSize);
  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void R5I2CDriverComponentImpl ::
    config_handler(
        const NATIVE_INT_TYPE portNum,
        U32 busSpeed,
        U32 slaveAddr,
        U32 timeout
    )
  {
     I2CBusSpeed speed;

    switch (busSpeed) {
        case 100000:
            speed = I2C_BUS_SPEED_100K;
            break;
        case 400000:
            speed = I2C_BUS_SPEED_400K;
            break;
        default:
            FW_ASSERT(false, busSpeed);
            speed = I2C_BUS_SPEED_100K;
            break;
    }

    I2CDrvConfigPort(this->m_instance,
                     portNum,
                     speed,
                     timeout,
                     slaveAddr);
  }

  void R5I2CDriverComponentImpl ::
    writeRead_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &writeBuffer,
        Fw::Buffer &readBuffer
    )
  {
    I2CDrvAddTxn(this->m_instance,
                 portNum,
                 reinterpret_cast<U8*>(writeBuffer.getdata()),
                 writeBuffer.getsize(),
                 reinterpret_cast<U8*>(readBuffer.getdata()),
                 readBuffer.getsize());
  }

  Drv::I2CStatus R5I2CDriverComponentImpl ::
    status_handler(
        const NATIVE_INT_TYPE portNum,
        bool shouldBlock
    )
  {
    I2CTransactionStatus txnStat;
    txnStat = I2CDrvCheckTxn(this->m_instance,
                             portNum,
                             shouldBlock);

    Drv::I2CStatus stat;

    switch (txnStat) {
        case TXN_STATUS_BUSY:
            stat = Drv::I2C_STATUS_BUSY;
            break;
        case TXN_STATUS_OK:
            stat = Drv::I2C_STATUS_OK;
            break;
        case TXN_STATUS_TX_NACK:
            stat = Drv::I2C_STATUS_TX_NACK;
            break;
        case TXN_STATUS_RX_NACK:
            stat = Drv::I2C_STATUS_RX_NACK;
            break;
        default:
            FW_ASSERT(false, txnStat);
            stat = Drv::I2C_STATUS_OK;
            break;
    }

    return stat;
  }

  
  void R5I2CDriverComponentImpl ::
    service_handler(
      NATIVE_INT_TYPE portNum, /*!< The port number*/
      NATIVE_UINT_TYPE context /*!< The call order*/
    )
  {
    I2CDrvService(this->m_instance);
  }

} // end namespace R5
