// ====================================================================== 
// \title  BlspI2CDriverImpl.hpp
// \author tcanham
// \brief  hpp file for BlspI2CDriver component implementation class
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

#ifndef BlspI2CDriver_HPP
#define BlspI2CDriver_HPP

#include "SnapdragonFlight/BlspI2CDriver/BlspI2CDriverComponentAc.hpp"

namespace SnapdragonFlight {

    /**
     * This was taken from the dspal_tester example
     *
     * Supported SPI frequency to talk to MPU9x50 slave device
     * MPU9x50 SPI interface supports upto 20MHz frequency. However 20MHz is not
     * reliable in our test and corrupted data is observed.
     */
    enum I2CFrequency
    {
       I2C_FREQUENCY_100KHZ = 100UL,
       I2C_FREQUENCY_400KHZ = 400UL,
       I2C_FREQUENCY_1MHZ = 1000UL,
    };

    class BlspI2CDriverComponentImpl: public BlspI2CDriverComponentBase {

        public:

            // ----------------------------------------------------------------------
            // Construction, initialization, and destruction
            // ----------------------------------------------------------------------

            //! Construct object BlspI2CDriver
            //!
            BlspI2CDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
                    const char * const compName /*!< The component name*/
#else
                    void
#endif
                    );

            //! Initialize object BlspI2CDriver
            //!
            void init(const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
            );

            //! Destroy object BlspI2CDriver
            //!
            ~BlspI2CDriverComponentImpl(void);

            //! Open device
            void open(NATIVE_INT_TYPE device, I2CFrequency clock);

        PRIVATE:

            // ----------------------------------------------------------------------
            // Handler implementations for user-defined typed input ports
            // ----------------------------------------------------------------------

            //! Handler implementation for I2CReadWrite
            //!
            void I2CReadWrite_handler(const NATIVE_INT_TYPE portNum, /*!< The port number*/
                                      Fw::Buffer &WriteBuffer,
                                      Fw::Buffer &readBuffer);

            //! Handler implementation for I2CConfig
            //!
            void I2CConfig_handler(const NATIVE_INT_TYPE portNum, /*!< The port number*/
                                   U32 busSpeed,
                                   U32 slaveAddr,
                                   U32 timeout);

            NATIVE_INT_TYPE m_fd;
            NATIVE_INT_TYPE m_device;
            U32 m_addr;
            U32 m_readBytes;
            U32 m_writeBytes;

    };

} // end namespace Snapdragon

#endif
