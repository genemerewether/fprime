// ====================================================================== 
// \title  BlspSpiDriverImpl.hpp
// \author tcanham
// \brief  hpp file for BlspSpiDriver component implementation class
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

#ifndef BlspSpiDriver_HPP
#define BlspSpiDriver_HPP

#include "SnapdragonFlight/BlspSpiDriver/BlspSpiDriverComponentAc.hpp"

namespace SnapdragonFlight {

    /**
     * This was taken from the dspal_tester example
     *
     * Supported SPI frequency to talk to MPU9x50 slave device
     * MPU9x50 SPI interface supports upto 20MHz frequency. However 20MHz is not
     * reliable in our test and corrupted data is observed.
     */
    enum SpiFrequency
    {
       SPI_FREQUENCY_1MHZ = 1000000UL,
       SPI_FREQUENCY_5MHZ = 5000000UL,
       SPI_FREQUENCY_10MHZ = 10000000UL,
       SPI_FREQUENCY_15MHZ = 15000000UL,
       SPI_FREQUENCY_20MHZ = 20000000UL,
    };

    class BlspSpiDriverComponentImpl: public BlspSpiDriverComponentBase {

        public:

            // ----------------------------------------------------------------------
            // Construction, initialization, and destruction
            // ----------------------------------------------------------------------

            //! Construct object BlspSpiDriver
            //!
            BlspSpiDriverComponentImpl(
#if FW_OBJECT_NAMES == 1
                    const char * const compName /*!< The component name*/
#else
                    void
#endif
                    );

            //! Initialize object BlspSpiDriver
            //!
            void init(const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
            );

            //! Destroy object BlspSpiDriver
            //!
            ~BlspSpiDriverComponentImpl(void);

            //! Open device
            void open(NATIVE_INT_TYPE device, SpiFrequency clock);

        PRIVATE:

            // ----------------------------------------------------------------------
            // Handler implementations for user-defined typed input ports
            // ----------------------------------------------------------------------

            //! Handler implementation for SpiReadWrite
            //!
            void SpiReadWrite_handler(const NATIVE_INT_TYPE portNum, /*!< The port number*/
            Fw::Buffer &WriteBuffer, Fw::Buffer &readBuffer);

            NATIVE_INT_TYPE m_fd;
            NATIVE_INT_TYPE m_device;
            U32 m_bytes;

    };

} // end namespace Snapdragon

#endif
