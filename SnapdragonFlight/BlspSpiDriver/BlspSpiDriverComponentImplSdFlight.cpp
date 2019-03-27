// ====================================================================== 
// \title  BlspSpiDriverImpl.cpp
// \author tcanham
// \brief  cpp file for BlspSpiDriver component implementation class
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

#include <SnapdragonFlight/BlspSpiDriver/BlspSpiDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include "SnapdragonFlight/DspRelay/dsp_relay.h"
#include <Fw/Types/Assert.hpp>

namespace SnapdragonFlight {

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void BlspSpiDriverComponentImpl::SpiReadWrite_handler(
            const NATIVE_INT_TYPE portNum, Fw::Buffer &writeBuffer,
            Fw::Buffer &readBuffer) {

        if (this->m_fd == -1) {
            return;
        }

        // TODO: Max buffer size for SPI is 512, so should check for this.
        //       See dev_fs_lib_spi.h- DSPAL_SPI_TRANSMIT_BUFFER_LENGTH
        NATIVE_INT_TYPE stat = dsp_relay_spi_relay_read_write(this->m_fd,
                reinterpret_cast<unsigned char*>(writeBuffer.getdata()),
                writeBuffer.getsize(),
                reinterpret_cast<unsigned char*>(readBuffer.getdata()),
                readBuffer.getsize());

        if (stat != 0) {
            this->log_WARNING_HI_SPI_WriteError(this->m_device,stat);
        }

        this->m_bytes += readBuffer.getsize();
        this->tlmWrite_SPI_Bytes(this->m_bytes);

    }

    void BlspSpiDriverComponentImpl::open(NATIVE_INT_TYPE device, SpiFrequency clock) {

        this->m_device = device;

        NATIVE_INT_TYPE fd = dsp_relay_spi_relay_open(device);
        if (-1 == fd) {
            this->log_WARNING_HI_SPI_OpenError(device,fd);
            return;
        }

        this->m_fd = fd;

        NATIVE_INT_TYPE stat = dsp_relay_spi_relay_configure(fd,clock);
        if (stat != 0) {
            this->log_WARNING_HI_SPI_ConfigError(device,stat);
            return;
        }
    }

    BlspSpiDriverComponentImpl::~BlspSpiDriverComponentImpl(void) {
        dsp_relay_spi_relay_close(this->m_fd);
    }

} // end namespace SnapdragonFlight
