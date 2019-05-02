// ====================================================================== 
// \title  BlspI2CDriverImpl.cpp
// \author tcanham
// \brief  cpp file for BlspI2CDriver component implementation class
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

#include <SnapdragonFlight/BlspI2CDriver/BlspI2CDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include "SnapdragonFlight/DspRelay/dsp_relay.h"
#include <Fw/Types/Assert.hpp>

namespace SnapdragonFlight {

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void BlspI2CDriverComponentImpl ::
      I2CConfig_handler(
          const NATIVE_INT_TYPE portNum,
          U32 busSpeed,
          U32 slaveAddr,
          U32 timeout
      )
    {
        NATIVE_INT_TYPE stat = dsp_relay_i2c_relay_configure(this->m_fd, busSpeed, slaveAddr, timeout);
        if (stat != 0) {
            this->log_WARNING_HI_I2C_ConfigError(this->m_device,stat);
            return;
        }
        this->m_addr = slaveAddr;
    }
  
    void BlspI2CDriverComponentImpl::I2CReadWrite_handler(
            const NATIVE_INT_TYPE portNum, Fw::Buffer &writeBuffer,
            Fw::Buffer &readBuffer) {

        if (this->m_fd == -1) {
            return;
        }

        NATIVE_INT_TYPE stat = dsp_relay_i2c_relay_read_write(this->m_fd,
                reinterpret_cast<unsigned char*>(writeBuffer.getdata()),
                writeBuffer.getsize(),
                reinterpret_cast<unsigned char*>(readBuffer.getdata()),
                readBuffer.getsize());

        if (stat != 0) {
            this->log_WARNING_HI_I2C_WriteError(this->m_device,
						this->m_addr,
						stat,
						0, writeBuffer.getsize());
        }
	
	this->m_readBytes += readBuffer.getsize();
	this->m_writeBytes += writeBuffer.getsize();
       
        this->tlmWrite_I2C_ReadBytes(this->m_readBytes);
        this->tlmWrite_I2C_WriteBytes(this->m_writeBytes);
    }

    void BlspI2CDriverComponentImpl::open(NATIVE_INT_TYPE device, I2CFrequency clock) {

        this->m_device = device;

        NATIVE_INT_TYPE fd = dsp_relay_i2c_relay_open(device);
        if (-1 == fd) {
            this->log_WARNING_HI_I2C_OpenError(device,fd);
            return;
        }

        this->m_fd = fd;
    }

    BlspI2CDriverComponentImpl::~BlspI2CDriverComponentImpl(void) {
        dsp_relay_i2c_relay_close(this->m_fd);
    }

} // end namespace SnapdragonFlight
