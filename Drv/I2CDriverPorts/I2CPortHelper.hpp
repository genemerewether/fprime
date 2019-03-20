/*
 * I2CPortHelper.hpp
 *
 *  Created on: Monday, 11 March 2019
 *  Author:     Gerik Kubiak
 *
 */
#ifndef DRV_I2C_PORT_HELPER_HPP_
#define DRV_I2C_PORT_HELPER_HPP_

#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Serializable.hpp>
#include <Fw/Buffer/BufferSerializableAc.hpp>

namespace Drv {

    void I2CWriteSingleRegisterHelper(U8 reg, U8 reg_value,
                                      Fw::Buffer& i2cWriteBuffer,
                                      Fw::Buffer& i2cReadBuffer);

    void I2CReadRegisterHelper(U8 reg, U32 read_bytes,
                               Fw::Buffer& i2cWriteBuffer,
                               Fw::Buffer& i2cReadBuffer);
}

#endif
