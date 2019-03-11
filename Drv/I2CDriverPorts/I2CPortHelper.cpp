/*
 * I2CPortHelper.cpp
 *
 *  Created on: Monday, 11 March 2019
 *  Author:     Gerik Kubiak
 *
 */

#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Serializable.hpp>
#include <Fw/Buffer/BufferSerializableAc.hpp>
#include <Fw/Types/Assert.hpp>

#include <Drv/I2CDriverPorts/I2CPortHelper.hpp>


namespace Drv {

  void I2CWriteSingleRegisterHelper(U8 reg, U8 reg_value,
                                    Fw::Buffer& i2cWriteBuffer,
                                    Fw::Buffer& i2cReadBuffer)
  {
      U32 writeBufferLen = i2cWriteBuffer.getsize();

      // Assert that we can store the 2 bytes for the register and the
      // value
      FW_ASSERT(writeBufferLen >= 2, writeBufferLen);

      U8* writeBuffer = reinterpret_cast<U8*>(i2cWriteBuffer.getdata());

      writeBuffer[0] = reg;
      writeBuffer[1] = reg_value;

      i2cWriteBuffer.setsize(2);

      i2cReadBuffer.setsize(0);
  }


  void I2CReadRegisterHelper(U8 reg, U32 read_bytes,
                             Fw::Buffer& i2cWriteBuffer,
                             Fw::Buffer& i2cReadBuffer)
  {
      U32 writeBufferLen = i2cWriteBuffer.getsize();

      // Assert that we can store the 1 byte for the register
      FW_ASSERT(writeBufferLen >= 1, writeBufferLen);

      // Assert that we can read all bytes requested
      FW_ASSERT(i2cReadBuffer.getsize() >= read_bytes);

      U8* writeBuffer = reinterpret_cast<U8*>(i2cWriteBuffer.getdata());

      writeBuffer[0] = reg;

      i2cReadBuffer.setsize(read_bytes);
  }

}

