#ifndef _ACTIVE_FILE_LOGGER_PACKET_HPP_
#define _ACTIVE_FILE_LOGGER_PACKET_HPP_

#include <Fw/Types/Serializable.hpp>

namespace Svc {

  class ActiveFileLoggerPacket : public Fw::SerializeBufferBase {
  public:

#ifdef BUILD_UT
    void operator=(const Fw::SerializeBufferBase& other) {
        this->resetSer();
        this->serialize(other.getBuffAddr(),other.getBuffLength(),true);
    }

    ActiveFileLoggerPacket(const Fw::SerializeBufferBase& other) : Fw::SerializeBufferBase() {
        this->resetSer();
        this->serialize(other.getBuffAddr(),other.getBuffLength(),true);
    }

    ActiveFileLoggerPacket(const ActiveFileLoggerPacket& other) : Fw::SerializeBufferBase() {
        this->resetSer();
        this->serialize(other.getBuffAddr(),other.getBuffLength(),true);
    }

    ActiveFileLoggerPacket()  : Fw::SerializeBufferBase() {

    }
#endif

    enum {
        ACTIVE_FILE_LOGGER_BUFFER_SIZE = 256,
        SERIALIZED_SIZE = ACTIVE_FILE_LOGGER_BUFFER_SIZE + sizeof(FwEnumStoreType)
    };

    NATIVE_UINT_TYPE getBuffCapacity (void) const {
      return ACTIVE_FILE_LOGGER_BUFFER_SIZE;
    }

    U8 * getBuffAddr (void) {
      return m_buffer;
    }

    const U8 * getBuffAddr (void) const {
      return m_buffer;
    }

  private:
    U8 m_buffer [ACTIVE_FILE_LOGGER_BUFFER_SIZE];

  };

}

#endif //_ACTIVE_FILE_LOGGER_PACKET_HPP_
