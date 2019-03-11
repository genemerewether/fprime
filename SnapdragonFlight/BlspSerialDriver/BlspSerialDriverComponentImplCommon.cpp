/*
 * BlspSerialDriverComponentImplCommon.cpp
 *
 *  Created on: Nov 29, 2016
 *      Author: tcanham
 */

#include <SnapdragonFlight/BlspSerialDriver/BlspSerialDriverComponentImpl.hpp>

namespace SnapdragonFlight {


  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

    BlspSerialDriverComponentImpl ::
  #if FW_OBJECT_NAMES == 1
      BlspSerialDriverComponentImpl(
          const char *const compName
      ) :
        BlspSerialDriverComponentBase(compName)
  #else
      BlspSerialDriverImpl(void) :
          BlspSerialDriverComponentBase()
  #endif
      ,m_fd(-1),m_device(-1),m_quitReadThread(false)
    {
        // initialize buffer set
        for (NATIVE_INT_TYPE entry = 0; entry < MAX_NUM_BUFFERS; entry++) {
            this->m_buffSet[entry].available = false;
        }

    }

    void BlspSerialDriverComponentImpl ::
      init(
          const NATIVE_INT_TYPE instance
      )
    {
      BlspSerialDriverComponentBase::init(instance);
    }

    void BlspSerialDriverComponentImpl ::
      readBufferSend_handler(
          const NATIVE_INT_TYPE portNum,
          Fw::Buffer &Buffer
      )
    {
        this->m_readBuffMutex.lock();
        bool found = false;

        // search for open entry
        for (NATIVE_UINT_TYPE entry = 0; entry < MAX_NUM_BUFFERS; entry++) {
            // Look for slots to fill. "Available" is from
            // the perspective of the driver thread looking for
            // buffers to fill, so add the buffer and make it available.
            if (not this->m_buffSet[entry].available) {
                this->m_buffSet[entry].readBuffer = Buffer;
                this->m_buffSet[entry].available = true;
                found = true;
                break;
            }
        }
        this->m_readBuffMutex.unLock();
        FW_ASSERT(found,Buffer.getbufferID(),Buffer.getmanagerID());

    }

}
