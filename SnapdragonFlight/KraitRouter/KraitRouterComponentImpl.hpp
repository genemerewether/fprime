// ======================================================================
// \title  KraitRouterImpl.hpp
// \author vagrant
// \brief  hpp file for KraitRouter component implementation class
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

#ifndef KraitRouter_HPP
#define KraitRouter_HPP

#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImplCfg.hpp>

#include "SnapdragonFlight/KraitRouter/KraitRouterComponentAc.hpp"

#include "Os/Mutex.hpp"

namespace SnapdragonFlight {

    class KraitRouterComponentImpl :
      public KraitRouterComponentBase
    {

      public:

        // ----------------------------------------------------------------------
        // Construction, initialization, and destruction
        // ----------------------------------------------------------------------

        //! Construct object KraitRouter
        //!
        KraitRouterComponentImpl(
  #if FW_OBJECT_NAMES == 1
            const char *const compName /*!< The component name*/
  #else
            void
  #endif
        );

        //! Initialize object KraitRouter
        //!
        void init(
            const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
        );

        //! Destroy object KraitRouter
        //!
        ~KraitRouterComponentImpl(void);

        int buffRead(unsigned int* port, unsigned char* buff, int buffLen, int* bytes);

        int portRead(unsigned int* port, unsigned char* buff, int buffLen, int* bytes);

        // TODO(mereweth) - split into portWrite and buffWrite (with return port)?
        int write(unsigned int port, const unsigned char* buff, int buffLen);

        bool m_quit;

      PRIVATE:

        // ----------------------------------------------------------------------
        // Handler implementations for user-defined typed input ports
        // ----------------------------------------------------------------------

        //! Handler implementation for Sched
        //!
        void Sched_handler(
            const NATIVE_INT_TYPE portNum, /*!< The port number*/
            NATIVE_UINT_TYPE context /*!< The call order*/
        );

      PRIVATE:

        // ----------------------------------------------------------------------
        // Handler implementations for user-defined serial input ports
        // ----------------------------------------------------------------------

        //! Handler implementation for HexPortsIn
        //!
        void HexPortsIn_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          Fw::SerializeBufferBase &Buffer /*!< The serialization buffer*/
        );

        // TODO(mereweth) - alloc on heap to allow setting size from Krait?
        // and/or allow setting size in Topology based on serialized size of each port?
        struct PortBufferEntry {
            unsigned char buff[KR_PORT_BUFF_SIZE];
            //Fw::ExternalSerializeBuffer portBuffObj;
            unsigned int portNum;
            unsigned int buffLen;
            volatile bool available;
        } m_recvPortBuffers[KR_NUM_RECV_PORT_BUFFS];

        volatile unsigned int m_recvPortBuffInsert;
        volatile unsigned int m_recvPortBuffRemove;

        PortBufferEntry m_sendPortBuffers[KR_NUM_SEND_PORT_BUFFS];

        volatile unsigned int m_sendPortBuffInsert;
        volatile unsigned int m_sendPortBuffRemove;

        volatile bool m_initialized;
    };

} // end namespace SnapdragonFlight

#endif
