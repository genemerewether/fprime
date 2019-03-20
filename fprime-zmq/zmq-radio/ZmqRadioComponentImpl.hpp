// ====================================================================== 
// \title  ZmqRadioComponentImpl.hpp
// \author dkooi
// \brief  hpp file for ZmqRadio implementation class.
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


#ifndef ZMQRADIOIMPL_HPP_
#define ZMQRADIOIMPL_HPP_

#include <string.h>
#include <stdlib.h>

#include <Os/Mutex.hpp>
#include <Fw/Com/ComPacket.hpp>
#include <Fw/Types/EightyCharString.hpp>

#include <fprime-zmq/zmq-radio/ZmqRadioComponentAc.hpp>
#include <fprime-zmq/zmq-radio/ZmqRadioCfg.hpp>
#include <fprime-zmq/zmq/include/zmq.h>

#include <Os/Task.hpp>

namespace Zmq{

    class ZmqRadioComponentImpl : public ZmqRadioComponentBase{
	public:
#if FW_OBJECT_NAMES == 1
	    ZmqRadioComponentImpl(const char* name); 
#else
	    ZmqRadioComponentImpl(void);
#endif

	    void init(NATIVE_INT_TYPE queueDepth, NATIVE_INT_TYPE instance);

	    /* Attempt a connection to <port> using socket identity <zmqId> 
	     * */
	    void open(const char* hostname, U32 port, const char* zmqId); 

	    virtual ~ZmqRadioComponentImpl();
	
	PROTECTED:
	PRIVATE:

		// Return values for zmqSocketRead
		enum {ZMQ_SOCKET_READ_EAGAIN = -1,
			  ZMQ_SOCKET_READ_ERROR  = -2};


		/* Setup zmq context and attempt server connection 
		 * */
	    void connect(void);

	    /* Send a registration call to server.
	     * Returns 0  if successful.
	     * Returns -1 if unsuccessful. 
	     * */
	    NATIVE_INT_TYPE registerToServer(void);

	    void preamble(void);
	    void finalizer(void);


	    //! Handler for input port downlinkPort
	    //
	    void downlinkPort_handler(
		NATIVE_INT_TYPE portNum, /*!< The port number*/
		Fw::ComBuffer &data, /*!< Buffer containing packet data*/
		U32 context /*!< Call context value; meaning chosen by user*/
	    ); 


	    //! Handler for input port fileDownlinkBufferSendIn
	    //
	    void fileDownlinkBufferSendIn_handler(
		NATIVE_INT_TYPE portNum, /*!< The port number*/
		Fw::Buffer fwBuffer 
	    );

	    /* Scheduled reconnect input */
	    void reconnect_handler(NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context );

	    /* Setup the subscription task */
	    void startSubscriptionTask(I32 priority);
	    static void subscriptionTaskRunnable(void* comp);

	    /* FPrime ZMQ Wrapper functions */
	    NATIVE_INT_TYPE zmqSocketWriteComBuffer(void* zmqSocket, Fw::ComBuffer &data);
	    NATIVE_INT_TYPE zmqSocketWriteFilePacket(void* zmqSocket, Fw::Buffer &buffer);
		void zmqSocketWrite(void* zmqSocket, zmq_msg_t* fPrimePacket);
		NATIVE_INT_TYPE zmqSocketRead(void* zmqSocket, U8* buf, NATIVE_INT_TYPE size);


	    /* ZMQ Components */
	    void* m_context; //!< zmq context
	    void* m_pubSocket; //!< zmq socket for outbound telemetry,events, and files
	    void* m_cmdSocket; //!< zmq socket for server registration

	    /* Network Info */
	    char m_zmqId[ZMQ_RADIO_ENDPOINT_NAME_SIZE]; //!< zmq socket identity 
	    char m_hostname[ZMQ_RADIO_ENDPOINT_NAME_SIZE];
	    U32  m_serverCmdPort;
	    U32  m_serverPubPort;
	    U32  m_serverSubPort;

	    /* Telemetry */
	    U32 m_packetsSent;
	    U32 m_packetsRecv;
	    U32 m_numDisconnectRetries;
	    U32 m_numConnects;
	    U32 m_numDisconnects;
	    

	    /* Private Internal State class*/
	    class State{

			public:
				/* Component States */
				static const U8 ZMQ_RADIO_DISCONNECTED_STATE   = 0x01;
				static const U8 ZMQ_RADIO_CONNECTED_STATE      = 0x02;

				State(ZmqRadioComponentImpl* parent); 
				U8 get();                     // Return current state
				void transitionConnected();    // Transition to connected state
				void transitionDisconnected(); // Transition to disconnected state

			private:
				U8 state;
				ZmqRadioComponentImpl* m_parent;

        };

        State m_state; //!< This component's state
        Os::Task subscriptionTask; //!< Listen for incoming packets


    };


} // namespace Zmq 




#endif // ZMQRADIOIMPL_HPP_
