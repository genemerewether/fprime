// ====================================================================== 
// \title  ZmqRadioComponentImpl.cpp
// \author dkooi
// \brief  cpp file for ZmqRadio implementation class.
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


#include <Fw/Types/BasicTypes.hpp>
#include <fprime-zmq/zmq-radio/ZmqRadioComponentImpl.hpp>

//#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__)
#define DEBUG_PRINT(x,...)

namespace Zmq{



	/* Helper Functions */
	namespace { 

		bool zmqError(const char* from) {
			switch (zmq_errno()) {
				case EAGAIN:
				    //printf("%s: ZMQ EAGAIN\n", from);
				    return true;
				case EFSM:
				     DEBUG_PRINT("%s: ZMQ EFSM", from);
				     return true;
				case ETERM:
					DEBUG_PRINT("%s: ZMQ terminate\n",from);
					return true;
				case ENOTSOCK:
					DEBUG_PRINT("%s: ZMQ ENOTSOCK\n",from);
					return true;
				case EINTR:
					DEBUG_PRINT("%s: ZMQ EINTR\n",from);
					return false;
				case EFAULT:
					DEBUG_PRINT("%s: ZMQ EFAULT\n",from);
					return false;
				case ENOMEM:
					DEBUG_PRINT("%s: ZMQ ENOMEM\n", from);
					return false;
				default:
					DEBUG_PRINT("%s: ZMQ error: %s\n",from,zmq_strerror(zmq_errno()));
					return true;
			}
		}

	} // namespace



#if FW_OBJECT_NAMES == 1
	ZmqRadioComponentImpl :: ZmqRadioComponentImpl(const char* name): ZmqRadioComponentBase(name)
#else
	ZmqRadioComponentImpl :: ZmqRadioComponentImpl(void)
#endif
	// ZMQ Components
	,m_context(0)
	,m_pubSocket(0)
	,m_cmdSocket(0)

	// Telemetry
	,m_packetsSent(0)
	,m_packetsRecv(0)
	,m_numDisconnectRetries(0)
	,m_numConnects(0)
	,m_numDisconnects(0)
	,m_state(this)
	{
	}

	void ZmqRadioComponentImpl::init(NATIVE_INT_TYPE queueDepth, NATIVE_INT_TYPE instance){
	    ZmqRadioComponentBase::init(queueDepth, instance);
	}

	void ZmqRadioComponentImpl::preamble(void){ 
	    DEBUG_PRINT("Preamble\n");
	    this->connect();
	    this->startSubscriptionTask(100);
	}

	void ZmqRadioComponentImpl::finalizer(void){
	    // Close Zmq
	    DEBUG_PRINT("Finalizer\n");

	    // The transition to disconnect destroys all zmq resources.
	    // If the zmq resources are destroyed here the subscription thread
	    // might call transitionDisconnected() and attempt to destroy
	    // an already destroyed zmq resource.
		m_state.transitionDisconnected();
	}

	ZmqRadioComponentImpl::~ZmqRadioComponentImpl(void){
	    // Object destruction
	    DEBUG_PRINT("Destruct\n");
	}


	void ZmqRadioComponentImpl::open(const char* hostname, U32 port, const char* zmqId){
		DEBUG_PRINT("Saving network information\n");

	    strncpy(this->m_hostname, hostname, strlen(hostname)); // Save hostname
	    this->m_hostname[strlen(hostname)] = 0; // Null terminate

	    strncpy(this->m_zmqId, zmqId, strlen(zmqId));  // Save id for socket identification	
	    this->m_zmqId[strlen(zmqId)] = 0; // Null terminate

	    this->m_serverCmdPort = port; // Save server command port 
	}

	void ZmqRadioComponentImpl::connect(void){
	    int rc = 0; // Return code

	    DEBUG_PRINT("Connecting\n");
	    // Setup context
	    this->m_context   = zmq_ctx_new();
	    if(not this->m_context){
	    	zmqError("ZmqRadioComponentImpl::connect Error creating context.");
	    	Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
	    	this->log_WARNING_HI_ZR_ContextError(errArg);
	    	this->m_state.transitionDisconnected();
	    	return;
	    }

	    // Create sockets and set options 

	    /* Cmd Socket */
	    this->m_cmdSocket = zmq_socket(this->m_context, ZMQ_DEALER);
	    if(not this->m_cmdSocket){ 
			zmqError("ZmqRadioComponentImpl::connect Error creating cmd socket");
			Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
			this->log_WARNING_HI_ZR_SocketError(errArg);
			this->m_state.transitionDisconnected();
			return;
	    }	
	    zmq_setsockopt(this->m_cmdSocket, ZMQ_IDENTITY, &this->m_zmqId, strlen(this->m_zmqId));
	    zmq_setsockopt(this->m_cmdSocket, ZMQ_LINGER, &ZMQ_RADIO_LINGER, sizeof(ZMQ_RADIO_LINGER));
	    zmq_setsockopt(this->m_cmdSocket, ZMQ_RCVTIMEO, &ZMQ_RADIO_RCVTIMEO, sizeof(ZMQ_RADIO_RCVTIMEO));
	    zmq_setsockopt(this->m_cmdSocket, ZMQ_SNDTIMEO, &ZMQ_RADIO_SNDTIMEO, sizeof(ZMQ_RADIO_SNDTIMEO));
	    
	    /* Pub Socket */
	    this->m_pubSocket = zmq_socket(this->m_context, ZMQ_DEALER);
	    if(not this->m_pubSocket){ 
			zmqError("ZmqRadioComponentImpl::connect Error creating pub socket");
			Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
			this->log_WARNING_HI_ZR_SocketError(errArg);
			this->m_state.transitionDisconnected();
			return;
	    }
	    zmq_setsockopt(this->m_pubSocket, ZMQ_IDENTITY, &this->m_zmqId, strlen(this->m_zmqId));
	    zmq_setsockopt(this->m_pubSocket, ZMQ_LINGER, &ZMQ_RADIO_LINGER, sizeof(ZMQ_RADIO_LINGER));
	    zmq_setsockopt(this->m_pubSocket, ZMQ_SNDHWM, &ZMQ_RADIO_SNDHWM, sizeof(ZMQ_RADIO_SNDHWM));
	    zmq_setsockopt(this->m_pubSocket, ZMQ_RCVTIMEO, &ZMQ_RADIO_RCVTIMEO, sizeof(ZMQ_RADIO_RCVTIMEO));
	    zmq_setsockopt(this->m_pubSocket, ZMQ_SNDTIMEO, &ZMQ_RADIO_SNDTIMEO, sizeof(ZMQ_RADIO_SNDTIMEO));


	    // Attempt registration
	    rc = this->registerToServer();
	    if(rc == -1){
	    	this->m_state.transitionDisconnected();
	    }else{
	    	this->m_state.transitionConnected();
	    }

	    return;
	}

	NATIVE_INT_TYPE ZmqRadioComponentImpl::registerToServer(){
		DEBUG_PRINT("Registering to server\n");

		NATIVE_INT_TYPE rc = 0; //Return code
		char endpoint[ZMQ_RADIO_ENDPOINT_NAME_SIZE];

		// Connect cmd socket to server
		(void)snprintf(endpoint,ZMQ_RADIO_ENDPOINT_NAME_SIZE,"tcp://%s:%d",this->m_hostname, this->m_serverCmdPort);
		zmq_connect(this->m_cmdSocket, endpoint);

		DEBUG_PRINT("SERVER: %s\n", endpoint);

		// Send server a registration command
		const U8 regMsgSize = ZMQ_RADIO_REG_MSG_SIZE;
		const char *reg_msgArr[regMsgSize] = {"REG", "FLIGHT", "ZMQ"};

		U8 i;
		for(i = 0; i < regMsgSize; i++){
			const char *msg = reg_msgArr[i];
			size_t len = strlen(msg);

			zmq_msg_t z_msg; // Declare zmq msg struct
			int rc = zmq_msg_init_size(&z_msg, len); // Allocate msg_t 
			FW_ASSERT(rc == 0);

			memcpy(zmq_msg_data (&z_msg), msg, len); // Copy part into msg
			rc = zmq_msg_send(&z_msg, this->m_cmdSocket, ((i == regMsgSize-1) ? 0 : ZMQ_SNDMORE) ); // Set SNDMORE flag to zero if last message
		
			if (-1 == rc) {
				zmqError("ZmqRadioComponentImpl::registerToServer Error sending registration message."); 
				Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
				this->log_WARNING_HI_ZR_SendError(errArg);
				return -1;
			}

			zmq_msg_close(&z_msg);
		}	     
		DEBUG_PRINT("Sent reg msg\n");


		// Receive server response
		const U8 regRespSize = ZMQ_RADIO_REG_RESP_MSG_SIZE; // How many response messages to expect
		U32 regStatus = 0;
		for(i = 0; i < regRespSize; i++){
			zmq_msg_t msg;
			zmq_msg_init(&msg);
			int size = zmq_msg_recv(&msg, this->m_cmdSocket, 0);
			if(size == -1){
				zmqError("ZmqRadioComponentImpl::registerToServer Error receiving server registration response.");
				Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
				this->log_WARNING_HI_ZR_ReceiveError(errArg);
				return -1;
			}

			// Receive the various msg parts
			switch(i){
				case 0:
					memcpy(&regStatus, zmq_msg_data(&msg), size);
					DEBUG_PRINT("status: %d\n", regStatus);
					break;
				case 1:
					memcpy(&this->m_serverPubPort, zmq_msg_data(&msg), size);
					DEBUG_PRINT("serverPubPort: %d\n", this->m_serverPubPort);
					break;
				case 2:
					memcpy(&this->m_serverSubPort, zmq_msg_data(&msg), size);
					DEBUG_PRINT("serverSubPort: %d\n", this->m_serverSubPort);
					break;

				default:
		            FW_ASSERT(0);
				
			}
			zmq_msg_close(&msg);
		}

		// Check registration status
		if(regStatus == 0){
			Fw::LogStringArg errArg("Registration Error: Registration status 0\n");
			this->log_WARNING_HI_ZR_SocketError(errArg);
			return -1;
		}

		// Connect publish socket
		(void)snprintf(endpoint,ZMQ_RADIO_ENDPOINT_NAME_SIZE,"tcp://%s:%d",this->m_hostname, this->m_serverSubPort);
		// null terminate
        endpoint[ZMQ_RADIO_ENDPOINT_NAME_SIZE-1] = 0;
		rc = zmq_connect(this->m_pubSocket,endpoint);
		if (-1 == rc) {
			zmqError("ZmqRadioComponentImpl::registerToServer Error connecting publish socket.");
			Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
			this->log_WARNING_HI_ZR_SocketError(errArg);
			return -1;
		} 

	    return 0;

	}

	/* Handlers */

	void ZmqRadioComponentImpl::reconnect_handler(NATIVE_INT_TYPE portNum, NATIVE_UINT_TYPE context ){

		switch(this->m_state.get()){

			case State::ZMQ_RADIO_CONNECTED_STATE:
				// We are connected, do nothing
				DEBUG_PRINT("reconnect_handler: Is connected. Do nothing.\n");
				break;
			case State::ZMQ_RADIO_DISCONNECTED_STATE:
				// We are disconnected, attempt reconnection
				DEBUG_PRINT("reconnect_handler: Not connected. Reconnect.\n");
				this->connect();
				break;

			default:
		        FW_ASSERT(0);

		}
		
		// Send out telemetry
		this->tlmWrite_ZR_NumDisconnects(this->m_numDisconnects);
		this->tlmWrite_ZR_NumConnects(this->m_numConnects);
		this->tlmWrite_ZR_NumDisconnectRetries(this->m_numDisconnectRetries);
		this->tlmWrite_ZR_PktsSent(this->m_packetsSent);
		this->tlmWrite_ZR_PktsRecv(this->m_packetsRecv);

	}


	void ZmqRadioComponentImpl::downlinkPort_handler(
				    NATIVE_INT_TYPE portNum,
				    Fw::ComBuffer &data,
				    U32 context
	)
	{	
		int rc = 0;
		switch(this->m_state.get()){

			case State::ZMQ_RADIO_CONNECTED_STATE:
				rc = zmqSocketWriteComBuffer(this->m_pubSocket, data);

				break;
			case State::ZMQ_RADIO_DISCONNECTED_STATE:
				// Drop packets
				break;

			default:
		        FW_ASSERT(0);
		}	
	}

	void ZmqRadioComponentImpl::fileDownlinkBufferSendIn_handler(
	    NATIVE_INT_TYPE portNum, /*!< The port number*/
	    Fw::Buffer fwBuffer 
	)
	{
		switch(this->m_state.get()){

			case State::ZMQ_RADIO_CONNECTED_STATE:
				// Write files down
				this->zmqSocketWriteFilePacket(this->m_pubSocket, fwBuffer);
				break;
			case State::ZMQ_RADIO_DISCONNECTED_STATE:
				// Drop packets
				break;

			default:
		        FW_ASSERT(0);

		}
	}

	/* FPrime ZMQ Wrapper functions */

    NATIVE_INT_TYPE ZmqRadioComponentImpl::zmqSocketWriteComBuffer(void* zmqSocket, Fw::ComBuffer &data) {

    	//printf("Data Size: 0x%04x\n", data.getBuffLength());
    	//printf("Data Desc: 0x%04x\n", *(U32*)data.getBuffAddr());


    	U32 data_net_size = htonl(data.getBuffLength());
    	U8 buf[sizeof(data_net_size) + data.getBuffLength()]; // Create a buffer to hold entire packet
    	memcpy(buf, &data_net_size, sizeof(data_net_size)); // Copy size
    	memcpy(buf + sizeof(data_net_size),  (U8*)data.getBuffAddr(), data.getBuffLength()); // Copy packet

    	zmq_msg_t fPrimePacket;
    	zmq_msg_init_size(&fPrimePacket, sizeof(buf));
    	memcpy(zmq_msg_data(&fPrimePacket), buf, sizeof(buf));
		
		this->zmqSocketWrite(zmqSocket, &fPrimePacket);
    	
    	return 1;
    }

   	NATIVE_INT_TYPE ZmqRadioComponentImpl::zmqSocketWriteFilePacket(void* zmqSocket, Fw::Buffer &buffer){

   		U32 bufferSize = buffer.getsize();

   		U32 packetSize = htonl(bufferSize + 4); // Size of buffer plus description
   		U32 desc       = 3; // File desc
   		U8 downlinkBuffer[sizeof(packetSize) + sizeof(desc) + bufferSize]; // Create a buffer for header and packet

   		memcpy(downlinkBuffer, &packetSize, sizeof(packetSize));
   		memcpy(downlinkBuffer + sizeof(packetSize), &desc, sizeof(desc));
   		memcpy(downlinkBuffer + sizeof(packetSize) + sizeof(desc) , (U8*)buffer.getdata(), bufferSize);

   		zmq_msg_t fPrimePacket;
    	zmq_msg_init_size(&fPrimePacket, sizeof(downlinkBuffer));
    	memcpy(zmq_msg_data(&fPrimePacket), downlinkBuffer, sizeof(downlinkBuffer));

    	this->zmqSocketWrite(zmqSocket, &fPrimePacket);

   		return 1;
   	}

   	void ZmqRadioComponentImpl::zmqSocketWrite(void* zmqSocket, zmq_msg_t* fPrimePacket){
   		int rc = zmq_msg_send(fPrimePacket, zmqSocket, 0);
    	zmq_msg_close(fPrimePacket);

    	if(rc == -1){
    		zmqError("zmqSocketWrite Error\n");
    		if(zmq_errno() == EAGAIN){ // HWM reached and timed out. Assume connection down.
    			this->m_state.transitionDisconnected();

    		}else{
	    		Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
	    		this->log_WARNING_HI_ZR_SendError(errArg);
    		}
    	}else{
    		// Success. Increase packets sent
    		this->m_packetsSent++;
    	}
   	}


    NATIVE_INT_TYPE ZmqRadioComponentImpl::zmqSocketRead(void* zmqSocket, U8* buf, NATIVE_INT_TYPE size) {

		NATIVE_INT_TYPE rc = 0; // return code
        NATIVE_INT_TYPE total=0;

        // Ignore the zmq identifier
		zmq_msg_t zmqID;
    	zmq_msg_init(&zmqID);
    	rc = zmq_msg_recv(&zmqID, zmqSocket, 0);
    	zmq_msg_close(&zmqID);
    	if(rc == -1){
    		if(zmq_errno() == EAGAIN){ // Recv call timed out 
    			return ZMQ_SOCKET_READ_EAGAIN;
    		}else{ // A more serious error has occured
	    		zmqError("ZmqRadioComponentImpl::zmqSocketRead: zmq_msg_recv error.");
		    	Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
		    	this->log_WARNING_HI_ZR_ReceiveError(errArg);
		    	return ZMQ_SOCKET_READ_ERROR;
	    	}
    	}

    	// Receive FPrime packet
    	zmq_msg_t fPrimePacket;
    	zmq_msg_init(&fPrimePacket);
    	total = zmq_msg_recv(&fPrimePacket, zmqSocket, 0);

    	if(total == -1){
    		if(zmq_errno() == EAGAIN){ // Recv timed out
    			return ZMQ_SOCKET_READ_EAGAIN;
    		}else{

	    		zmqError("ZmqRadioComponentImpl::zmqSocketRead: zmq_msg_recv error.");
		    	Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
		    	this->log_WARNING_HI_ZR_ReceiveError(errArg);
		    	return ZMQ_SOCKET_READ_ERROR;
		    }

    	}else{ // Success. Copy packet data into buf, close message, and increase number packets received
    		memcpy(buf, zmq_msg_data(&fPrimePacket), total);
    		zmq_msg_close(&fPrimePacket);
    		this->m_packetsRecv++;
    	}

        return total;
    }

    void ZmqRadioComponentImpl::startSubscriptionTask(I32 priority){
		Fw::EightyCharString name("ScktRead");

        // Spawn read task
    	Os::Task::TaskStatus stat = this->subscriptionTask.start(name,0, priority,10*1024, ZmqRadioComponentImpl::subscriptionTaskRunnable, this);
    	FW_ASSERT(Os::Task::TASK_OK == stat,static_cast<NATIVE_INT_TYPE>(stat));
    
    }


    void ZmqRadioComponentImpl::subscriptionTaskRunnable(void* ptr){
    	DEBUG_PRINT("Entering subscriptionTask\n");
    	fflush(stderr);

		// Get reference to component
		ZmqRadioComponentImpl* comp = (ZmqRadioComponentImpl*) ptr;
    	
    	void* subSocket = 0; // Zmq Subscription socket

		U32 packetDelimiter;
		U32 packetSize;
		U32 packetDesc;
		U8 buf[FW_COM_BUFFER_MAX_SIZE];
		
    	while(1){
	        U16 buf_ptr = 0; // Reset buffer pointer


	        switch(comp->m_state.get()){
	        	case State::ZMQ_RADIO_DISCONNECTED_STATE:

	        		// Idle
	        		break;

		        case State::ZMQ_RADIO_CONNECTED_STATE:

		        	if(subSocket == 0){ // Create subSocket if disconnected
					    subSocket = zmq_socket(comp->m_context, ZMQ_ROUTER); 
					    if(subSocket == 0){ 
							zmqError("ZmqRadioComponentImpl::connect Error creating sub socket");
							Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
							comp->log_WARNING_HI_ZR_SocketError(errArg);

							// Close this thread's zmq resource
							zmq_close(subSocket);
	        				subSocket = 0;

							comp->m_state.transitionDisconnected();
							break;
					    }
					    zmq_setsockopt(subSocket, ZMQ_IDENTITY, &comp->m_zmqId, strlen(comp->m_zmqId));
					    zmq_setsockopt(subSocket, ZMQ_LINGER, &ZMQ_RADIO_LINGER, sizeof(ZMQ_RADIO_LINGER));
					    zmq_setsockopt(subSocket, ZMQ_RCVTIMEO, &ZMQ_RADIO_RCVTIMEO, sizeof(ZMQ_RADIO_RCVTIMEO));
					    zmq_setsockopt(subSocket, ZMQ_SNDTIMEO, &ZMQ_RADIO_SNDTIMEO, sizeof(ZMQ_RADIO_SNDTIMEO));

						// Connect subscribe socket
						char endpoint[ZMQ_RADIO_ENDPOINT_NAME_SIZE];
						(void)snprintf(endpoint, ZMQ_RADIO_ENDPOINT_NAME_SIZE, "tcp://%s:%d", comp->m_hostname, comp->m_serverPubPort);
						int rc = zmq_connect(subSocket,endpoint);
						if (-1 == rc) {
							zmqError("ZmqRadioComponentImpl::subscriptionTaskRunnable: Error connecting subscribe socket.");
							Fw::LogStringArg errArg(zmq_strerror(zmq_errno()));
							comp->log_WARNING_HI_ZR_SocketError(errArg);

							// Close this thread's zmq resource
							zmq_close(subSocket);
	        				subSocket = 0;

							comp->m_state.transitionDisconnected();
							break;
						} 


		        	} // if subSocket == 0

		    		// Read incoming zmq message
		    		I32 msgSize = 0;
					msgSize = comp->zmqSocketRead(subSocket, buf, (NATIVE_INT_TYPE)FW_COM_BUFFER_MAX_SIZE);
					if(msgSize > 0){ // Successful read
						// Pass
					}else if(msgSize == ZMQ_SOCKET_READ_ERROR){ // A serious zmq error
						// Close this thread's zmq resource
						zmq_close(subSocket);
        				subSocket = 0;

						comp->m_state.transitionDisconnected();
						break;
					}else if(msgSize == ZMQ_SOCKET_READ_EAGAIN){ // Socket has timed out
						break; // Break switch and retry
					}

					// Extract packet delimiter
					packetDelimiter = *(U32*)(buf+buf_ptr);
					packetDelimiter = ntohl(packetDelimiter);

		            // correct for network order
		            packetDelimiter = ntohl(packetDelimiter);
		            //printf("Packet delimiter: 0x%04x\n",packetDelimiter);

		            // if magic number to quit, exit loop
		            if (packetDelimiter == 0xA5A5A5A5) {
		                (void) printf("packetDelimiter = 0x%x\n", packetDelimiter);
		                //break;
		            } else if (packetDelimiter != 0x5A5A5A5A) {
		                (void) printf("Unexpected delimiter 0x%08X\n",packetDelimiter);
		                // just keep reading until a delimiter is found
		                continue;
		            }

		            // Increment buffer pointer
		            buf_ptr += sizeof(packetDelimiter);

		            // Extract FPrime packet size
		            packetSize = *(U32*)(buf + buf_ptr);
		            packetSize = ntohl(packetSize);
		            //printf("Packet Size: 0x%04x\n", packetSize);

		            // Increment buffer pointer
		            buf_ptr += sizeof(packetSize);

		            // Extract FPrime packet description
		            packetDesc = *(U32*)(buf + buf_ptr);
		            packetDesc = ntohl(packetDesc);

		            // Increment buffer pointer
		            buf_ptr += sizeof(packetDesc);

		            switch(packetDesc) {

		                case Fw::ComPacket::FW_PACKET_COMMAND:
		                {
		                	U8 cmdPacket[FW_COM_BUFFER_MAX_SIZE];

		                    // check size of command
		                    if (packetSize > FW_COM_BUFFER_MAX_SIZE) {
		                        (void) printf("Packet to large! :%d\n",packetSize);
		                        // might as well wait for the next packet
		                        break;
		                    }


		                    /*
		                    cmdBuffer[3] = packetDesc & 0xff;
		                    cmdBuffer[2] = (packetDesc & 0xff00) >> 8;
		                    cmdBuffer[1] = (packetDesc & 0xff0000) >> 16;
		                    cmdBuffer[0] = (packetDesc & 0xff000000) >> 24;
							*/
				    		
				    		// Add description
		                    packetDesc = ntohl(packetDesc); // Is this the same as above?
		                    memcpy(cmdPacket, &packetDesc, sizeof(packetDesc));

				    		// Add command data [ Size of cmd data is packet size - packetDesc size ]
		                    memcpy(cmdPacket + sizeof(packetDesc), buf + buf_ptr, packetSize - sizeof(packetDesc));


		                    if (comp->isConnected_uplinkPort_OutputPort(0)) {
		                         Fw::ComBuffer cmdBuffer(cmdPacket, packetSize);
		                         comp->uplinkPort_out(0,cmdBuffer,0);
		                    }
		                    break;
		                }
		            

		                case Fw::ComPacket::FW_PACKET_FILE:
		                {
		                
		                    // Get Buffer
		                    Fw::Buffer packet_buffer = comp->fileUplinkBufferGet_out(0, packetSize - sizeof(packetDesc));
		                    U8* data_ptr = (U8*)packet_buffer.getdata();

							/*
		                     for(uint32_t i =0; i < bytesRead; i++){
		                         (void) printf("IN_DATA:%02x\n", data_ptr[i]);
		                     }
							*/

							// Read file packet minus description. Same as above?
		                    memcpy(data_ptr, buf + buf_ptr, packetSize - sizeof(packetDesc));

		                    if (comp->isConnected_fileUplinkBufferSendOut_OutputPort(0)) {
		                        comp->fileUplinkBufferSendOut_out(0, packet_buffer);
		                    }

		                    break;
		                }

		                default:
		                    FW_ASSERT(0);
		            }

		            break;
	        
	        } // State switch

        } // while 1

    }


	/* State Class Implementation */	

	ZmqRadioComponentImpl::State::State(ZmqRadioComponentImpl* parent):
		state(ZMQ_RADIO_DISCONNECTED_STATE),
		m_parent(parent)
	{
		
	}

	U8 ZmqRadioComponentImpl::State::get(){
		return this->state;
	}

	void ZmqRadioComponentImpl::State::transitionConnected(){

		switch(this->state){

			case ZMQ_RADIO_CONNECTED_STATE:
				// Already connected
				break;

			case ZMQ_RADIO_DISCONNECTED_STATE:
				// Successful reconnection
				this->state = ZMQ_RADIO_CONNECTED_STATE;

				this->m_parent->log_ACTIVITY_HI_ZR_Connection();

				// Clear throttled logs
				this->m_parent->log_WARNING_HI_ZR_ReceiveError_ThrottleClear();
				this->m_parent->log_WARNING_HI_ZR_SendError_ThrottleClear();

				this->m_parent->m_numConnects++;
				break;

			default:
		    	FW_ASSERT(0);
		}

	}

	void ZmqRadioComponentImpl::State::transitionDisconnected(){
		/*
			This function has a mutex beacuse either the main thread or the subscription thread
			can call transitionDisconnected().
		*/

		// Ensure transition calls are atomic
		static Os::Mutex mutex;
		mutex.lock();

		// Release ZMQ resources to prepare for reconnect attempts
		zmq_close(this->m_parent->m_pubSocket);
		zmq_close(this->m_parent->m_cmdSocket);
		zmq_term(this->m_parent->m_context);

		switch(this->state){

			case ZMQ_RADIO_CONNECTED_STATE:
				DEBUG_PRINT("Disconnecting\n");

				// Disconnection experienced
				this->state = ZMQ_RADIO_DISCONNECTED_STATE;
				this->m_parent->log_WARNING_HI_ZR_Disconnection();
				this->m_parent->m_numDisconnects++;

				break;
			case ZMQ_RADIO_DISCONNECTED_STATE:
				// Reconnection failed

				this->m_parent->m_numDisconnectRetries++;
				break;

			default:
		        FW_ASSERT(0);

		}
		mutex.unLock();
	}

} // namespace Zmq 
