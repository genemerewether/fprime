// ====================================================================== 
// \title  STIM300Impl.cpp
// \author kubiak
// \brief  cpp file for STIM300 component implementation class
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


#include <Drv/IMU/STIM300/STIM300Impl.hpp>
#include <Drv/IMU/STIM300/STIM300ImplCfg.hpp>
#include <Drv/IMU/STIM300/STIM300Pkt.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <ROS/Gen/sensor_msgs/Types/ImuNoCovSerializableAc.hpp>
#include <ROS/Gen/geometry_msgs/Types/QuaternionSerializableAc.hpp>
#include <ROS/Gen/geometry_msgs/Types/Vector3SerializableAc.hpp>

namespace Drv {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction 
  // ----------------------------------------------------------------------

  STIM300ComponentImpl ::
#if FW_OBJECT_NAMES == 1
    STIM300ComponentImpl(
        const char *const compName
    ) :
      STIM300ComponentBase(compName),
#else
    STIM300Impl(void) :
#endif
      m_pktBuffer(),
      m_pktBufferIdx(),
      m_uartFwBuffer(),
      m_uartBuffer(),
      m_pktCounter(0),
      m_timeSyncState(S300_TS_CLEAR),
      m_numReceivedPkts(0),
      m_tsCheckCount(0),
      m_eventsRing()
  {
  }

  void STIM300ComponentImpl ::
    init(
        const NATIVE_INT_TYPE instance
    ) 
  {
    STIM300ComponentBase::init(instance);

    this->m_uartFwBuffer.setdata(reinterpret_cast<U64>(&this->m_uartBuffer[0]));
  }

  STIM300ComponentImpl ::
    ~STIM300ComponentImpl(void)
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void STIM300ComponentImpl ::
    sched_handler(
        const NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {


    switch (this->m_timeSyncState) {
      case S300_TS_CLEAR:
        this->runTsClear();
        break;
      case S300_TS_WAIT:
        this->runTsWait();
        break;
      case S300_TS_CHECK:
        this->runTsCheck();
        break;
      case S300_TS_SYNCED:
        this->runTsSynced();
        break;
      case S300_TS_NOSYNC:
        this->runTsNosync();
        break;
      default:
        FW_ASSERT(false, m_timeSyncState);
        break;
    }
    

    // Push Telemetry
    switch (this->m_timeSyncState) {
      case S300_TS_CLEAR:
          this->tlmWrite_TimeSyncStatus(STIM300_TS_CLEAR);
          break;
      case S300_TS_WAIT:
          this->tlmWrite_TimeSyncStatus(STIM300_TS_WAIT);
          break;
      case S300_TS_CHECK:
          this->tlmWrite_TimeSyncStatus(STIM300_TS_CHECK);
          break;
      case S300_TS_SYNCED:
          this->tlmWrite_TimeSyncStatus(STIM300_TS_SYNCED);
          break;
      case S300_TS_NOSYNC:
          this->tlmWrite_TimeSyncStatus(STIM300_TS_NOSYNC);
          break;
      default:
          FW_ASSERT(false, this->m_timeSyncState);
          break;
    }

    this->tlmWrite_NumPackets(this->m_numReceivedPkts);
  }

  void STIM300ComponentImpl ::
    runTsClear()
  {
    Fw::Time eventTime;

    // Clear Uart buffer
    this->m_pktBufferIdx = 0;

    // Gather all events. Ignore errors
    (void)this->gatherEvents();

    this->readUartBytes();
    this->m_pktBufferIdx = 0;

    this->m_eventsRing.reset();

    this->m_timeSyncState = S300_TS_WAIT;
  }

  void STIM300ComponentImpl ::
    runTsWait()
  {
    ROS::sensor_msgs::ImuNoCov imuPkt;
    U32 bytesRead = 0;
    Fw::Time eventTime;
    STIM300FindPktStatus findPktStat;
    STIM300GatherStatus gatherStat;
    STIM300ReadUartStatus readStat;

    gatherStat = this->gatherEvents();
    if (gatherStat == S300_GATHER_DROPPED) {
        this->log_WARNING_HI_TooManyEvents(STIM_MAX_EVENTS);
        this->m_timeSyncState = S300_TS_CLEAR;
    } else {
        FW_ASSERT(gatherStat == S300_GATHER_OK, gatherStat);

        readStat = this->readUartBytes();

        if (readStat == S300_UART_FULL ||
            readStat == S300_UART_ERR) {
            this->m_timeSyncState = S300_TS_CLEAR;
        } else {

            findPktStat = findSTIMPkt(&this->m_pktBuffer[bytesRead], this->m_pktBufferIdx, bytesRead, imuPkt);

            // Shift remaining bytes to the beginning of the uart buffer
            FW_ASSERT(this->m_pktBufferIdx >= bytesRead, this->m_pktBufferIdx, bytesRead);
            memmove(this->m_pktBuffer, &this->m_pktBuffer[bytesRead], this->m_pktBufferIdx - bytesRead);
            this->m_pktBufferIdx -= bytesRead;

            switch (findPktStat) {
                case S300_PKT_FOUND:
                    this->m_timeSyncState = S300_TS_CHECK;
                    this->m_pktCounter = imuPkt.getheader().getseq();
                    this->m_tsCheckCount = 0;

                    // Start checking TimeSync
                    this->runTsCheck();
                    break;
                case S300_PKT_NONE:
                    // Do nothing
                    break;
                case S300_PKT_LOST_SYNC:
                    // Try to re-sync
                    this->m_timeSyncState = S300_TS_CLEAR;
                    break;
                default:
                    FW_ASSERT(false, findPktStat);
                    break;
            }
        }
    }
  }

  void STIM300ComponentImpl ::
    runTsCheck()
  {
    FW_ASSERT(this->m_pktBufferIdx <= STIM_PKT_BUFFER_SIZE);

    Fw::Time eventTime;
    ROS::sensor_msgs::ImuNoCov imuPkt;
    U32 totalBytesRead = 0;
    STIM300FindPktStatus findPktStat;
    STIM300GatherStatus gatherStat;
    STIM300ReadUartStatus readStat;

    gatherStat = this->gatherEvents();
    if (gatherStat == S300_GATHER_DROPPED) {
        this->log_WARNING_HI_TooManyEvents(STIM_MAX_EVENTS);
        this->m_timeSyncState = S300_TS_CLEAR;
    } else {
        FW_ASSERT(gatherStat == S300_GATHER_OK, gatherStat);

        readStat = this->readUartBytes();

        if (readStat == S300_UART_FULL ||
            readStat == S300_UART_ERR) {
            this->m_timeSyncState = S300_TS_CLEAR;
        } else {
            FW_ASSERT(readStat == S300_UART_OK, readStat);

            // Find all packets in the current buffer
            do {
                U32 bytesRead = 0;

                findPktStat = findSTIMPkt(&this->m_pktBuffer[totalBytesRead], this->m_pktBufferIdx - totalBytesRead, bytesRead, imuPkt);

                if (findPktStat == S300_PKT_FOUND) {

                    if (imuPkt.getheader().getseq() != ((this->m_pktCounter + 1) % 256)) {
                        // Unexpected sequence number
                        //findPktStat = S300_PKT_LOST_SYNC;
                    } else {
                        this->m_pktCounter = (this->m_pktCounter + 1) % 256;
                    }
                }

                totalBytesRead += bytesRead;
            } while (findPktStat == S300_PKT_FOUND);

            FW_ASSERT(totalBytesRead <= this->m_pktBufferIdx, totalBytesRead, this->m_pktBufferIdx);

            // Shift remaining bytes to the beginning of the uart buffer
            if (totalBytesRead < this->m_pktBufferIdx) {
                memmove(this->m_pktBuffer, &this->m_pktBuffer[totalBytesRead], this->m_pktBufferIdx - totalBytesRead);
            }
            this->m_pktBufferIdx -= totalBytesRead;

            if (totalBytesRead > 0) {
                this->m_tsCheckCount++;
            }

            if (findPktStat == S300_PKT_LOST_SYNC) {
                this->m_timeSyncState = S300_TS_CLEAR;
            } else if (this->verifyConsistency() == S300_CONSISTENCY_INVALID) {
                this->m_timeSyncState = S300_TS_CLEAR;
            } else if (this->m_tsCheckCount == STIM_TS_CHECK_CYCLES) {
                log_ACTIVITY_LO_SyncComplete();
                this->m_timeSyncState = S300_TS_SYNCED;
            }
        }
    }
  }


  void STIM300ComponentImpl ::
    runTsSynced()
  {
    FW_ASSERT(this->m_pktBufferIdx <= STIM_PKT_BUFFER_SIZE);

    Fw::Time eventTime;
    ROS::sensor_msgs::ImuNoCov imuPkt;
    U32 totalBytesRead = 0;
    STIM300FindPktStatus findPktStat;
    STIM300GatherStatus gatherStat;
    STIM300ReadUartStatus readStat;

    gatherStat = this->gatherEvents();
    if (gatherStat == S300_GATHER_DROPPED) {
        this->log_WARNING_HI_TooManyEvents(STIM_MAX_EVENTS);
        this->m_timeSyncState = S300_TS_NOSYNC;
    } else {
        FW_ASSERT(gatherStat == S300_GATHER_OK, gatherStat);

        readStat = this->readUartBytes();

        if (readStat == S300_UART_FULL) {
            log_WARNING_HI_BufferFull();
            this->m_timeSyncState = S300_TS_NOSYNC;
        } else if (readStat == S300_UART_ERR) {
            log_WARNING_HI_UartError();
            this->m_timeSyncState = S300_TS_NOSYNC;
        } else {
            FW_ASSERT(readStat == S300_UART_OK, readStat);

            // Find all packets in the current buffer
            do {
                U32 bytesRead = 0;

                findPktStat = findSTIMPkt(&this->m_pktBuffer[totalBytesRead], this->m_pktBufferIdx - totalBytesRead, bytesRead, imuPkt);

                if (findPktStat == S300_PKT_FOUND) {

                    if (0) { //imuPkt.getheader().getseq() != ((this->m_pktCounter + 1) % 256)) {
                        // Unexpected sequence number
                        this->log_WARNING_HI_InvalidCounter(imuPkt.getheader().getseq(), (this->m_pktCounter + 1) % 256);
                        findPktStat = S300_PKT_LOST_SYNC;
                    } else {

                        for (int i = 0; i < NUM_IMU_OUTPUT_PORTS; i++) {
                            if (this->isConnected_IMU_OutputPort(i)) {
                                this->IMU_out(i, imuPkt);
                            }
                        }

                        this->m_pktCounter = (this->m_pktCounter + 1) % 256;
                        this->m_numReceivedPkts++;
                    }
                }

                totalBytesRead += bytesRead;
            } while (findPktStat == S300_PKT_FOUND);

            FW_ASSERT(totalBytesRead <= this->m_pktBufferIdx, totalBytesRead, this->m_pktBufferIdx);

            // Shift remaining bytes to the beginning of the uart buffer
            if (totalBytesRead < this->m_pktBufferIdx) {
                memmove(this->m_pktBuffer, &this->m_pktBuffer[totalBytesRead], this->m_pktBufferIdx - totalBytesRead);
            }

            this->m_pktBufferIdx -= totalBytesRead;

            if (findPktStat == S300_PKT_LOST_SYNC) {
                this->m_timeSyncState = S300_TS_NOSYNC;
            }

            if (this->verifyConsistency() == S300_CONSISTENCY_INVALID) {
                this->m_timeSyncState = S300_TS_NOSYNC;
            }
        }
    }
  }

  void STIM300ComponentImpl::
      runTsNosync()
  {
    // This is a placeholder for better lost-sync behavior
    // If sync is lost, no more IMU will be transmitted
    // due to a questionable time sync. This is the intended
    // behavior for debugging and early testing, but should
    // be modified if the STIM is used in closed-loop flight

    // TODO: Dead Reckoning

    (void)this->gatherEvents();
    this->m_pktBufferIdx = 0;
  }

  STIM300ComponentImpl::STIM300ReadUartStatus
      STIM300ComponentImpl::readUartBytes(void)
  {
    Drv::SerialReadStatus status;

    this->m_uartFwBuffer.setsize(STIM_UART_BUFFER_SIZE);

    this->serialRead_out(0, this->m_uartFwBuffer, status);

    if (status == Drv::SER_OK) {

        if (this->m_uartFwBuffer.getsize() < (STIM_PKT_BUFFER_SIZE - this->m_pktBufferIdx)) {
            // We have enough room in the packet buffer

            memcpy(&this->m_pktBuffer[this->m_pktBufferIdx], &this->m_uartBuffer[0], this->m_uartFwBuffer.getsize());

            this->m_pktBufferIdx += this->m_uartFwBuffer.getsize();
        } else {
            // Not enough room. Drop all bytes
            return S300_UART_FULL;
        }
    } else {
        return S300_UART_ERR;
    }

    return S300_UART_OK;
  }

  // TODO: Make buffer const
  STIM300ComponentImpl::STIM300FindPktStatus
      STIM300ComponentImpl::findSTIMPkt(U8* buffer,
                                        const U32 bufferLength,
                                        U32& bytesRead_out,
                                        ROS::sensor_msgs::ImuNoCov& pkt_out
      )
  {
    U32 idx = 0;
    U32 pktIdx = 0;
    U32 stimPktLen = 0;
    Fw::Time eventTime;
    bool foundPkt = false;
    bool foundCandidatePkt = false;

    while (idx < bufferLength) {

        foundCandidatePkt = false;

        while (idx < bufferLength &&
                !foundCandidatePkt) {

            switch (buffer[idx]) {
                case 0x90:
                    stimPktLen = sizeof(STIM300Packet_90);
                    foundCandidatePkt = true;
                    break;
                case 0x93:
                    stimPktLen = sizeof(STIM300Packet_93);
                    foundCandidatePkt = true;
                    break;
                default:
                    idx++;
                    break;
            }
        }

        if (foundCandidatePkt) {
            // Check that we have enough bytes to calculate CRC
            if (idx + stimPktLen <= bufferLength) {

                U8 dummyBytes = 0xFF;
                for (unsigned int i = 0; i < FW_NUM_ARRAY_ELEMENTS(dummyBytesMap); i++) {
                    if (dummyBytesMap[i].ident == buffer[idx]) {
                        dummyBytes = dummyBytesMap[i].dummyBytes;
                        break;
                    }
                }

                if (dummyBytes != 0xFF) {

                    // Check CRC
                    U32 crc = 0xFFFFFFFF;

                    for (pktIdx = 0; pktIdx < stimPktLen - 4; pktIdx++) {
                        crc = stimCrc32Lookup[((crc >> 24) ^ buffer[idx + pktIdx]) & 0xFF] ^ (crc << 8);
                    }

                    // Add dummy 0 bytes for CRC
                    for (int i = 0; i < dummyBytes; i++) {
                        crc = stimCrc32Lookup[((crc >> 24) ^ 0) & 0xFF] ^ (crc << 8);
                    }

                    for (; pktIdx < stimPktLen; pktIdx++) {
                        crc = stimCrc32Lookup[((crc >> 24) ^ buffer[idx + pktIdx]) & 0xFF] ^ (crc << 8);
                    }

                    if (crc == 0) {
                        // Found a valid packet
                        foundPkt = true;
                        break;
                    } else {
                        idx++;
                    }
                }
            } else {
                // Have not received the full packet. Return
                break;
            }
        }
    }

    if (foundPkt) {
        if (this->m_eventsRing.size() == 0) {
            // Unable to get a TOV event.
            bytesRead_out = idx + pktIdx;
            this->log_WARNING_HI_NoEvents();
            return S300_PKT_LOST_SYNC;
        } else {
            this->m_eventsRing.dequeue(&eventTime);

            switch (buffer[idx]) {
                case 0x90:
                    // Can't convert to Imu packet. No accel data
                    foundCandidatePkt = false;
                    break;
                case 0x93:
                    stimPktLen = sizeof(STIM300Packet_93);
                    toImuPktGeneric(*reinterpret_cast<STIM300Packet_93*>(&buffer[idx]), eventTime, pkt_out);
                    foundCandidatePkt = true;
                    break;
                default:
                    // Should not be possible
                    FW_ASSERT(false, buffer[idx]);
                    break;
            }

            bytesRead_out =  idx + pktIdx;
            return S300_PKT_FOUND;
        }

    } else {
        bytesRead_out = idx;
        return S300_PKT_NONE;
    }
  }



  STIM300ComponentImpl::STIM300GatherStatus
      STIM300ComponentImpl::gatherEvents()
  {
    Fw::Time eventTime;
    bool droppedPackets = false;

    do {
        this->packetTime_out(0, eventTime);

        if (eventTime.getTimeBase() != TB_NONE) {
            if (this->m_eventsRing.queue(&eventTime) != 0) {
                droppedPackets = true;
            }
        }

    } while (eventTime.getTimeBase() != TB_NONE);

    if (droppedPackets) {
        return STIM300ComponentImpl::S300_GATHER_DROPPED;
    } else {
        return STIM300ComponentImpl::S300_GATHER_OK;
    }
  }

  STIM300ComponentImpl::STIM300ConsistencyStatus
      STIM300ComponentImpl::verifyConsistency()
  {
    U32 stimPktLen;

    // Too many events
    if (this->m_eventsRing.size() > 1) {
        return S300_CONSISTENCY_INVALID;
    }

    // Not enough consumed uart data.
    // TODO: Should this be an assert?
    if (this->m_pktBufferIdx > 0) {
        switch (this->m_pktBuffer[0]) {
            case 0x90:
                stimPktLen = sizeof(STIM300Packet_90);
                break;
            case 0x93:
                stimPktLen = sizeof(STIM300Packet_93);
                break;
            default:
                stimPktLen = 0;
                break;
        }

        if (this->m_pktBufferIdx >= stimPktLen) {
            return S300_CONSISTENCY_INVALID;
        }
    }
    
    // TODO: Other error cases?

    return S300_CONSISTENCY_OK;
  }

  const U32 STIM300ComponentImpl::stimCrc32Lookup[256] = {
    0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B, 0x1A864DB2, 0x1E475005,
    0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61, 0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD,
    0x4C11DB70, 0x48D0C6C7, 0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
    0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3, 0x709F7B7A, 0x745E66CD,
    0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039, 0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5,
    0xBE2B5B58, 0xBAEA46EF, 0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
    0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB, 0xCEB42022, 0xCA753D95,
    0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1, 0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D,
    0x34867077, 0x30476DC0, 0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
    0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4, 0x0808D07D, 0x0CC9CDCA,
    0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE, 0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02,
    0x5E9F46BF, 0x5A5E5B08, 0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
    0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC, 0xB6238B25, 0xB2E29692,
    0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6, 0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A,
    0xE0B41DE7, 0xE4750050, 0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
    0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34, 0xDC3ABDED, 0xD8FBA05A,
    0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637, 0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB,
    0x4F040D56, 0x4BC510E1, 0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
    0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5, 0x3F9B762C, 0x3B5A6B9B,
    0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF, 0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623,
    0xF12F560E, 0xF5EE4BB9, 0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
    0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD, 0xCDA1F604, 0xC960EBB3,
    0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7, 0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B,
    0x9B3660C6, 0x9FF77D71, 0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
    0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2, 0x470CDD2B, 0x43CDC09C,
    0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8, 0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24,
    0x119B4BE9, 0x155A565E, 0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
    0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A, 0x2D15EBE3, 0x29D4F654,
    0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0, 0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C,
    0xE3A1CBC1, 0xE760D676, 0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
    0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662, 0x933EB0BB, 0x97FFAD0C,
    0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668, 0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4
  };

  const STIM300ComponentImpl::STIM300DummyMap STIM300ComponentImpl::dummyBytesMap[2] = {
    {0x90, 2},
    {0x93, 2}
  };

} // end namespace 
