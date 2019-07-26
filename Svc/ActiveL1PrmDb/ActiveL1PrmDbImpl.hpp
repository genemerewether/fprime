// ====================================================================== 
// \title  ActiveL1PrmDbImpl.hpp
// \author kubiak
// \brief  hpp file for ActiveL1PrmDb component implementation class
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

#ifndef ActiveL1PrmDb_HPP
#define ActiveL1PrmDb_HPP

#include "Svc/ActiveL1PrmDb/ActiveL1PrmDbComponentAc.hpp"
#include <Os/File.hpp>

#include <Svc/ActiveL1PrmDb/ActiveL1PrmDbImplCfg.hpp>
#include <Svc/PrmDb/PrmDbImpl.hpp>

namespace Svc {

  class ActiveL1PrmDbComponentImpl :
    public ActiveL1PrmDbComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object ActiveL1PrmDb
      //!
      ActiveL1PrmDbComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
          const char *file,
          const NATIVE_INT_TYPE maxRecvSize
#else
          const char *file,
          const NATIVE_INT_TYPE maxRecvSize
#endif
      );

      //! Initialize object ActiveL1PrmDb
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object ActiveL1PrmDb
      //!
      ~ActiveL1PrmDbComponentImpl(void);

      struct PrmDbRange {

          PrmDbRange() :
              port(0),
              lowId(0),
              highId(0),
              valid(false) {
          }


          PrmDbRange(const NATIVE_INT_TYPE p,
                     const FwPrmIdType l,
                     const FwPrmIdType h) : 
              port(p),
              lowId(l),
              highId(h),
              valid(true) {

          }

          NATIVE_INT_TYPE port;
          FwPrmIdType lowId;
          FwPrmIdType highId;
          bool valid;
      };

      void readPrmFile(void);

      void setPrmDbRanges(const PrmDbRange* ranges,
                          const NATIVE_INT_TYPE numRanges);

      void setupPrmTransfer();

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for sendPrmReady
      //!
      void sendPrmReady_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 maxSize,
          bool reload
      );


      //! Handler implementation for setPrm
      //!
      void setPrm_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType id, 
          Fw::ParamBuffer &val 
      );

      //! Handler for input port recvPrm
      //
      void recvPrm_handler(
          NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool morePrms, 
          Fw::ParamList val 
      );

      //! Handler implementation for pingIn
      //!
      void pingIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

      //! Handler implementation for getPrm
      //!
      Fw::ParamValid getPrm_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          FwPrmIdType id, 
          Fw::ParamBuffer &val 
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations 
      // ----------------------------------------------------------------------

      //! Implementation for PRM_SAVE_FILE command handler
      //! Command to save parameter image to file. Uses file name passed to constructor
      void PRM_SAVE_FILE_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      // ----------------------------------------------------------------------
      // Helper Functions
      // ----------------------------------------------------------------------

      bool prmIdInRange(const NATIVE_INT_TYPE port, const FwPrmIdType id);


      PrmDbImpl m_prmDb;

      Fw::EightyCharString m_fileName;

      struct PrmDbRange m_prmDbRanges[ACTIVE_PRMDB_NUM_RANGES];

      struct Level2PrmDb {
          PrmDbImpl::PrmDbIterator iter;
          Fw::ParamList m_sendBuffer;
      } m_level2PrmDbs[NUM_SENDPRM_OUTPUT_PORTS];

      const NATIVE_INT_TYPE m_maxRecvSize;

    };

} // end namespace Svc

#endif
