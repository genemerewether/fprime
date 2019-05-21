// ====================================================================== 
// \title  MVDFSImpl.hpp
// \author mereweth
// \brief  hpp file for MVDFS component implementation class
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

#ifndef MVDFS_HPP
#define MVDFS_HPP

#include "SnapdragonFlight/MVDFS/MVDFSComponentAc.hpp"

#ifdef BUILD_SDFLIGHT
#include "mv.h"
#include "mvDFS.h"
#endif

namespace SnapdragonFlight {

  class MVDFSComponentImpl :
    public MVDFSComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object MVDFS
      //!
      MVDFSComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object MVDFS
      //!
      void init(
          const NATIVE_INT_TYPE queueDepth, /*!< The queue depth*/
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object MVDFS
      //!
      ~MVDFSComponentImpl(void);

    PRIVATE:

      //! Preamble override
      void preamble(void);
      //! Finalizer override
      void finalizer(void);

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for ImageIn
      //!
      void ImageIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          ROS::sensor_msgs::Image &Image 
      );

      //! Handler implementation for sched
      //!
      void sched_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          NATIVE_UINT_TYPE context /*!< The call order*/
      );

      //! Handler implementation for pingIn
      //!
      void pingIn_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          U32 key /*!< Value to return to pinger*/
      );

    PRIVATE:

      // ----------------------------------------------------------------------
      // Command handler implementations 
      // ----------------------------------------------------------------------

      //! Implementation for MVDFS_Reinit command handler
      //! 
      void MVDFS_Reinit_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq /*!< The command sequence number*/
      );

      //! Implementation for MVDFS_Activate command handler
      //! 
      void MVDFS_Activate_cmdHandler(
          const FwOpcodeType opCode, /*!< The opcode*/
          const U32 cmdSeq, /*!< The command sequence number*/
          bool enable 
      );

      void initHelper();

#ifdef BUILD_SDFLIGHT
      struct mvDFS* m_mvDFSPtr;
    
      mvCameraConfiguration m_depthCameraIntrinsics;
#endif

      bool m_initialized;
      
      bool m_activated;

      U32 m_imagesProcessed;

    };

} // end namespace SnapdragonFlight

#endif
