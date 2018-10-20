// ====================================================================== 
// \title  R5PrmImpl.hpp
// \author tcanham
// \brief  hpp file for R5Prm component implementation class
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

#ifndef R5Prm_HPP
#define R5Prm_HPP

#include "R5/R5Prm/R5PrmComponentAc.hpp"
#include <Fw/Types/Serializable.hpp>
#include <R5/Ports/PrmEntry.hpp>
#include <R5/R5Prm/R5PrmComponentImplCfg.hpp>


#define SECTOR_SIZE 4096
#define PARAMETER_SECTOR 1
#define BACKUP_SECTOR 2
#define UPPER_BANK_PTR ((uint8_t*)0xF0200000)

#define BYTES_PER_WRITE 8

// TODO: Find a good value for this
#define LOOP_MAX_IDX 1000000

// Bitfields to determine if a parameter's value is valid
#define BITFIELD_PARAMETER(X) ((R5::MAX_R5_PARAMS * (4 * 3)) + (X))

// CRC32 of all parameters
#define CRC_PARAMETER 4092

#define NUM_CRC32_CALCS 128

namespace R5 {

  class R5PrmComponentImpl :
    public R5PrmComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object R5Prm
      //!
      R5PrmComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName /*!< The component name*/
#else
          void
#endif
      );

      //! Initialize object R5Prm
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! load from flash
      void load(void);

      //! Destroy object R5Prm
      //!
      ~R5PrmComponentImpl(void);

      // Set the parameter array to 0. Currently used during testing
      void clear_parameter_memory(void);

    PRIVATE:

      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for set
      //!
      void set_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::PrmEntry &prm,
          bool &valid
      );

      //! Handler implementation for get
      //!
      void get_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          R5::PrmEntry &prm, 
          bool &valid 
      );

      //! Handler implementation for write
      //!
      void write_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          bool force
      );

      //! Handler implementation for check
      //!
      void check_handler(
          const NATIVE_INT_TYPE portNum, /*!< The port number*/
          PrmWriteStatus &stat 
      );

      //! Check if a parameter id has a value
      bool have_parameter(uint32_t param_id);


      typedef enum {
          CS_INIT,
          CS_CALC_CRC,
          CS_INIT_FLASH,
          CS_PRIMARY_ENABLE,
          CS_PRIMARY_ENABLE_WAIT,
          CS_PRIMARY_ERASE,
          CS_PRIMARY_ERASE_WAIT,
          CS_PRIMARY_WRITE,
          CS_PRIMARY_WRITE_WAIT,
          CS_BACKUP_ENABLE,
          CS_BACKUP_ENABLE_WAIT,
          CS_BACKUP_ERASE,
          CS_BACKUP_ERASE_WAIT,
          CS_BACKUP_WRITE,
          CS_BACKUP_WRITE_WAIT,
          CS_DONE
      } CommitState;


      uint8_t m_parameters[SECTOR_SIZE];

      uint32_t m_writesLeft;
      uint8_t* m_writeAddress;
      uint8_t* m_paramAddress;

      uint32_t m_crc32;
      uint32_t m_crc_calcs_left;

      PrmWriteStatus m_prmStatus;
      CommitState m_commitState;

      Fw::ExternalSerializeBuffer m_extParamBuffer;

    };

} // end namespace R5

#endif
