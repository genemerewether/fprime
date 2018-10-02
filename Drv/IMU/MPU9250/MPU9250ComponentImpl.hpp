// ======================================================================
// \title  MPU9250Impl.hpp
// \author mereweth
// \brief  hpp file for MPU9250 component implementation class
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

#ifndef MPU9250_HPP
#define MPU9250_HPP

#include "Drv/IMU/MPU9250/MPU9250ComponentAc.hpp"
#include "Drv/IMU/MPU9250/MPU9250ComponentImplCfg.hpp"

namespace Drv {

  class MPU9250ComponentImpl :
    public MPU9250ComponentBase
  {

    public:

      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct object MPU9250
      //!
      MPU9250ComponentImpl(
#if FW_OBJECT_NAMES == 1
          const char *const compName, /*!< The component name*/
#endif
          bool useMagnetometer
      );

      //! Initialize object MPU9250
      //!
      void init(
          const NATIVE_INT_TYPE instance = 0 /*!< The instance number*/
      );

      //! Destroy object MPU9250
      //!
      ~MPU9250ComponentImpl(void);

    PRIVATE:

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    //! Handler implementation for sched
    //!
    void sched_handler(
        const NATIVE_INT_TYPE portNum, /*!< The port number*/
        NATIVE_UINT_TYPE context /*!< The call order*/
    );

    // ----------------------------------------------------------------------
    // Constants/Types
    // ----------------------------------------------------------------------

    /*! \brief MPU9250 Initialization State
     *
     */
    enum InitState {
        INIT_RESET,
        INIT_POWER_ON_1,
        INIT_POWER_ON_2,
        INIT_I2C_RESET,
        INIT_FIFO_RESET,
        INIT_FIFO_CONFIG,
        INIT_INT_CONFIG,
        INIT_GEN_CONFIG,
        INIT_GYRO_CONFIG,
        INIT_ACCEL_CONFIG_1,
        INIT_ACCEL_CONFIG_2,
        INIT_MAG_CONFIG,
        INIT_COMPLETE,
        INIT_ERROR,
    } m_initState;

    enum OperationMode {
        OPMODE_INTERRUPT,
        OPMODE_FIFO,
    } m_opMode;

    // ----------------------------------------------------------------------
    // Member variables
    // ----------------------------------------------------------------------

    float m_gyroRawToRadS;
    float m_accelRawToMS2;

    bool m_useMagnetometer;

    NATIVE_UINT_TYPE m_cycleCount;

  };

} // end namespace Drv

#endif
