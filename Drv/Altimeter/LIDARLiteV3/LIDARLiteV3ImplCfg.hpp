/**
 * \file   LIDARLiteV3ComponentImplCfg.hpp
 * \author Gene Merewether, Gerik Kubiak
 * \brief  Config for component that converts Altimeter sensor data from DN to EU
 *
 */


#ifndef LIDARLITEV3COMPONENTIMPLCFG_HPP_
#define LIDARLITEV3COMPONENTIMPLCFG_HPP_

#define LLV3_ACQ_REG (0x0)
#define LLV3_STATUS_REG (0x1)
#define LLV3_SIG_COUNT_VAL_REG (0x2)
#define LLV3_ACQ_CONFIG_REG (0x4)
#define LLV3_THRESHOLD_BYPASS_REG (0x1C)
#define LLV3_MEASURE_REG (0x0F)

#define LLV3_READ_SEQ (1 << 7)

#define LLV3_STATUS_PROC_ERR (1 << 6)
#define LLV3_STATUS_HEALTH_OK (1 << 5)
#define LLV3_STATUS_2ND_RETURN_ERR (1 << 4)
#define LLV3_STATUS_SIGNAL_ERR (1 << 3)
#define LLV3_STATUS_OVF_ERR (1 << 2)
#define LLV3_STATUS_REF_OVF (1 << 1)
#define LLV3_STATUS_BUSY (1 << 0)

#define LLV3_ACQ_CONFIG_DISABLE_REF_PROC (1 << 6)
#define LLV3_ACQ_CONFIG_USE_MEAS_DELAY (1 << 5)
#define LLV3_ACQ_CONFIG_DISABLE_REF_FILTER (1 << 4)
#define LLV3_ACQ_CONFIG_DISABLE_MEAS_TERM (1 << 3)
#define LLV3_ACQ_CONFIG_USE_REF_COUNT_VAL (1 << 2)
#define LLV3_ACQ_CONFIG_DEFAULT_PWM (0 << 0)
#define LLV3_ACQ_CONFIG_STATUS_OUTPUT (1 << 0)
#define LLV3_ACQ_CONFIG_FIXED_DELAY (2 << 0)
#define LLV3_ACQ_CONFIG_OSC_OUT (3 << 0)


namespace Drv {

    static const U8 LLV3_ADDR = 0x62;

    static const U32 LLV3_WRITE_BUFF_LEN = 8;
    static const U32 LLV3_READ_BUFF_LEN = 8;

    static const U8 LLV3_SIG_COUNT_VAL = 0x80;
    static const U8 LLV3_ACQ_CONFIG_VAL = 0x08;
    static const U8 LLV3_THRESHOLD_BYPASS_VAL = 0x00;


    // Take a measurement
    static const U8 LLV3_ACQ_CMD = 0x4;

    static const U8 LLV3_STATUS_ERR_MASK = LLV3_STATUS_PROC_ERR |
                                           LLV3_STATUS_HEALTH_OK |
                                           LLV3_STATUS_2ND_RETURN_ERR |
                                           LLV3_STATUS_SIGNAL_ERR;
                                           //LLV3_STATUS_OVF_ERR;


} // namespace Drv

#endif // LIDARLITEV3COMPONENTIMPLCFG_HPP_
