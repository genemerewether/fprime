// ======================================================================
// \title  R5A2DDriverImpl.cpp
// \author tcanham
// \brief  cpp file for R5A2DDriver component implementation class
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

#include <R5/A2DDrv/R5A2DDriverComponentImpl.hpp>
#include "Fw/Types/BasicTypes.hpp"
#include "R5A2DDriverComponentImplCfg.hpp"

#include <R5/TiHal/include/HL_adc.h>

#include <Fw/Types/Assert.hpp>

#include <stdio.h>  // for printf

#define adc1Ram ((U32 volatile*)0xFF3E0000U)
#define adc2Ram ((U32 volatile*)0xFF3A0000U)

namespace R5 {

    static F32 const A2D_RAW_TO_VOLT_KOEFF = (A2D_REF_VOLTAGE / 4095.0F);

    // This table MUST match the HalCoGen generated list of acquired
    // channels. The ADC packs the channels together in memory based on
    // which channels have been selected for acquisition. The lookup offset
    // in the array should increase by one for every channel acquired.

    static const NATIVE_INT_TYPE Adc1ChannelLookup[32] = {
        -1, // 0
        -1, // 1
        -1, // 2
        -1, // 3
        -1, // 4
        0, // 5
        1, // 6
        -1, // 7
        2, // 8
        3, // 9
        -1, // 10
        4, // 11
        -1, // 12
        5, // 13
        6, // 14
        -1, // 15
        7, // 16
        -1, // 17
        8, // 18
        -1, // 19
        -1, // 20
        -1, // 21
        9, // 22
        -1, // 23
        10, // 24
        11, // 25
        12, // 26
        -1, // 27
        -1, // 28
        -1, // 29
        -1, // 30
        13  // 31
    };

    static const NATIVE_INT_TYPE Adc2ChannelLookup[32] = {
        -1, // 0
        0, // 1
        -1, // 2
        1, // 3
        2, // 4
        -1, // 5
        -1, // 6
        3, // 7
        -1, // 8
        -1, // 9
        4, // 10
        -1, // 11
        5, // 12
        -1, // 13
        -1, // 14
        6, // 15
        7, // 16
        8, // 17
        9, // 18
        10, // 19
        11, // 20
        -1, // 21
        -1, // 22
        -1, // 23
        -1, // 24
        -1, // 25
        -1, // 26
        -1, // 27
        -1, // 28
        -1, // 29
        -1, // 30
        -1  // 31
    };

    // ----------------------------------------------------------------------
    // Handler implementations for user-defined typed input ports
    // ----------------------------------------------------------------------

    void R5A2DDriverComponentImpl::get_handler(const NATIVE_INT_TYPE portNum,
            A2DGetBank bank, U32 channel, F32 &val, U16 &raw) {
        FW_ASSERT(((A2D_GET_BANK_1 == bank) or (A2D_GET_BANK_2 == bank)), bank);
        FW_ASSERT((channel < 32), channel);

        const NATIVE_INT_TYPE *tablePtr = (A2D_GET_BANK_1 == bank) ?
                Adc1ChannelLookup:Adc2ChannelLookup;

        // look up requested channel
        const NATIVE_INT_TYPE offset = tablePtr[channel];
        FW_ASSERT(offset != -1);

        U32 result = (A2D_GET_BANK_1 == bank) ?
                adc1Ram[offset] : adc2Ram[offset];

        raw = 0xfff & result;
        U8 id = (result >> 12) & 0x1f;
        // Make sure the stored ID matches
        FW_ASSERT(channel == id,bank,channel,id,result);
        val = A2D_RAW_TO_VOLT_KOEFF * static_cast<F32>(raw);
    }

} // end namespace R5
