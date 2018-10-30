/*
 * LLDebugComponentImplCfg.hpp
 *
 *  Created on: May 9, 2017
 *      Author: tcanham
 */

#ifndef LLDEBUG_LLDEBUGCOMPONENTIMPLCFG_HPP_
#define LLDEBUG_LLDEBUGCOMPONENTIMPLCFG_HPP_

namespace LLProc {

    static const NATIVE_UINT_TYPE LL_DEBUG_TEXT_SIZE = 40;
    // I know it's a macro, but can't see type from outside component
    #define SW_ASSERT_GPIO_BANK (R5::GPIO_SET_BANK_A)
    static const NATIVE_UINT_TYPE SW_ASSERT_GPIO_BIT = 0;

}

#endif /* LLDEBUG_LLDEBUGCOMPONENTIMPLCFG_HPP_ */
