/*
 * LLRouterComponentImplCfg.hpp
 *
 *  Created on: Feb 22, 2017
 *      Author: tcanham
 */

#ifndef LLROUTER_LLROUTERCOMPONENTIMPLCFG_HPP_
#define LLROUTER_LLROUTERCOMPONENTIMPLCFG_HPP_

enum {
    SERIAL_BUFFER_SIZE = 4096,
    MAX_TRANS_SIZE = 4096,
    SERIAL_RECEIVE_BUFFER_POOL_SIZE = 10 // allows slip of 10 cycles
};

namespace HLProc {

enum {
    LLRTR_SCHED_TLM,
    LLRTR_SCHED_CYCLE_SERIAL
};

}

#endif /* LLROUTER_LLROUTERCOMPONENTIMPLCFG_HPP_ */
