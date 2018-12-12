/*
 * HLRouterComponentImplCfg.hpp
 *
 *  Created on: Feb 22, 2017
 *      Author: tcanham
 */

#ifndef HLROUTER_HLROUTERCOMPONENTIMPLCFG_HPP_
#define HLROUTER_HLROUTERCOMPONENTIMPLCFG_HPP_

namespace LLProc {
  
enum {
    SERIAL_BUFFER_SIZE = 800 // 400 bytes, twice what we can send
};

enum {
    HLRTR_SCHED_UART_SEND,
    HLRTR_SCHED_UART_RECEIVE
};
  
}
#endif /* HLROUTER_HLROUTERCOMPONENTIMPLCFG_HPP_ */
