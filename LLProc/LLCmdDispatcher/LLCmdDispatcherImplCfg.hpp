/*
 * CmdDispatcherImplCfg.hpp
 *
 *  Created on: May 6, 2015
 *      Author: tcanham
 */

#ifndef LLCMDDISPATCHER_COMMANDDISPATCHERIMPLCFG_HPP_
#define LLCMDDISPATCHER_COMMANDDISPATCHERIMPLCFG_HPP_

// Define configuration values for dispatcher

namespace LLProc {

enum {
    LLCMD_DISPATCHER_DISPATCH_TABLE_SIZE = 500, // !< The size of the table holding opcodes to dispatch
    LLCMD_DISPATCHER_SEQUENCER_TABLE_SIZE = 25, // !< The size of the table holding commands in progress
};

}

#endif /* LLCMDDISPATCHER_COMMANDDISPATCHERIMPLCFG_HPP_ */
