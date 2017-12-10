/*
 * HexRouterComponentImplCfg.hpp
 *
 *  Created on: Dec 9, 2017
 *      Author: mereweth
 */

#ifndef HEXROUTER_HEXROUTERCOMPONENTIMPLCFG_HPP_
#define HEXROUTER_HEXROUTERCOMPONENTIMPLCFG_HPP_

enum {
  // TODO(mereweth) - move write size to KraitRouter and include
    WRITE_BUFF_SIZE = 256,	// Can't increase this past 256 without causing other issues
    
    READ_BUFF_SIZE = 4*1024		// This is large enough for the circular buffer
};

#endif /* HEXROUTER_HEXROUTERCOMPONENTIMPLCFG_HPP_ */
