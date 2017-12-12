/*
 * HexRouterComponentImplCfg.hpp
 *
 *  Created on: Dec 9, 2017
 *      Author: mereweth
 */

#ifndef HEXROUTER_HEXROUTERCOMPONENTIMPLCFG_HPP_
#define HEXROUTER_HEXROUTERCOMPONENTIMPLCFG_HPP_

namespace SnapdragonFlight {

  enum {
      READ_BUFF_SIZE = 4*1024,		// This is large enough for the circular buffer

      // specifically for async processed cycling buffers
      RECEIVE_BUFFER_POOL_SIZE = 4,
      RECEIVE_BUFFER_SIZE = 1024,

      // TODO (mereweth) - set as max of serialized port sizes
      READ_PORT_SIZE = 512
  };

};

#endif /* HEXROUTER_HEXROUTERCOMPONENTIMPLCFG_HPP_ */
