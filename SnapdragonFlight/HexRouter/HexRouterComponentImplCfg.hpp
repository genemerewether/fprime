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
      HR_READ_BUFF_SIZE = 4*1024,		// This is large enough for the circular buffer

      // specifically for async processed cycling buffers
      HR_RECEIVE_BUFFER_POOL_SIZE = 4,
      HR_RECEIVE_BUFFER_SIZE = 1024,

      // TODO (mereweth) - set as max of serialized port sizes
      HR_READ_PORT_SIZE = 512
  };

};

#endif /* HEXROUTER_HEXROUTERCOMPONENTIMPLCFG_HPP_ */
