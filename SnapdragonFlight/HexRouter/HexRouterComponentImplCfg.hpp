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
      // specifically for async processed cycling buffers
      HR_RECEIVE_BUFFER_POOL_SIZE = 4,
      HR_RECEIVE_BUFFER_SIZE = 1024,

      HR_RPC_READ_SIZE = 1024,

      HR_NO_MSG_SLEEP_US = 10000
  };

};

#endif /* HEXROUTER_HEXROUTERCOMPONENTIMPLCFG_HPP_ */
