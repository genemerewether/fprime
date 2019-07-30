/*
 * ActivePrmDblImplCfg.hpp
 *
 *  Created on: Feb 15, 2019
 *      Author: kubiak
 */

#ifndef ACTIVE_PRMDB_PRMDBLIMPLCFG_HPP_
#define ACTIVE_PRMDB_PRMDBLIMPLCFG_HPP_

// Anonymous namespace for configuration parameters
namespace {

    enum {
        ACTIVE_PRMDB_NUM_RANGES = 64, // !< Number of parameter ranges
        ACTIVE_PRMDB_ENTRY_DELIMETER = 0xA5 // !< Byte value that should precede each parameter in file; sanity check against file integrity. Should match ground system.
    };

}



#endif /* ACTIVE_PRMDB_PRMDBLIMPLCFG_HPP_ */

