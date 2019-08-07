/*
 * PrmDblImplCfg.hpp
 *
 *  Created on: Mar 13, 2015
 *      Author: tcanham
 */

#ifndef PRMDB_PRMDBLIMPLCFG_HPP_
#define PRMDB_PRMDBLIMPLCFG_HPP_

// Anonymous namespace for configuration parameters
namespace {

    enum {
#ifdef BUILD_TIR5
        PRMDB_NUM_DB_ENTRIES = 200, // !< Number of entries in the parameter database
#else
        PRMDB_NUM_DB_ENTRIES = 1000, // !< Number of entries in the parameter database
#endif
    };

}



#endif /* PRMDB_PRMDBLIMPLCFG_HPP_ */
