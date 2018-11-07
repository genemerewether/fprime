/*
 * HLProcPolyDb.hpp
 *
 *  Created on: Apr 9, 2018
 *      Author: tim
 */

#ifndef HLPROC_CFG_HLPROCPOLYDB_HPP_
#define HLPROC_CFG_HLPROCPOLYDB_HPP_

namespace HLProc {

    // This enum will act as an index for the PolyDb entries for the HLProc
    enum {
        HLPROC_POLYDB_POWER_STATE, // Power state of high-level processor
    };

    enum HLProcPolyDbPowerState {
        HLPROC_POLYDB_POWER_AWAKE, //!< Awake state
        HLPROC_POLYDB_POWER_PREFLIGHT, //!< Pre-flight state
        HLPROC_POLYDB_POWER_FLIGHT, //!< Flight state
        HLPROC_POLYDB_POWER_SLEEP, //!< Sleep state
    };
}
#endif /* HLPROC_CFG_HLPROCPOLYDB_HPP_ */
