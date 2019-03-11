/*
 * BlspSerialDriverComponentImplCfg.hpp
 *
 *  Created on: Nov 29, 2016
 *      Author: tcanham
 */

#ifndef BLSPSERIALDRIVER_BLSPSERIALDRIVERCOMPONENTIMPLCFG_HPP_
#define BLSPSERIALDRIVER_BLSPSERIALDRIVERCOMPONENTIMPLCFG_HPP_

enum {
    MAX_NUM_BUFFERS = 20,		// Increasing this from 10 b/c RceAdapter couldn't always keep up with just 10 for reads
    WRITE_BUFF_SIZE = 256,	// Can't increase this past 256 without causing other issues
    READ_BUFF_SIZE = 4*1024		// This is large enough for the circular buffer
};

#endif /* BLSPSERIALDRIVER_BLSPSERIALDRIVERCOMPONENTIMPLCFG_HPP_ */
