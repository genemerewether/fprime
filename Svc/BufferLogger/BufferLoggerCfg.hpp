/*
 * BufferLoggerCfg.hpp
 *
 *  Created on: March 13, 2018
 *      Author: mereweth
 */

#ifndef SVC_BUFFERLOGGERCFG_HPP
#define SVC_BUFFERLOGGERCFG_HPP

// #define BL_PREALLOC # preallocate all file blocks at file open

namespace Svc {

enum {
#ifdef BUILD_UT
    BL_CHUNK_SIZE = 3,
#else
    BL_CHUNK_SIZE = 8192000,
#endif
    BL_MAX_DIRECT_CHUNK_SIZE = 1024,
    BL_NUM_OLD_FDS = 20
};

enum BufferLoggerFileMode {
    BL_DIRECT_WRITE, // requires block-aligned buffers (512 bytes on Snapdragon)
    BL_BULK_WRITE,
    BL_LOOPING_WRITE,
    BL_REGULAR_WRITE, // Doesn't wait for write to hit disk!
    BL_WRITE_MODE_MAX
};

enum BufferLoggerCloseMode {
    BL_CLOSE_SYNC,
    BL_CLOSE_ASYNC,
    BL_CLOSE_LEAK
};

} // namespace Svc

#endif // SVC_BUFFERLOGGERCFG_HPP
