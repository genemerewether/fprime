/*
 * HiresCamComponentImplCfg.hpp
 *
 *  Created on: May 11, 2017
 *      Author: mereweth
 */

#ifndef HIRESCAM_HIRESCAMCOMPONENTIMPLCFG_HPP
#define HIRESCAM_HIRESCAMCOMPONENTIMPLCFG_HPP

// for serializable size
#include <Svc/ImgComp/CameraFrameSerializableAc.hpp>

#include "Common/Cfg/QuestConstants.hpp"

namespace SnapdragonFlight {

enum {
    HIRESCAM_CAMERA_TYPE = 0,
    HIRESCAM_DEACTIVATE_WAIT_SEC = 2,
    HIRESCAM_IMG_PRE_WAIT_SEC = 1,
    HIRESCAM_VID_PRE_WAIT_SEC = 1,
    HIRESCAM_VID_WAIT_SEC = 2,
    HIRESCAM_IMG_WAIT_SEC = 5, // Wait this many seconds for an image when deactivated
    HIRESCAM_MAX_NUM_WAYPOINTS = 100,
    HIRESCAM_MAX_NUM_BUFFERS = 1000,
    HIRESCAM_MIN_RATE_WARN = 10, // Warn if video drops below this FPS
    HIRESCAM_FRAMES_STOP_VID = 300, // Default value for waypoint timeout
    HIRESCAM_SCHED_STOP_VID = 10 // Default value for waypoint timeout
};

enum {
    /*HIRESCAM_13MP_RAW_IMAGE_WIDTH = 4208,
    HIRESCAM_13MP_RAW_IMAGE_HEIGHT = 3120,
    HIRESCAM_13MP_RAW_IMAGE_PADDING_PER_ROW = 4,
    HIRESCAM_13MP_RAW_IMAGE_SIZE = ((HIRESCAM_13MP_RAW_IMAGE_WIDTH * 10) / 8
                                + HIRESCAM_13MP_RAW_IMAGE_PADDING_PER_ROW) * HIRESCAM_13MP_RAW_IMAGE_HEIGHT,*/

    HIRESCAM_13MP_IMAGE_CALLBACK_WIDTH = 4224,
    HIRESCAM_13MP_IMAGE_CALLBACK_SKIP = 67584,
    HIRESCAM_13MP_IMAGE_WIDTH = 4208,
    HIRESCAM_13MP_IMAGE_HEIGHT = 3120,
    HIRESCAM_13MP_IMAGE_CALLBACK_TAIL = 33792,
    HIRESCAM_13MP_IMAGE_CALLBACK_SIZE = HIRESCAM_13MP_IMAGE_CALLBACK_WIDTH
                                    * HIRESCAM_13MP_IMAGE_HEIGHT * 3 / 2
                                    + HIRESCAM_13MP_IMAGE_CALLBACK_SKIP,
    HIRESCAM_13MP_IMAGE_SIZE = HIRESCAM_13MP_IMAGE_CALLBACK_SIZE,

    HIRESCAM_2MP_IMAGE_WIDTH = 1920,
    HIRESCAM_2MP_IMAGE_HEIGHT = 1080,
    HIRESCAM_2MP_IMAGE_CALLBACK_SKIP = 15360,
    HIRESCAM_2MP_IMAGE_CALLBACK_SIZE = HIRESCAM_2MP_IMAGE_WIDTH
                                   * HIRESCAM_2MP_IMAGE_HEIGHT * 3 / 2
                                   + HIRESCAM_2MP_IMAGE_CALLBACK_SKIP,
    HIRESCAM_2MP_IMAGE_SIZE = HIRESCAM_2MP_IMAGE_CALLBACK_SIZE,

    HIRESCAM_VGA_IMAGE_WIDTH = 640,
    HIRESCAM_VGA_IMAGE_HEIGHT = 480,
    HIRESCAM_VGA_IMAGE_CALLBACK_SKIP = 20480,
    HIRESCAM_VGA_IMAGE_CALLBACK_SIZE = HIRESCAM_VGA_IMAGE_WIDTH
                                  * HIRESCAM_VGA_IMAGE_HEIGHT * 3 / 2
                                  + HIRESCAM_VGA_IMAGE_CALLBACK_SKIP,
    HIRESCAM_VGA_IMAGE_SIZE = HIRESCAM_VGA_IMAGE_CALLBACK_SIZE,
};

enum {
    // align to the chunk size, rounding up
    HIRESCAM_BUFFER_ALIGN_DIV = (HIRESCAM_13MP_IMAGE_CALLBACK_SIZE + Svc::CameraFrame::SERIALIZED_SIZE) / Cfg::DIRECT_CHUNK_SIZE,
    HIRESCAM_BUFFER_SIZE = (1 + HIRESCAM_BUFFER_ALIGN_DIV) * Cfg::DIRECT_CHUNK_SIZE,
};

} // end namespace SnapdragonFlight

#endif // #ifndef HIRESCAM_HIRESCAMCOMPONENTIMPLCFG_HPP