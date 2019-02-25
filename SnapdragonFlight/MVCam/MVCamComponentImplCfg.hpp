/*
 * MVCamComponentImplCfg.hpp
 *
 *  Created on: May 11, 2017
 *      Author: mereweth
 */

#ifndef MVCAM_MVCAMCOMPONENTIMPLCFG_HPP
#define MVCAM_MVCAMCOMPONENTIMPLCFG_HPP

// for serializable size
#include <Svc/ImgComp/CameraFrameSerializableAc.hpp>

#include "Common/Cfg/QuestConstants.hpp"

//#define ALWAYS_CYCLE

namespace SnapdragonFlight {

enum {
  MVCAM_CAMERA_TYPE = 1
};

// TODO (mereweth@jpl.nasa.gov) - change this to a parameter?
enum {
  MVCAM_MIN_RATE_WARN = 28, // EVR when less than this many images per RTI
  MVCAM_FRAME_SKIP = 0, // 2 //For 10 Hz images on ImageSend port
  MVCAM_IMG_WAIT_SEC = 2, // Wait this many sched calls for an image
  MVCAM_PARAMS_APPLY_SKIP = 1, // Skip # frames before update gain and exposure due to delay in parameter propagation
  MVCAM_MAX_NUM_WAYPOINTS = 100,
  MVCAM_IMG_MAX_NUM_BUFFERS = 2000, // Sized for NAVOUTDOOR
  MVCAM_IMG_HP_BUFFER_POOL_SIZE = 5,
  MVCAM_IMG_LP_BUFFER_POOL_SIZE = MVCAM_IMG_MAX_NUM_BUFFERS,
  MVCAM_IMG_TLM_SKIP = 10,
};

enum {
  MVCAM_IMAGE_WIDTH = 640,
  MVCAM_IMAGE_HEIGHT = 480,

  MVCAM_AOI_WIDTH = 640,  // Area to compute auto-exposure on
  MVCAM_AOI_HEIGHT = 480, // Area to compute auto-exposure on

  MVCAM_AOI_START_COL = (MVCAM_IMAGE_WIDTH - MVCAM_AOI_WIDTH) / 2,
  MVCAM_AOI_START_ROW = (MVCAM_IMAGE_HEIGHT - MVCAM_AOI_HEIGHT) / 2,
  MVCAM_AOI_STRIDE = MVCAM_IMAGE_WIDTH,

  MVCAM_IMAGE_SIZE = MVCAM_IMAGE_WIDTH * MVCAM_IMAGE_HEIGHT, // Save and send 8-bit image

  // align to the chunk size, rounding up
  MVCAM_BUFFER_ALIGN_DIV =  (MVCAM_IMAGE_SIZE + Svc::CameraFrame::SERIALIZED_SIZE) / Cfg::DIRECT_CHUNK_SIZE,
  MVCAM_BUFFER_SIZE = (1 + MVCAM_BUFFER_ALIGN_DIV) * Cfg::DIRECT_CHUNK_SIZE,

  MVCAM_10BIT_CALLBACK_SIZE = MVCAM_IMAGE_SIZE * 10 / 8,
  MVCAM_8BIT_CALLBACK_SIZE = MVCAM_IMAGE_SIZE,
};

const float MVCAM_DEFAULT_ROW_PERIOD_US = 19.3333; // This is used to compute the total exposure time.

} // end namespace SnapdragonFlight

#endif // #ifndef MVCAM_MVCAMCOMPONENTIMPLCFG_HPP
