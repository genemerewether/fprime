/*
 * StereoCamComponentImplCfg.hpp
 *
 *  Created on: May 11, 2017
 *      Author: mereweth
 */

#ifndef SCAM_SCAMCOMPONENTIMPLCFG_HPP
#define SCAM_SCAMCOMPONENTIMPLCFG_HPP

// for serializable size
#include <Svc/CameraFrame/CameraFrameSerializableAc.hpp>

#include "Common/Cfg/QuestConstants.hpp"

//#define ALWAYS_CYCLE

namespace SnapdragonFlight {

enum {
#ifdef SOC_8096
  SCAM_CAMERA_TYPE = 2
#else
  SCAM_CAMERA_TYPE = 3
#endif
};

// TODO (mereweth@jpl.nasa.gov) - change this to a parameter?
enum {
  SCAM_MIN_RATE_WARN = 28, // EVR when less than this many images per RTI
  SCAM_FRAME_SKIP = 0, // 2 //For 10 Hz images on ImageSend port
  SCAM_IMG_WAIT_SEC = 2, // Wait this many sched calls for an image
  SCAM_PARAMS_APPLY_SKIP = 1, // Skip # frames before update gain and exposure due to delay in parameter propagation
  SCAM_MAX_NUM_WAYPOINTS = 100,
  SCAM_IMG_MAX_NUM_BUFFERS = 2000, // Sized for NAVOUTDOOR
  SCAM_IMG_HP_BUFFER_POOL_SIZE = 20,
  SCAM_IMG_LP_BUFFER_POOL_SIZE = SCAM_IMG_MAX_NUM_BUFFERS,
  SCAM_IMG_TLM_SKIP = 10,
};

enum {
  SCAM_IMAGE_WIDTH = 1280,
  SCAM_IMAGE_HEIGHT = 480,

  SCAM_AOI_WIDTH = 1280,  // Area to compute auto-exposure on
  SCAM_AOI_HEIGHT = 480, // Area to compute auto-exposure on

  SCAM_AOI_START_COL = (SCAM_IMAGE_WIDTH - SCAM_AOI_WIDTH) / 2,
  SCAM_AOI_START_ROW = (SCAM_IMAGE_HEIGHT - SCAM_AOI_HEIGHT) / 2,
  SCAM_AOI_STRIDE = SCAM_IMAGE_WIDTH,

  SCAM_IMAGE_SIZE = SCAM_IMAGE_WIDTH * SCAM_IMAGE_HEIGHT, // Save and send 8-bit image

  // align to the chunk size, rounding up
  SCAM_BUFFER_ALIGN_DIV =  (SCAM_IMAGE_SIZE + Svc::CameraFrame::SERIALIZED_SIZE) / Cfg::DIRECT_CHUNK_SIZE,
  SCAM_BUFFER_SIZE = (1 + SCAM_BUFFER_ALIGN_DIV) * Cfg::DIRECT_CHUNK_SIZE,

  SCAM_10BIT_CALLBACK_SIZE = SCAM_IMAGE_SIZE * 10 / 8,
  SCAM_8BIT_CALLBACK_SIZE = SCAM_IMAGE_SIZE,
};

const float SCAM_DEFAULT_ROW_PERIOD_US = 19.3333; // This is used to compute the total exposure time.

} // end namespace SnapdragonFlight

#endif // #ifndef SCAM_SCAMCOMPONENTIMPLCFG_HPP
