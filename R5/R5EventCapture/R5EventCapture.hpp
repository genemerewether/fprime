
#ifndef __EVENT_CAPTURE_H__
#define __EVENT_CAPTURE_H__

#include <Fw/Types/BasicTypes.hpp>

namespace R5 {

  struct ECEvent {
      U32 seconds, useconds;
  };

  void EventCaptureInit(U32 fpPort, U8* dmaMemoryA, U8* dmaMemoryB, uint32_t dmaMemorySize);

  bool EventCaptureGetEvent(U32 fpPort, struct ECEvent& event);
}

#endif
