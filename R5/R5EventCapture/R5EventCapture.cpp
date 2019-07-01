
#include <R5/TiHal/include/HL_reg_htu.h>
#include <R5/TiHal/include/HL_reg_het.h>
#include <R5/TiHal/include/HL_rti.h>

#include <R5/R5EventCapture/htu_reg.h>
#include <R5/R5EventCapture/R5EventCapture.hpp>
#include <R5/R5EventCapture/R5EventCaptureCfg.hpp>

#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Assert.hpp>

#include <Utils/RingBuffer/RingBuffer.h>

#include <string.h>

#define HTU_DCP 0
#define HET_TIME_MAX 0x2000000

#define HET_TICK_PER_US (0.853333F)

#define R5_USEC_DIV (75)
#define RTI_MAX_SUBSECONDS (R5_USEC_DIV * 1000 * 1000)

namespace R5 {

  struct ECFifoEntry {
    U32 seconds, useconds;
  };

  enum HTUCP {
    HTU_CP_A,
    HTU_CP_B
  };

  struct ECConfig {
    U32 DCP;
    U32 instruction;
  };

  struct ECPort {
    struct ECConfig config;

    Utils::RingBuffer<struct ECFifoEntry,ECFifoEntryDepth> fifo;

    // HTU Buffers
    U32* bufferA;
    U32* bufferB;
    U32 bufferSize;
    U32 bufferEntries;

    // Active HTU CP
    HTUCP activeCP;

    // RTI-HET Time sync
    RtiTime rtiTime;
    U32 hetTime;
  };

  static const struct ECConfig ECConfigs[ECNumEventPins] = {
      // Port 0. STIM300 TOV Event
      {
          .DCP = 0,
          .instruction = 1
      },
      // Port 1. R5 Time Sync Event
      {
          .DCP = 1,
          .instruction = 2
      }
  };

  static struct ECPort ECPorts[ECNumEventPins] = {
      // Port 0. STIM300 TOV Event
      {
          .config = ECConfigs[0]
      },
      {
          .config = ECConfigs[1]
      }
  };

  // Assumes that het_time is within one het count cycles of last Het Timesync
  static void ECTransformTime(struct ECPort* port, U32 hetTime, struct ECFifoEntry& entry_out) {

    U32 hetOffset;
    U32 hetOffsetUsec;
    U32 hetOffsetSubseconds;

    // Het time is stored in the upper 25 bits
    hetTime = hetTime >> 7;

    // TODO: Verify that we don't wrap the HET Counter
    if (hetTime < port->hetTime) {
        hetOffset = port->hetTime - hetTime;
    } else {
        hetOffset = (HET_TIME_MAX - hetTime) + port->hetTime;
    }

    hetOffsetUsec = hetOffset * HET_TICK_PER_US;
    hetOffsetSubseconds = hetOffsetUsec * R5_USEC_DIV;

    RtiTime rtiTemp = port->rtiTime;

    // If the time from when the event happened to the GetEvent
    // call is too large we cannot convert the time reliably.
    // In this case throw and error
    if (hetOffsetSubseconds >= RTI_MAX_SUBSECONDS) {
        entry_out.seconds = 0xFFFFFFFF;
        entry_out.useconds = 0xFFFFFFFF;
        return;
    }

    // TODO: Think about overflow
    if (rtiTemp.subseconds >= hetOffsetSubseconds) {
        rtiTemp.subseconds -= hetOffsetSubseconds;
    } else {
        // TODO: Verify that seconds is >= 1 so we don't wrap.
        rtiTemp.seconds -= 1;
        rtiTemp.subseconds = RTI_MAX_SUBSECONDS - hetOffsetSubseconds;
    }

    entry_out.seconds = rtiTemp.seconds;
    entry_out.useconds = rtiTemp.subseconds / R5_USEC_DIV;
  }

  static bool ECSwitchBuffer(struct ECPort* port) {

    U32 timeout = 0;

    // Hold the VBUS until current transaction is done
    htuREG1->GC |= HTU_GC_VBUSHOLD;

    while (htuREG1->ACPE & HTU_ACPE_BUSBUSY) {
        if (timeout > ECBusyCheckTimeout) {
            htuREG1->GC &= ~(HTU_GC_VBUSHOLD);
            return false;
        }
        timeout++;
    }

    if (port->activeCP == HTU_CP_A) {
        // No longer busy. Switch to CP B
        htuREG1->CPENA = HTU_CPENA_CPENA_ENB(port->config.DCP);
        port->activeCP = HTU_CP_B;
    } else {
        // Switch to CB A
        htuREG1->CPENA = HTU_CPENA_CPENA_ENA(port->config.DCP);
        port->activeCP = HTU_CP_A;
    }

    // Stop holding the bus
    htuREG1->GC &= ~(HTU_GC_VBUSHOLD);

    return true;
  }

  static bool ECReadInactiveBuffer(struct ECPort* port) {
    //U32 bufferEntries;
    struct ECFifoEntry fifoEntry;
    U32 idx;
    U32* HTUBufferPtr;

    FW_ASSERT(port->activeCP == HTU_CP_A ||
              port->activeCP == HTU_CP_B, port->activeCP);

    if (port->activeCP == HTU_CP_A) {
        // Read number of frames transfered into B buffer
        //bufferEntries = (htuCDCP1[port->config.DCP].CFCOUNT >> CDCP_CFCOUNT_CFTCTB_SHIFT) &
                         //CDCP_CFCOUNT_CFTCTB_MASK;
        HTUBufferPtr = port->bufferB;
    } else {
        // Read number of frames transfered into A buffer
        //bufferEntries = (htuCDCP1[port->config.DCP].CFCOUNT >> CDCP_CFCOUNT_CFTCTA_SHIFT) &
                         //CDCP_CFCOUNT_CFTCTA_MASK;
        HTUBufferPtr = port->bufferA;
    }

    //FW_ASSERT(bufferEntries <= ECHTUBufferEntries, bufferEntries);

    for (idx = 0; idx < port->bufferEntries; idx++) {
        if (HTUBufferPtr[idx] == 0) {
            break;
        }
        // Read entries and push them to the Fifo
        ECTransformTime(port, HTUBufferPtr[idx], fifoEntry);

        if (port->fifo.queue(&fifoEntry) != 0) {
            return false;
        }
    }

    memset(HTUBufferPtr, 0, port->bufferSize);

    return true;
  }

  static void ECFifoEntryToEvent(struct ECPort* port, const ECFifoEntry& f_entry, ECEvent& event_out) {
    event_out.seconds = f_entry.seconds;
    event_out.useconds = f_entry.useconds;
  }

  static void ECCalcTimeConversion(struct ECPort* port) {
    rtiGetHighResTime(&port->rtiTime);
    // Het time is stored in the upper 25 bits
    port->hetTime = hetRAM1->Instruction[0].Data >> 7;
  }

  void EventCaptureInit(U32 fpPort, U8* dmaMemoryA, U8* dmaMemoryB, uint32_t dmaMemorySize) {

    FW_ASSERT(fpPort < ECNumEventPins, fpPort, ECNumEventPins);

    FW_ASSERT(dmaMemoryA != NULL, reinterpret_cast<uint32>(dmaMemoryA));
    FW_ASSERT(dmaMemoryB != NULL, reinterpret_cast<uint32>(dmaMemoryB));

    struct ECPort* port = &ECPorts[fpPort];

    port->bufferA = reinterpret_cast<U32*>(dmaMemoryA);
    port->bufferB = reinterpret_cast<U32*>(dmaMemoryB);
    port->bufferSize = dmaMemorySize;
    port->bufferEntries = dmaMemorySize / sizeof(U32);

    memset(port->bufferA, 0, dmaMemorySize);
    memset(port->bufferB, 0, dmaMemorySize);

    port->fifo.reset();

    // Enable Continue On Request Lost
    htuREG1->RLBECTRL = HTU_RLBECTRL_CORL;

    // Set HTU Buffer locations
    htuDCP1[port->config.DCP].IFADDRA = reinterpret_cast<uint32>(port->bufferA);
    htuDCP1[port->config.DCP].IFADDRB = reinterpret_cast<uint32>(port->bufferB);

    // (Instruction 1 * 4 entries per Instruction) + Data field offset
    htuDCP1[port->config.DCP].IHADDRCT = ((port->config.instruction*4) + 2) << DCP_IHADDRCT_IHADDR_SHIFT;

    htuDCP1[port->config.DCP].ITCOUNT = (1 << DCP_ITCOUNT_IETCOUNT_SHIFT) | // 1 Element per Frame
                                        (port->bufferEntries << DCP_ITCOUNT_IFTCOUNT_SHIFT); // ECBufferLength Frames

    // Enable CP 0, buffer A
    htuREG1->CPENA = HTU_CPENA_CPENA_ENA(port->config.DCP);
    port->activeCP = HTU_CP_A;

    // Enable the HTU
    htuREG1->GC = HTU_GC_HTUEN;

    // Enable HTU requests on line 0
    hetREG1->REQENS = (1 << port->config.DCP);
  }

  bool EventCaptureGetEvent(U32 fpPort, struct ECEvent& event) {

    I32 fifo_stat;
    struct ECFifoEntry f_entry;

    FW_ASSERT(fpPort < ECNumEventPins, fpPort, ECNumEventPins);

    struct ECPort* port = &ECPorts[fpPort];

    FW_ASSERT(port->bufferA != NULL);

    fifo_stat = port->fifo.dequeue(&f_entry);

    if (fifo_stat != 0) {
        ECCalcTimeConversion(port);

        ECSwitchBuffer(port);

        ECReadInactiveBuffer(port);

        fifo_stat = port->fifo.dequeue(&f_entry);
        if (fifo_stat != 0) {
            return false;
        }
    }

    ECFifoEntryToEvent(port, f_entry, event);
    return true;
  }

}
