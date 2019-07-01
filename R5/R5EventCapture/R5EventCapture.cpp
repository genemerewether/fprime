
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

  static Utils::RingBuffer<struct ECFifoEntry,ECFifoEntryDepth> ECFifo;

  // HTU Buffers
  static U32* ECHTUBufferA = nullptr;
  static U32* ECHTUBufferB = nullptr;
  static U32 ECHTUBufferSize = 0;
  static U32 ECHTUBufferEntries = 0;

  // Active HTU CP
  static HTUCP activeCP;

  // RTI-HET Time sync
  static RtiTime ECSyncRtiTime;
  static U32 ECSyncHetTime;


  // Assumes that het_time is within one het count cycles of last Het Timesync
  static void ECTransformTime(U32 hetTime, struct ECFifoEntry& entry_out) {

    U32 hetOffset;
    U32 hetOffsetUsec;
    U32 hetOffsetSubseconds;

    // Het time is stored in the upper 25 bits
    hetTime = hetTime >> 7;

    // TODO: Verify that we don't wrap the HET Counter
    if (hetTime < ECSyncHetTime) {
        hetOffset = ECSyncHetTime - hetTime;
    } else {
        hetOffset = (HET_TIME_MAX - hetTime) + ECSyncHetTime;
    }

    hetOffsetUsec = hetOffset * HET_TICK_PER_US;
    hetOffsetSubseconds = hetOffsetUsec * R5_USEC_DIV;

    RtiTime rtiTemp = ECSyncRtiTime;

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

  static bool ECSwitchBuffer() {

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

    if (activeCP == HTU_CP_A) {
        // No longer busy. Switch to CP B
        htuREG1->CPENA = HTU_CPENA_CPENA_ENB(HTU_DCP);
        activeCP = HTU_CP_B;
    } else {
        // Switch to CB A
        htuREG1->CPENA = HTU_CPENA_CPENA_ENA(HTU_DCP);
        activeCP = HTU_CP_A;
    }

    // Stop holding the bus
    htuREG1->GC &= ~(HTU_GC_VBUSHOLD);

    return true;
  }

  static bool ECReadInactiveBuffer() {
    U32 bufferEntries;
    struct ECFifoEntry fifoEntry;
    U32 idx;
    U32* HTUBufferPtr;

    FW_ASSERT(activeCP == HTU_CP_A ||
              activeCP == HTU_CP_B, activeCP);

    if (activeCP == HTU_CP_A) {
        // Read number of frames transfered into B buffer
        bufferEntries = (htuCDCP1[HTU_DCP].CFCOUNT >> CDCP_CFCOUNT_CFTCTB_SHIFT) &
                            CDCP_CFCOUNT_CFTCTB_MASK;
        HTUBufferPtr = ECHTUBufferB;
    } else {
        // Read number of frames transfered into A buffer
        bufferEntries = (htuCDCP1[HTU_DCP].CFCOUNT >> CDCP_CFCOUNT_CFTCTA_SHIFT) &
                            CDCP_CFCOUNT_CFTCTA_MASK;
        HTUBufferPtr = ECHTUBufferA;
    }

    //FW_ASSERT(bufferEntries <= ECHTUBufferEntries, bufferEntries);

    for (idx = 0; idx < ECHTUBufferEntries; idx++) {
        if (HTUBufferPtr[idx] == 0) {
            break;
        }
        // Read entries and push them to the Fifo
        ECTransformTime(HTUBufferPtr[idx], fifoEntry);

        if (ECFifo.queue(&fifoEntry) != 0) {
            return false;
        }
    }

    memset(HTUBufferPtr, 0, ECHTUBufferSize);

    return true;
  }

  static void ECFifoEntryToEvent(const ECFifoEntry& f_entry, ECEvent& event_out) {
    event_out.seconds = f_entry.seconds;
    event_out.useconds = f_entry.useconds;
  }

  static void ECCalcTimeConversion() {
    rtiGetHighResTime(&ECSyncRtiTime);
    // Het time is stored in the upper 25 bits
    ECSyncHetTime = hetRAM1->Instruction[0].Data >> 7;
  }

  void EventCaptureInit(U8* dmaMemoryA, U8* dmaMemoryB, uint32_t dmaMemorySize) {

    FW_ASSERT(dmaMemoryA != NULL, reinterpret_cast<uint32>(dmaMemoryA));
    FW_ASSERT(dmaMemoryB != NULL, reinterpret_cast<uint32>(dmaMemoryB));

    ECHTUBufferA = reinterpret_cast<U32*>(dmaMemoryA);
    ECHTUBufferB = reinterpret_cast<U32*>(dmaMemoryB);
    ECHTUBufferSize = dmaMemorySize;
    ECHTUBufferEntries = ECHTUBufferSize / sizeof(U32);

    memset(ECHTUBufferA, 0, dmaMemorySize);
    memset(ECHTUBufferB, 0, dmaMemorySize);

    ECFifo.reset();

    // Enable Continue On Request Lost
    htuREG1->RLBECTRL = HTU_RLBECTRL_CORL;

    // Set HTU Buffer locations
    htuDCP1[HTU_DCP].IFADDRA = reinterpret_cast<uint32>(ECHTUBufferA);
    htuDCP1[HTU_DCP].IFADDRB = reinterpret_cast<uint32>(ECHTUBufferB);

    // (Instruction 1 * 4 entries per Instruction) + Data field offset
    htuDCP1[HTU_DCP].IHADDRCT = ((1*4) + 2) << DCP_IHADDRCT_IHADDR_SHIFT;

    htuDCP1[HTU_DCP].ITCOUNT = (1 << DCP_ITCOUNT_IETCOUNT_SHIFT) | // 1 Element per Frame
                               (ECHTUBufferEntries << DCP_ITCOUNT_IFTCOUNT_SHIFT); // ECBufferLength Frames

    // Enable CP 0, buffer A
    htuREG1->CPENA = HTU_CPENA_CPENA_ENA(HTU_DCP);
    activeCP = HTU_CP_A;

    // Enable the HTU
    htuREG1->GC = HTU_GC_HTUEN;

    // Enable HTU requests on line 0
    hetREG1->REQENS = (1 << 0);
  }

  bool EventCaptureGetEvent(struct ECEvent& event) {

    I32 fifo_stat;
    struct ECFifoEntry f_entry;

    fifo_stat = ECFifo.dequeue(&f_entry);

    if (fifo_stat != 0) {
        ECCalcTimeConversion();

        ECSwitchBuffer();

        ECReadInactiveBuffer();

        fifo_stat = ECFifo.dequeue(&f_entry);
        if (fifo_stat != 0) {
            return false;
        }
    }

    ECFifoEntryToEvent(f_entry, event);
    return true;
  }

}
