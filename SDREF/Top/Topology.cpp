#include <ros/ros.h>

#include <Components.hpp>

enum {
    CORE_NONE = -1,
    CORE_0 = 0,
    CORE_1 = 1,
    CORE_2 = 2,
    CORE_3 = 3,

    CORE_CDH = CORE_0,
    CORE_GNC = CORE_1,
    CORE_DEV = CORE_1,
    CORE_PRCP = CORE_3
};

#include <Fw/Types/Assert.hpp>
#include <Os/Task.hpp>
#include <Os/Log.hpp>
#include <Fw/Types/MallocAllocator.hpp>
#include <Fw/Types/MmapAllocator.hpp>

#if defined TGT_OS_TYPE_LINUX || TGT_OS_TYPE_DARWIN
#include <getopt.h>
#include <stdlib.h>
#include <ctype.h>
#endif

#ifdef BUILD_SDFLIGHT
#include <HEXREF/Rpc/hexref.h>
#endif

#if defined TGT_OS_TYPE_LINUX
#include <sys/wait.h>
#endif

#define DEBUG_PRINT(x,...) printf(x,##__VA_ARGS__); fflush(stdout)
//#define DEBUG_PRINT(x,...)

#define PRM_PATH "/eng/SDREFPrmDb.dat"

// Registry
#if FW_OBJECT_REGISTRATION == 1
static Fw::SimpleObjRegistry simpleReg;
#endif

Fw::MallocAllocator seqMallocator;
Fw::MallocAllocator buffMallocator;
Fw::MmapAllocator hiresMallocator;

Svc::RateGroupDriverImpl* rgDrv_ptr = 0;
Svc::ActiveRateGroupImpl* rgTlm_ptr = 0;
Svc::ActiveRateGroupImpl* rgXfer_ptr = 0;

Svc::SocketGndIfImpl* sockGndIf_ptr = 0;
Svc::SocketGndIfImpl* sockGndIfLL_ptr = 0;
Svc::ActiveTextLoggerComponentImpl* textLogger_ptr = 0;
Svc::ActiveLoggerImpl* eventLogger_ptr = 0;
Svc::ActiveLoggerImpl* eventLoggerLL_ptr = 0;
Svc::ActiveFileLoggerImpl* fileLogger_ptr = 0;
Svc::SerLoggerComponentImpl* serLogger_ptr = 0;
Svc::LinuxTimeImpl* linuxTime_ptr = 0;
Svc::TlmChanImpl* chanTlm_ptr = 0;
Svc::CommandDispatcherImpl* cmdDisp_ptr = 0;
Svc::CmdSequencerComponentImpl* cmdSeq_ptr = 0;
Svc::CmdSequencerComponentImpl* cmdSeq2_ptr = 0;
Svc::ActiveL1PrmDbComponentImpl* prmDb_ptr = 0;
Svc::SerialTextConverterComponentImpl* serialTextConv_ptr = 0;
Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr = 0;
Svc::FatalHandlerComponentImpl* fatalHandler_ptr = 0;

SnapdragonFlight::HexRouterComponentImpl* hexRouter_ptr = 0;
SnapdragonFlight::MVCamComponentImpl* mvCam_ptr = 0;
SnapdragonFlight::StereoCamComponentImpl* stereoCam_ptr = 0;
SnapdragonFlight::MVVislamComponentImpl* mvVislam_ptr = 0;
SnapdragonFlight::MVDFSComponentImpl* mvDFS_ptr = 0;
SnapdragonFlight::HiresCamComponentImpl* hiresCam_ptr = 0;
SnapdragonFlight::SnapdragonHealthComponentImpl* snapHealth_ptr = 0;
HLProc::LLRouterComponentImpl* llRouter_ptr = 0;
HLProc::HLRosIfaceComponentImpl* sdRosIface_ptr = 0;
Gnc::MultirotorCtrlIfaceComponentImpl* mrCtrlIface_ptr = 0;
Gnc::FilterIfaceComponentImpl* filterIface_ptr = 0;
ROS::RosSeqComponentImpl* rosSeq_ptr = 0;
HLProc::EventExpanderComponentImpl* eventExp_ptr = 0;
Svc::UdpReceiverComponentImpl* udpReceiver_ptr = 0;

Drv::ATINetboxComponentImpl* atiNetbox_ptr = 0;

Drv::LinuxSerialDriverComponentImpl* serialDriverLL_ptr = 0;
Drv::LinuxSerialDriverComponentImpl* serialDriverDebug_ptr = 0;
SnapdragonFlight::BlspSerialDriverComponentImpl* blspSerialDriverLL_ptr = 0;
SnapdragonFlight::BlspSerialDriverComponentImpl* blspSerialDriverDebug_ptr = 0;
Drv::LinuxGpioDriverComponentImpl* gpioTimeSync_ptr = 0;
SnapdragonFlight::BlspGpioDriverComponentImpl* blspGpioTimeSync_ptr = 0;
Svc::IPCRelayComponentImpl* ipcRelay_ptr = 0;

Svc::TimeConvertComponentImpl* timeConvert_ptr = 0;
Svc::TimeSyncOffsetComponentImpl* llTimeSync_ptr = 0;
SnapdragonFlight::DspOffsetComponentImpl* dspTimeSync_ptr = 0;

Svc::ImgTlmComponentImpl* imgTlm_ptr = 0;

Svc::BufferLogger* buffLogMVCamUnproc_ptr = 0;
Svc::BufferLogger* buffLogHiresCamUnproc_ptr = 0;
Svc::BufferLogger* buffLogStereoCamUnproc_ptr = 0;
Svc::BufferAccumulator* buffAccumMVCamUnproc_ptr = 0;
Svc::BufferAccumulator* buffAccumHiresCamUnproc_ptr = 0;
Svc::BufferAccumulator* buffAccumStereoCamUnproc_ptr = 0;

ewok::EwokComponentImpl* ewok_ptr = 0;

void allocComps() {
    // Component instance pointers
    NATIVE_INT_TYPE rgDivs[] = {30, 1};
    rgDrv_ptr = new Svc::RateGroupDriverImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDRV",
#endif
                        rgDivs,FW_NUM_ARRAY_ELEMENTS(rgDivs));

    NATIVE_UINT_TYPE rgTlmContext[Svc::ActiveRateGroupImpl::CONTEXT_SIZE] = { 0 };
    rgTlm_ptr = new Svc::ActiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                        "RGTLM",
#endif
                        rgTlmContext,FW_NUM_ARRAY_ELEMENTS(rgTlmContext));

    NATIVE_UINT_TYPE rgXferContext[Svc::ActiveRateGroupImpl::CONTEXT_SIZE] = { 0 };
    rgXfer_ptr = new Svc::ActiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                        "RGXFER",
#endif
                        rgXferContext,FW_NUM_ARRAY_ELEMENTS(rgXferContext));

    sockGndIf_ptr = new Svc::SocketGndIfImpl
#if FW_OBJECT_NAMES == 1
                        ("SGIF")
#endif
;

    sockGndIfLL_ptr = new Svc::SocketGndIfImpl
#if FW_OBJECT_NAMES == 1
                        ("SGIFLL")
#endif
;

    textLogger_ptr = new Svc::ActiveTextLoggerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("TLOG")
#endif
;

    eventLogger_ptr = new Svc::ActiveLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("ELOG")
#endif
;

    eventLoggerLL_ptr = new Svc::ActiveLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("ELOGLL")
#endif
;

    fileLogger_ptr = new Svc::ActiveFileLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("FLOG")
#endif
;

    serLogger_ptr = new Svc::SerLoggerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SERLOG")
#endif
;

    linuxTime_ptr = new Svc::LinuxTimeImpl
#if FW_OBJECT_NAMES == 1
                        ("LTIME")
#endif
;

    chanTlm_ptr = new Svc::TlmChanImpl
#if FW_OBJECT_NAMES == 1
                        ("TLM")
#endif
;

    cmdDisp_ptr = new Svc::CommandDispatcherImpl
#if FW_OBJECT_NAMES == 1
                        ("CMDDISP")
#endif
;

    cmdSeq_ptr = new Svc::CmdSequencerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("CMDSEQ")
#endif
;

    cmdSeq2_ptr = new Svc::CmdSequencerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("CMDSEQ2")
#endif
;

    prmDb_ptr = new Svc::ActiveL1PrmDbComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("PRM",PRM_PATH, 1024) // 1024 max recv
#else
                        (PRM_PATH)
#endif
;

    snapHealth_ptr = new SnapdragonFlight::SnapdragonHealthComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SDHEALTH")
#endif
;

    mvCam_ptr = new SnapdragonFlight::MVCamComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("MVCAM")
#endif
;

    stereoCam_ptr = new SnapdragonFlight::StereoCamComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SCAM")
#endif
;

    mvVislam_ptr = new SnapdragonFlight::MVVislamComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("MVVISLAM")
#endif
;

    mvDFS_ptr = new SnapdragonFlight::MVDFSComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("MVDFS")
#endif
;

    ipcRelay_ptr = new Svc::IPCRelayComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("IPCRELAY")
#endif
;

    hiresCam_ptr = new SnapdragonFlight::HiresCamComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("HIRESCAM")
#endif
;

    imgTlm_ptr = new Svc::ImgTlmComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("IMGTLM")
#endif
;

    buffLogMVCamUnproc_ptr = new Svc::BufferLogger
#if FW_OBJECT_NAMES == 1
                        ("BUFLOG_M_U")
#endif
;

    buffLogHiresCamUnproc_ptr = new Svc::BufferLogger
#if FW_OBJECT_NAMES == 1
                        ("BUFLOG_H_U")
#endif
;

    buffLogStereoCamUnproc_ptr = new Svc::BufferLogger
#if FW_OBJECT_NAMES == 1
                        ("BUFLOG_S_U")
#endif
;

    buffAccumMVCamUnproc_ptr = new Svc::BufferAccumulator
#if FW_OBJECT_NAMES == 1
                        ("BUFACC_M_U")
#endif
;

    buffAccumHiresCamUnproc_ptr = new Svc::BufferAccumulator
#if FW_OBJECT_NAMES == 1
                        ("BUFACC_H_U")
#endif
;

    buffAccumStereoCamUnproc_ptr = new Svc::BufferAccumulator
#if FW_OBJECT_NAMES == 1
                        ("BUFACC_S_U")
#endif
;

    hexRouter_ptr = new SnapdragonFlight::HexRouterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("HEXRTR")
#endif
;

    llRouter_ptr = new HLProc::LLRouterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("LLROUTER")
#endif
;

    atiNetbox_ptr = new Drv::ATINetboxComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("ATINETBOX")
#endif
;

    serialDriverLL_ptr = new Drv::LinuxSerialDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SERIALDRVLL")
#endif
;

    serialDriverDebug_ptr = new Drv::LinuxSerialDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SERIALDRVDBUG")
#endif
;

    blspSerialDriverLL_ptr = new SnapdragonFlight::BlspSerialDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("BSERDRVLL")
#endif
;

    blspSerialDriverDebug_ptr = new SnapdragonFlight::BlspSerialDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("BSERDRVDBUG")
#endif
;

    gpioTimeSync_ptr = new Drv::LinuxGpioDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("GSYNC")
#endif
;

    blspGpioTimeSync_ptr = new SnapdragonFlight::BlspGpioDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("BGSYNC")
#endif
;

    timeConvert_ptr = new Svc::TimeConvertComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("TIMECONV")
#endif
;

    llTimeSync_ptr = new Svc::TimeSyncOffsetComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("LLSYNC")
#endif
;

    dspTimeSync_ptr = new SnapdragonFlight::DspOffsetComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("DSPSYNC")
#endif
;

    serialTextConv_ptr = new Svc::SerialTextConverterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("STCONVERTER")
#endif
;

    rosSeq_ptr = new ROS::RosSeqComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("ROSSEQ")
#endif
;

    sdRosIface_ptr = new HLProc::HLRosIfaceComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SDROSIFACE")
#endif
;

    mrCtrlIface_ptr = new Gnc::MultirotorCtrlIfaceComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("MRCTRLIFACE")
#endif
;

    filterIface_ptr = new Gnc::FilterIfaceComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("FILTIFACE")
#endif
;

    eventExp_ptr = new HLProc::EventExpanderComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("EVEXP")
#endif
;

    fatalAdapter_ptr = new Svc::AssertFatalAdapterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("fatalAdapter")
#endif
;

    fatalHandler_ptr = new Svc::FatalHandlerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("fatalHandler")
#endif
;

    udpReceiver_ptr = new Svc::UdpReceiverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("UDPRECV")
#endif
;

    ewok_ptr = new ewok::EwokComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("EWOK")
#endif
;
}

#if FW_OBJECT_REGISTRATION == 1

void dumparch(void) {
    simpleReg.dump();
}

#if FW_OBJECT_NAMES == 1
void dumpobj(const char* objName) {
    simpleReg.dump(objName);
}
#endif

#endif

void manualConstruct(bool llRouterDevices,
                     bool gncCamConnect,
                     bool gncCloudConnect) {
    if (!llRouterDevices) {
        // Sequence Com buffer and cmd response
        cmdSeq_ptr->set_comCmdOut_OutputPort(1, hexRouter_ptr->get_KraitPortsIn_InputPort(0));
        hexRouter_ptr->set_HexPortsOut_OutputPort(0, cmdSeq_ptr->get_cmdResponseIn_InputPort(1));

        // L1 <-> L2 PrmDb
        prmDb_ptr->set_sendPrm_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(10));
        hexRouter_ptr->set_HexPortsOut_OutputPort(10, prmDb_ptr->get_recvPrm_InputPort(0));
        prmDb_ptr->set_recvPrmReady_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(11));
        hexRouter_ptr->set_HexPortsOut_OutputPort(11, prmDb_ptr->get_sendPrmReady_InputPort(0));

        hexRouter_ptr->set_HexPortsOut_OutputPort(1, mvVislam_ptr->get_Imu_InputPort(0));
        hexRouter_ptr->set_HexPortsOut_OutputPort(2, filterIface_ptr->get_Odometry_InputPort(0));
        hexRouter_ptr->set_HexPortsOut_OutputPort(3, mrCtrlIface_ptr->get_AccelCommand_InputPort(0));
        hexRouter_ptr->set_HexPortsOut_OutputPort(4, eventExp_ptr->get_LogRecv_InputPort(0));
        hexRouter_ptr->set_HexPortsOut_OutputPort(5, sockGndIfLL_ptr->get_downlinkPort_InputPort(0));
        hexRouter_ptr->set_HexPortsOut_OutputPort(6, serLogger_ptr->get_SerPortIn_InputPort(0));
        // port 7 only used with LLRouter (timesync)
        //hexRouter_ptr->set_LLPortsOut_OutputPort(9, sdRosIface_ptr->get_Range_InputPort(0));

        rgXfer_ptr->set_RateGroupMemberOut_OutputPort(Svc::ActiveRateGroupImpl::CONTEXT_SIZE-1,
                                                      hexRouter_ptr->get_Sched_InputPort(0));

        sdRosIface_ptr->set_ActuatorsData_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(2));
        sdRosIface_ptr->set_ActuatorsData_OutputPort(1, hexRouter_ptr->get_KraitPortsIn_InputPort(3));
        sockGndIfLL_ptr->set_uplinkPort_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(4));
        mrCtrlIface_ptr->set_flatOutput_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(5));
        mrCtrlIface_ptr->set_attRateThrust_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(6));
        udpReceiver_ptr->set_PortsOut_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(7));

        cmdSeq2_ptr->set_comCmdOut_OutputPort(1, hexRouter_ptr->get_KraitPortsIn_InputPort(8));
        hexRouter_ptr->set_HexPortsOut_OutputPort(8, cmdSeq2_ptr->get_cmdResponseIn_InputPort(1));

        mrCtrlIface_ptr->set_boolStamped_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(9));

        rgTlm_ptr->set_RateGroupMemberOut_OutputPort(Svc::ActiveRateGroupImpl::CONTEXT_SIZE-1,
                                                     dspTimeSync_ptr->get_SchedIn_InputPort(0));
        dspTimeSync_ptr->set_ClockTimes_OutputPort(0, timeConvert_ptr->get_ClockTimes_InputPort(0));
    }
    else { // llRouterDevices
        // Sequence Com buffer and cmd response
        cmdSeq_ptr->set_comCmdOut_OutputPort(1, llRouter_ptr->get_HLPortsIn_InputPort(0));
        llRouter_ptr->set_LLPortsOut_OutputPort(0, cmdSeq_ptr->get_cmdResponseIn_InputPort(1));

        // L1 <-> L2 PrmDb
        prmDb_ptr->set_sendPrm_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(10));
        llRouter_ptr->set_LLPortsOut_OutputPort(10, prmDb_ptr->get_recvPrm_InputPort(0));
        prmDb_ptr->set_recvPrmReady_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(11));
        llRouter_ptr->set_LLPortsOut_OutputPort(11, prmDb_ptr->get_sendPrmReady_InputPort(0));

        llRouter_ptr->set_LLPortsOut_OutputPort(1, mvVislam_ptr->get_Imu_InputPort(0));
        llRouter_ptr->set_LLPortsOut_OutputPort(2, filterIface_ptr->get_Odometry_InputPort(0));
        llRouter_ptr->set_LLPortsOut_OutputPort(3, mrCtrlIface_ptr->get_AccelCommand_InputPort(0));
        llRouter_ptr->set_LLPortsOut_OutputPort(4, eventExp_ptr->get_LogRecv_InputPort(0));
        llRouter_ptr->set_LLPortsOut_OutputPort(5, sockGndIfLL_ptr->get_downlinkPort_InputPort(0));
        llRouter_ptr->set_LLPortsOut_OutputPort(6, serLogger_ptr->get_SerPortIn_InputPort(0));
        llRouter_ptr->set_LLPortsOut_OutputPort(7, llTimeSync_ptr->get_LLTime_InputPort(0));
        llRouter_ptr->set_LLPortsOut_OutputPort(9, sdRosIface_ptr->get_Range_InputPort(0));

        sdRosIface_ptr->set_ActuatorsData_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(2));
        sdRosIface_ptr->set_ActuatorsData_OutputPort(1, llRouter_ptr->get_HLPortsIn_InputPort(3));
        sockGndIfLL_ptr->set_uplinkPort_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(4));
        mrCtrlIface_ptr->set_flatOutput_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(5));
        mrCtrlIface_ptr->set_attRateThrust_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(6));
        udpReceiver_ptr->set_PortsOut_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(7));

        cmdSeq2_ptr->set_comCmdOut_OutputPort(1, llRouter_ptr->get_HLPortsIn_InputPort(8));
        llRouter_ptr->set_LLPortsOut_OutputPort(8, cmdSeq2_ptr->get_cmdResponseIn_InputPort(1));

        mrCtrlIface_ptr->set_boolStamped_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(9));

        rgTlm_ptr->set_RateGroupMemberOut_OutputPort(Svc::ActiveRateGroupImpl::CONTEXT_SIZE-1,
                                                     llTimeSync_ptr->get_SchedIn_InputPort(0));
        llTimeSync_ptr->set_ClockTimes_OutputPort(0, timeConvert_ptr->get_ClockTimes_InputPort(0));

        //llTimeSync_ptr->set_GPIOPulse_OutputPort(0, blspGpioTimeSync_ptr->get_gpioWrite_InputPort(0));
        llTimeSync_ptr->set_GPIOPulse_OutputPort(0, gpioTimeSync_ptr->get_gpioWrite_InputPort(0));
    }//LLROUTER_DEVICES

#ifdef BUILD_SDFLIGHT
    llRouter_ptr->set_SerialBufferSend_OutputPort(0, blspSerialDriverLL_ptr->get_readBufferSend_InputPort(0));
    llRouter_ptr->set_SerWritePort_OutputPort(0, blspSerialDriverLL_ptr->get_serialSend_InputPort(0));
    blspSerialDriverLL_ptr->set_serialRecv_OutputPort(0, llRouter_ptr->get_SerReadPort_InputPort(0));
    
    serialTextConv_ptr->set_SerialBufferSend_OutputPort(0, blspSerialDriverDebug_ptr->get_readBufferSend_InputPort(0));
    blspSerialDriverDebug_ptr->set_serialRecv_OutputPort(0, serialTextConv_ptr->get_SerReadPort_InputPort(0));
#else // BUILD_SDFLIGHT
    llRouter_ptr->set_SerialBufferSend_OutputPort(0, serialDriverLL_ptr->get_readBufferSend_InputPort(0));
    llRouter_ptr->set_SerWritePort_OutputPort(0, serialDriverLL_ptr->get_serialSend_InputPort(0));
    serialDriverLL_ptr->set_serialRecv_OutputPort(0, llRouter_ptr->get_SerReadPort_InputPort(0));
    
    serialTextConv_ptr->set_SerialBufferSend_OutputPort(0, serialDriverDebug_ptr->get_readBufferSend_InputPort(0));
    serialDriverDebug_ptr->set_serialRecv_OutputPort(0, serialTextConv_ptr->get_SerReadPort_InputPort(0));
#endif //BUILD_SDFLIGHT
    
    hiresCam_ptr->set_CmdStatus_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(0));
    // doesn't matter which compCmdStat port # for cmdDisp
    ipcRelay_ptr->set_proc2Out_OutputPort(0, cmdDisp_ptr->get_compCmdStat_InputPort(0));

    hiresCam_ptr->set_Tlm_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(1));
    // doesn't matter which TlmRecv port # for pktTlm
    ipcRelay_ptr->set_proc2Out_OutputPort(1, chanTlm_ptr->get_TlmRecv_InputPort(0));

    hiresCam_ptr->set_Log_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(2));
    // doesn't matter which LogRecv port # for eventLogger
    ipcRelay_ptr->set_proc2Out_OutputPort(2, eventLogger_ptr->get_LogRecv_InputPort(0));

    hiresCam_ptr->set_LogText_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(3));
    // doesn't matter which TextLogger port # for textLogger
    ipcRelay_ptr->set_proc2Out_OutputPort(3, textLogger_ptr->get_TextLogger_InputPort(0));

    /*hiresCam_ptr->set_ProcSend_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(4));
      ipcRelay_ptr->set_proc2Out_OutputPort(4, buffAccumHiresCamProc_ptr->get_bufferSendInFill_InputPort(0));*/

    hiresCam_ptr->set_UnprocSend_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(5));
    ipcRelay_ptr->set_proc2Out_OutputPort(5, buffAccumHiresCamUnproc_ptr->get_bufferSendInFill_InputPort(0));


    stereoCam_ptr->set_CmdStatus_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(6));
    // doesn't matter which compCmdStat port # for cmdDisp
    ipcRelay_ptr->set_proc2Out_OutputPort(6, cmdDisp_ptr->get_compCmdStat_InputPort(0));

    stereoCam_ptr->set_Tlm_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(7));
    // doesn't matter which TlmRecv port # for pktTlm
    ipcRelay_ptr->set_proc2Out_OutputPort(7, chanTlm_ptr->get_TlmRecv_InputPort(0));

    stereoCam_ptr->set_Log_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(8));
    // doesn't matter which LogRecv port # for eventLogger
    ipcRelay_ptr->set_proc2Out_OutputPort(8, eventLogger_ptr->get_LogRecv_InputPort(0));

    stereoCam_ptr->set_LogText_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(9));
    // doesn't matter which TextLogger port # for textLogger
    ipcRelay_ptr->set_proc2Out_OutputPort(9, textLogger_ptr->get_TextLogger_InputPort(0));

    /*stereoCam_ptr->set_ProcSend_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(10));
      ipcRelay_ptr->set_proc2Out_OutputPort(10, buffAccumstereoCamProc_ptr->get_bufferSendInFill_InputPort(0));*/

    stereoCam_ptr->set_UnprocSend_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(11));
    ipcRelay_ptr->set_proc2Out_OutputPort(11, buffAccumStereoCamUnproc_ptr->get_bufferSendInFill_InputPort(0));

    if (gncCamConnect) {
        mvCam_ptr->set_GncBufferSend_OutputPort(0, mvVislam_ptr->get_ImageIn_InputPort(0));
        mvVislam_ptr->set_ImageBufferReturn_OutputPort(0, mvCam_ptr->get_GncBufferReturn_InputPort(0));


        stereoCam_ptr->set_GncBufferSend_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(12));
        ipcRelay_ptr->set_proc2Out_OutputPort(12, mvDFS_ptr->get_ImageIn_InputPort(0));

        mvDFS_ptr->set_ImageBufferReturn_OutputPort(0, stereoCam_ptr->get_GncBufferAsyncReturn_InputPort(0));

        mvVislam_ptr->set_ImuStateUpdate_OutputPort(1, filterIface_ptr->get_ImuStateUpdateReport_InputPort(0));
        if (llRouterDevices) {
            mvVislam_ptr->set_ImuStateUpdate_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(1));
        } else {
            mvVislam_ptr->set_ImuStateUpdate_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(1));
        }
    }
    else {
        mvCam_ptr->set_GncBufferSend_OutputPort(0, sdRosIface_ptr->get_ImageRecv_InputPort(0));
        sdRosIface_ptr->set_ImageForward_OutputPort(0, mvCam_ptr->get_GncBufferReturn_InputPort(0));


        stereoCam_ptr->set_GncBufferSend_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(12));
        ipcRelay_ptr->set_proc2Out_OutputPort(12, sdRosIface_ptr->get_ImageRecv_InputPort(1));

        sdRosIface_ptr->set_ImageForward_OutputPort(1, stereoCam_ptr->get_GncBufferAsyncReturn_InputPort(0));

        if (llRouterDevices) {
            filterIface_ptr->set_ImuStateUpdate_OutputPort(0, llRouter_ptr->get_HLPortsIn_InputPort(1));
        } else {
            filterIface_ptr->set_ImuStateUpdate_OutputPort(0, hexRouter_ptr->get_KraitPortsIn_InputPort(1));
        }
    }
    
    stereoCam_ptr->set_ParamSet_OutputPort(0, ipcRelay_ptr->get_proc1In_InputPort(13));
    ipcRelay_ptr->set_proc2Out_OutputPort(13, prmDb_ptr->get_setPrm_InputPort(0));

    if (gncCloudConnect) {
        mvDFS_ptr->set_PointCloud_OutputPort(0, ewok_ptr->get_PointCloud_InputPort(0));
        //mvVislam_ptr->set_PointCloud_OutputPort(0, ewok_ptr->get_PointCloud_InputPort(1));
    }
    else {
        mvDFS_ptr->set_PointCloud_OutputPort(0, sdRosIface_ptr->get_PointCloud_InputPort(0));
    }
    mvVislam_ptr->set_PointCloud_OutputPort(0, sdRosIface_ptr->get_PointCloud_InputPort(1));
}

void constructApp(unsigned int port_number, unsigned int ll_port_number,
                  char* udp_recv_string, char* udp_send_string,
                  char* zmq_image_string,
                  char* hostname,
                  unsigned int boot_count,
                  bool &isHiresChild, bool &isStereoChild,
                  bool llRouterDevices, bool gncCamConnect, bool gncCloudConnect,
                  bool startSocketNow) {
    allocComps();

#if FW_PORT_TRACING
    Fw::PortBase::setTrace(false);
#endif

    // Initialize rate group driver
    rgDrv_ptr->init();

    // Initialize the rate groups
    rgTlm_ptr->init(60,0);
    rgXfer_ptr->init(60,0);

#if FW_ENABLE_TEXT_LOGGING
    // Queue needs to be large enough to process many text messages at once
    textLogger_ptr->init(500);
#endif

    eventLogger_ptr->init(500,0);
    eventLoggerLL_ptr->init(500,0);
    fileLogger_ptr->init(60);
    serLogger_ptr->init(0);
    serLogger_ptr->setStreamId(AFL_ACTADAP_ESC);

    linuxTime_ptr->init(0);

    chanTlm_ptr->init(60,0);

    cmdDisp_ptr->init(60,0);

    cmdSeq_ptr->init(60,0);
    cmdSeq_ptr->allocateBuffer(0,seqMallocator,100*1024);

    cmdSeq2_ptr->init(60,0);
    cmdSeq2_ptr->allocateBuffer(0,seqMallocator,100*1024);

    prmDb_ptr->init(60,0);
    snapHealth_ptr->init(60,0);
    snapHealth_ptr->setBootCount(boot_count);
    snapHealth_ptr->setInitPowerState(SnapdragonFlight::SH_SAVER_DYNAMIC);

    atiNetbox_ptr->init();

    sockGndIf_ptr->init(0);
    sockGndIfLL_ptr->init(1);

    eventExp_ptr->init(0);

    fatalAdapter_ptr->init(0);
    fatalHandler_ptr->init(0);

    imgTlm_ptr->init(60, 0);

    buffLogMVCamUnproc_ptr->init(60, 0);
    buffLogHiresCamUnproc_ptr->init(60, 0);
    buffLogStereoCamUnproc_ptr->init(60, 0);
    buffAccumMVCamUnproc_ptr->init(60, 0);
    buffAccumHiresCamUnproc_ptr->init(60, 0);
    buffAccumStereoCamUnproc_ptr->init(60, 0);

    mvCam_ptr->init(60, 0);
    mvVislam_ptr->init(2000, 0);
    mvDFS_ptr->init(60, 0);
    ipcRelay_ptr->init(60, IPC_RELAY_BUFFER_SIZE, 0);
    hiresCam_ptr->init(60, 0);
    stereoCam_ptr->init(60, 0);
    hexRouter_ptr->init(60, 1000); // message size
    
    sdRosIface_ptr->init(0);
    mrCtrlIface_ptr->init(0);
    filterIface_ptr->init(0);
    rosSeq_ptr->init(0);
    ewok_ptr->init(0);

    serialTextConv_ptr->init(60,0);
    llRouter_ptr->init(60,SERIAL_BUFFER_SIZE,0);
    serialDriverLL_ptr->init();
    serialDriverDebug_ptr->init();

    blspSerialDriverLL_ptr->init();
    blspSerialDriverDebug_ptr->init();
    gpioTimeSync_ptr->init();
    blspGpioTimeSync_ptr->init();
    timeConvert_ptr->init();
    llTimeSync_ptr->init(60, 0);
    dspTimeSync_ptr->init(60, 0);

    udpReceiver_ptr->init(0);
    
    // Connect rate groups to rate group driver
    constructSDREFArchitecture();

    manualConstruct(llRouterDevices, gncCamConnect, gncCloudConnect);

    const U32 tempPortNum[2] = {1, 0};
    const FwOpcodeType tempMinOpcode[2] = {0, 20000};
    const FwOpcodeType tempMaxOpcode[2] = {19999, 39999};
    cmdSeq_ptr->setOpCodeRanges(2,
                                tempPortNum,
                                tempMinOpcode,
                                tempMaxOpcode);
    cmdSeq2_ptr->setOpCodeRanges(2,
                                 tempPortNum,
                                 tempMinOpcode,
                                 tempMaxOpcode);

    /* Register commands */
    sockGndIf_ptr->regCommands();
    sockGndIfLL_ptr->regCommands();
    cmdSeq_ptr->regCommands();
    cmdSeq2_ptr->regCommands();
    cmdDisp_ptr->regCommands();
    eventLogger_ptr->regCommands();
    eventLoggerLL_ptr->regCommands();
    fileLogger_ptr->regCommands();
    prmDb_ptr->regCommands();
    snapHealth_ptr->regCommands();
    mvCam_ptr->regCommands();
    stereoCam_ptr->regCommands();
    mvVislam_ptr->regCommands();
    mvDFS_ptr->regCommands();
    hiresCam_ptr->regCommands();
    atiNetbox_ptr->regCommands();
    fatalHandler_ptr->regCommands();

    buffLogMVCamUnproc_ptr->regCommands();
    buffLogHiresCamUnproc_ptr->regCommands();
    buffLogStereoCamUnproc_ptr->regCommands();
    buffAccumMVCamUnproc_ptr->regCommands();
    buffAccumHiresCamUnproc_ptr->regCommands();
    buffAccumStereoCamUnproc_ptr->regCommands();

    llRouter_ptr->regCommands();
    serialTextConv_ptr->regCommands();

    // initialize file logs
    fileLogger_ptr->initLog("/log/");

    if (zmq_image_string) {
        imgTlm_ptr->open(zmq_image_string);
    }

    // read parameters

    struct Svc::ActiveL1PrmDbComponentImpl::PrmDbRange L2Ranges[] = {
        {0, 0, 19999}
    };
    prmDb_ptr->setPrmDbRanges(L2Ranges, FW_NUM_ARRAY_ELEMENTS(L2Ranges));
    prmDb_ptr->readPrmFile();

    mvCam_ptr->loadParameters();
    mvVislam_ptr->loadParameters();
    mvDFS_ptr->loadParameters();
    atiNetbox_ptr->loadParameters();
    stereoCam_ptr->loadParameters();

    // NOTE(mereweth) - convert to TB_PROC_TIME for F' outputs of components with time conversion
    mrCtrlIface_ptr->setTBDes(TB_PROC_TIME);
    sdRosIface_ptr->setTBDes(TB_PROC_TIME);
    filterIface_ptr->setTBDes(TB_PROC_TIME);
    mvVislam_ptr->setTBDes(TB_PROC_TIME);
    
    // set static time offset
    // TODO(mereweth) - change this if we start timing out on correspondences
    // TODO(mereweth) - check to make sure we are not using ROS sim time
    {
        Fw::InputTimePairPort* port = timeConvert_ptr->get_ClockTimes_InputPort(0);
        Fw::Time t1(TB_ROS_TIME, 0, 0, 0);
        Fw::Time t2(TB_WORKSTATION_TIME, 0, 0, 0);
        port->invoke(t1, t2);
    }

    char logFileName[256];
    snprintf(logFileName, sizeof(logFileName), "/eng/STC_%u.txt", boot_count % 10);
    serialTextConv_ptr->set_log_file(logFileName, 100*1024, 0);

    snprintf(logFileName, sizeof(logFileName), "/eng/TextLog_%u.txt", boot_count % 10);
    textLogger_ptr->set_log_file(logFileName,100*1024, 0);

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wunused-local-typedefs"
    COMPILE_TIME_ASSERT(200 <= SnapdragonFlight::MVCAM_IMG_LP_BUFFER_POOL_SIZE,
                        NCAM_IMG_LP_BUFFER_ENOUGH);
    COMPILE_TIME_ASSERT(10 <= SnapdragonFlight::HIRESCAM_MAX_NUM_BUFFERS,
                        RCAM_BUFFER_ENOUGH);
    COMPILE_TIME_ASSERT(200 <= SnapdragonFlight::SCAM_IMG_LP_BUFFER_POOL_SIZE,
                        SCAM_IMG_LP_BUFFER_ENOUGH);
    #pragma GCC diagnostic pop

    mvCam_ptr->allocateBuffers(0, buffMallocator,
                               SnapdragonFlight::MVCAM_IMG_HP_BUFFER_POOL_SIZE,
                               200);
    hiresCam_ptr->allocateBuffers(0, hiresMallocator, 10);
#ifdef SOC_8096
    stereoCam_ptr->allocateBuffers(0, hiresMallocator,
                                   SnapdragonFlight::SCAM_IMG_HP_BUFFER_POOL_SIZE,
                                   200);
#endif

    // buffAccum doesn't own the buffers, so it's not the limiting factor
    buffAccumMVCamUnproc_ptr->allocateQueue(0,buffMallocator,
                                            SnapdragonFlight::MVCAM_IMG_MAX_NUM_BUFFERS);
    buffAccumHiresCamUnproc_ptr->allocateQueue(0,buffMallocator,
                                               SnapdragonFlight::HIRESCAM_MAX_NUM_BUFFERS);
    buffAccumStereoCamUnproc_ptr->allocateQueue(0,buffMallocator,
                                                SnapdragonFlight::SCAM_IMG_MAX_NUM_BUFFERS);

    static const NATIVE_UINT_TYPE maxLogSize = 25U * 1000U * 1000U;
    buffLogMVCamUnproc_ptr->initLog("/img/mvcam_", ".upbin", maxLogSize, sizeof(U32),
                                    0, // 0 means unlimited number of buffers per file
                                    Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC,
#ifdef SOC_8096
                                    4096);
#else
                                    512);
#endif
    buffLogHiresCamUnproc_ptr->initLog("/img/hirescam_",".upbin", maxLogSize, sizeof(U32),
                                       0, // 0 means unlimited number of buffers
                                       Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC,
#ifdef SOC_8096
                                    4096);
#else
                                    512);
#endif
    buffLogStereoCamUnproc_ptr->initLog("/img/stereocam_",".upbin", maxLogSize, sizeof(U32),
                                        0, // 0 means unlimited number of buffers
                                        Svc::BL_DIRECT_WRITE, Svc::BL_CLOSE_SYNC,
#ifdef SOC_8096
                                    4096);
#else
                                    512);
#endif
    buffLogMVCamUnproc_ptr->setBaseName("");
    buffLogHiresCamUnproc_ptr->setBaseName("");
    buffLogStereoCamUnproc_ptr->setBaseName("");

    isHiresChild = false;
    isStereoChild = false;
// fork works just fine on Darwin; just haven't got POSIX mq for IPC ports
#if defined TGT_OS_TYPE_LINUX
    int childPID = hiresCam_ptr->spawnChild();
    if (childPID == 0) { // we are in the child process
        isHiresChild = true;
#endif

        hiresCam_ptr->start(0,40,5*1000*1024, CORE_PRCP);

#if defined TGT_OS_TYPE_LINUX
        return; // don't start any other threads in the child
    }
#endif

#ifdef SOC_8096
    childPID = stereoCam_ptr->spawnChild();
    if (childPID == 0) { // we are in the child process
        isStereoChild = true;
#endif

        stereoCam_ptr->start(0,50,5*1000*1024, CORE_PRCP);

#ifdef SOC_8096
        return; // don't start any other threads in the child
    }
#endif

    // NOTE(mereweth) - putting this near the top in case we want to fork in ipcRelay instead
    ipcRelay_ptr->start(0,30,20*1024, CORE_DEV);

    // Active component startup
    // start rate groups
    rgTlm_ptr->start(0, 50, 20*1024);
    rgXfer_ptr->start(0, 95, 20*1024);
    // start dispatcher
    cmdDisp_ptr->start(0,60,20*1024);
    // start sequencer
    cmdSeq_ptr->start(0,50,20*1024);
    cmdSeq2_ptr->start(0,50,20*1024);
    // start telemetry
    eventLogger_ptr->start(0,50,20*1024);
    eventLoggerLL_ptr->start(0,50,20*1024);
    chanTlm_ptr->start(0,60,20*1024);
    prmDb_ptr->start(0,50,20*1024);
    textLogger_ptr->start(0,30,20*1024);

    dspTimeSync_ptr->start(0,30,20*1024);
    llTimeSync_ptr->start(0,30,20*1024);

    snapHealth_ptr->start(0,40,20*1024);

    mvCam_ptr->start(0, 80, 5*1000*1024, CORE_DEV);
    mvVislam_ptr->start(0, 80, 5*1000*1024, CORE_GNC);
    mvDFS_ptr->start(0, 79, 5*1000*1024, CORE_PRCP);
    hexRouter_ptr->start(0, 90, 20*1024, CORE_DEV);

    imgTlm_ptr->start(0, 1, 20*1024);

    buffLogMVCamUnproc_ptr->start(0, 20, 20*1024);
    buffLogHiresCamUnproc_ptr->start(0, 20, 20*1024);
    buffLogStereoCamUnproc_ptr->start(0, 20, 20*1024);
    buffAccumMVCamUnproc_ptr->start(0, 20, 20*1024);
    buffAccumHiresCamUnproc_ptr->start(0, 20, 20*1024);
    buffAccumStereoCamUnproc_ptr->start(0, 20, 20*1024);

    llRouter_ptr->start(0, 85, 20*1024);
    serialTextConv_ptr->start(0,79,20*1024);

    fileLogger_ptr->start(0,50,20*1024);

    if (!llRouterDevices) {
        hexRouter_ptr->startPortReadThread(90,20*1024, CORE_DEV);
        //hexRouter_ptr->startBuffReadThread(60,20*1024, CORE_DEV);
    }

    if (llRouterDevices) {
#ifdef BUILD_SDFLIGHT
        gpioTimeSync_ptr->open(
#ifdef SOC_8096
                               25, // J7, pin 7, 3.3V level
#else
                               47, //TODO(Mereweth) - this is a placeholder; J12, pin 4, 3.3V level
#endif
                               Drv::LinuxGpioDriverComponentImpl::GPIO_OUT);

        // Must start serial drivers after tasks that setup the buffers for the driver:
        blspSerialDriverLL_ptr->open(
#ifdef SOC_8096
                                     7, // tty 3, J10, SONAR_UART, BLSP7
#else
                                     3, //TODO(Mereweth) - this is a placeholder; J12, BLSP8
#endif
                                     SnapdragonFlight::BlspSerialDriverComponentImpl::BAUD_921K,
                                     SnapdragonFlight::BlspSerialDriverComponentImpl::NO_FLOW,
                                     true);
    
        blspSerialDriverDebug_ptr->open(
#ifdef SOC_8096
                                        5, // tty 1, J12, ESC_UART, BLSP5
#else
                                        2, //TODO(Mereweth) - this is a placeholder; J13, BLSP6
#endif
                                        SnapdragonFlight::BlspSerialDriverComponentImpl::BAUD_921K,
                                        SnapdragonFlight::BlspSerialDriverComponentImpl::NO_FLOW,
                                        true);

        /* ---------- Done opening devices, now start device threads ---------- */
        blspSerialDriverLL_ptr->startReadThread(98, 20*1024);
        blspSerialDriverDebug_ptr->startReadThread(40, 20*1024);
#else //not BUILD_SDFLIGHT
        // NOTE - need a GPIO for timesync - will assert when gpio is used!!
        //gpioTimeSync_ptr->open(TODO, Drv::LinuxGpioDriverComponentImpl::GPIO_OUT);
        
        // Must start serial drivers after tasks that setup the buffers for the driver:
        serialDriverLL_ptr->open("/dev/ttyUSB0",
                                 Drv::LinuxSerialDriverComponentImpl::BAUD_921K,
                                 Drv::LinuxSerialDriverComponentImpl::NO_FLOW,
                                 Drv::LinuxSerialDriverComponentImpl::PARITY_NONE,
                                 true);

        serialDriverDebug_ptr->open("/dev/ttyUSB1",
                                    Drv::LinuxSerialDriverComponentImpl::BAUD_921K,
                                    Drv::LinuxSerialDriverComponentImpl::NO_FLOW,
                                    Drv::LinuxSerialDriverComponentImpl::PARITY_NONE,
                                    true);

        /* ---------- Done opening devices, now start device threads ---------- */
        serialDriverLL_ptr->startReadThread(98, 20*1024);
        serialDriverDebug_ptr->startReadThread(40, 20*1024);
#endif //BUILD_SDFLIGHT
    } //llRouterDevices

    atiNetbox_ptr->set_thread_attr(0, 30, 20*1024, true, CORE_GNC);
    atiNetbox_ptr->open("192.168.2.20", "192.168.2.10",
                        Drv::ATINetboxComponentImpl::ATINETBOX_RDP_PORT);

    // Initialize socket server
    if (port_number && hostname) {
        if (startSocketNow) {
            sockGndIf_ptr->startSocketTask(40, 20*1024, port_number, hostname, Svc::SocketGndIfImpl::SEND_TCP);
        } else {
            sockGndIf_ptr->setSocketTaskProperties(40, 20*1024, port_number, hostname, Svc::SocketGndIfImpl::SEND_TCP);
        }
    }
    if (ll_port_number && hostname) {
        if (startSocketNow) {
            sockGndIfLL_ptr->startSocketTask(40, 20*1024, ll_port_number, hostname, Svc::SocketGndIfImpl::SEND_TCP);
        } else {
            sockGndIfLL_ptr->setSocketTaskProperties(40, 20*1024, ll_port_number, hostname, Svc::SocketGndIfImpl::SEND_TCP);
        }
    }

    if (udp_recv_string) {
        udpReceiver_ptr->open(udp_recv_string);
        udpReceiver_ptr->startThread(85,20*1024);
    }

#if FW_OBJECT_REGISTRATION == 1
    //simpleReg.dump();
#endif

}


void run1cycle(void) {
    // call interrupt to emulate a clock
    Svc::InputCyclePort* port = rgDrv_ptr->get_CycleIn_InputPort(0);
    Svc::TimerVal cycleStart;
    cycleStart.take();
    port->invoke(cycleStart);
    Os::Task::delay(33);
}

void runcycles(NATIVE_INT_TYPE cycles) {
    if (cycles == -1) {
        while (true) {
            run1cycle();
        }
    }

    for (NATIVE_INT_TYPE cycle = 0; cycle < cycles; cycle++) {
        run1cycle();
    }
}

void exitTasks(bool isHiresChild, bool isStereoChild,
               bool llRouterDevices) {
#if defined TGT_OS_TYPE_LINUX
    if (isHiresChild) {
#endif
        hiresCam_ptr->exit();
        hiresCam_ptr->deallocateBuffers(hiresMallocator);
        hiresCam_ptr->join(NULL);
        DEBUG_PRINT("After hires thread quit\n");
#if defined TGT_OS_TYPE_LINUX
        return;
    }
#endif

#ifdef SOC_8096
    if (isStereoChild) {
#endif
        stereoCam_ptr->exit();
#ifdef SOC_8096
        stereoCam_ptr->deallocateBuffers(hiresMallocator);
#endif
        stereoCam_ptr->join(NULL);
        DEBUG_PRINT("After stereo thread quit\n");
#ifdef SOC_8096
        return;
    }
#endif

    ipcRelay_ptr->exit();
    DEBUG_PRINT("After IPCRelay thread quit\n");

    mvCam_ptr->exit();
    mvCam_ptr->deallocateBuffers(buffMallocator);
    mvCam_ptr->join(NULL);
    DEBUG_PRINT("After mvcam thread quit\n");
    if (!llRouterDevices) {
        hexRouter_ptr->quitReadThreads();
        DEBUG_PRINT("After HexRouter read thread quit\n");
    }

    buffAccumMVCamUnproc_ptr->deallocateQueue(buffMallocator);
    buffAccumHiresCamUnproc_ptr->deallocateQueue(buffMallocator);
    buffAccumStereoCamUnproc_ptr->deallocateQueue(buffMallocator);

    if (llRouterDevices) {
#ifdef BUILD_SDFLIGHT
        blspSerialDriverLL_ptr->quitReadThread();
        blspSerialDriverDebug_ptr->quitReadThread();
#else //BUILD_SDFLIGHT
        serialDriverLL_ptr->quitReadThread();
        serialDriverDebug_ptr->quitReadThread();
#endif //BUILD_SDFLIGHT
    }
    atiNetbox_ptr->stop();

    imgTlm_ptr->exit();

    buffLogMVCamUnproc_ptr->exit();
    buffLogHiresCamUnproc_ptr->exit();
    buffAccumMVCamUnproc_ptr->exit();
    buffAccumHiresCamUnproc_ptr->exit();

    mvVislam_ptr->exit();
    mvDFS_ptr->exit();
    llRouter_ptr->exit();
    serialTextConv_ptr->exit();

    snapHealth_ptr->exit();
    rgTlm_ptr->exit();
    rgXfer_ptr->exit();
    cmdDisp_ptr->exit();
    eventLogger_ptr->exit();
    eventLoggerLL_ptr->exit();
    chanTlm_ptr->exit();
    prmDb_ptr->exit();
    textLogger_ptr->exit();
    fileLogger_ptr->exit();
    cmdSeq_ptr->exit();
    cmdSeq2_ptr->exit();
    hexRouter_ptr->exit();
    DEBUG_PRINT("After HexRouter quit\n");
}

extern "C" {
    // Defined in version.c
    extern const unsigned int FSW_HASH;
    extern const char* FSW_BRANCH;
}

void print_usage() {
    (void) printf("Usage: ./SDREF hash 0x%x branch %s\n"
                  "[options]\n"
                  "-p\tport_number\n"
                  "-d\trun on Snapdragon DSP\n"
                  "-r\tnot present: connect all to GNC;\n"
                  "\t\t0: connect all to ROS; 1: connect point cloud to ROS\n"
                  "-x\tll_port_number\n"
                  "-u\tUDP recv port string\n"
                  "-t\tUDP transmit port string\n"
                  "-z\tZMQ image send string\n"
                  "-a\thostname/IP address\n"
                  "-l\tFor time-based cycles\n"
                  "-i\tto disable init\n"
                  "-f\tto disable fini\n"
                  "-o\tto run # cycles instead of continuously\n"
                  "-b\tBoot count\n"
                  "-s\tStart socket immediately\n",
                  FSW_HASH, FSW_BRANCH);
}


#include <signal.h>
#include <stdio.h>

extern "C" {
    int main(int argc, char* argv[]);
};

volatile sig_atomic_t terminate = 0;
volatile sig_atomic_t hexref_finid = 0;

static void sighandler(int signum) {
    terminate = 1;
    if (SIGSEGV == signum) {
        printf("segv; calling hexref_fini\n");
        if (!hexref_finid) {
            hexref_fini();
            hexref_finid = 1;
            kill(getpid(), signum);
        }
    }
}

void dummy() {
    while(!terminate) {
        Os::Task::delay(1000);
    }
}

int main(int argc, char* argv[]) {
    bool noInit = false;
    bool noFini = false;
    bool kraitCycle = false;
    bool hexCycle = true;
    int numKraitCycles = 0;
    U32 port_number = 0;
    U32 ll_port_number = 0;
    I32 option = 0;
    char *hostname = NULL;
    char* udp_recv_string = 0;
    char* udp_send_string = 0;
    char* zmq_image_string = 0;
    bool local_cycle = true;
    bool isHiresChild = false;
    bool isStereoChild = false;
    bool startSocketNow = false;
    U32 boot_count = 0;
    bool llRouterDevices = true;
    I32 rosConnect = 0;
    bool gncCamConnect = true;
    bool gncCloudConnect = true;

    // Removes ROS cmdline args as a side-effect
    ros::init(argc,argv,"SDREF", ros::init_options::NoSigintHandler);

    while ((option = getopt(argc, argv, "r:difhlsp:x:a:u:t:o:b:z:")) != -1){
        switch(option) {
            case 'h':
                print_usage();
                return 0;
                break;
            case 'r':
                gncCamConnect = false;
                gncCloudConnect = false;
                
                rosConnect = atoi(optarg);
                if (rosConnect >= 1) {
                    gncCamConnect = true;
                }
                break;
            case 'd':
                llRouterDevices = false;
                break;
            case 'l':
                local_cycle = true;
                break;
            case 'b':
                boot_count = atoi(optarg);
                break;
            case 'p':
                port_number = atoi(optarg);
                break;
            case 'x':
                ll_port_number = atoi(optarg);
                break;
            case 'z':
                zmq_image_string = optarg;
                break;
            case 'a':
                hostname = optarg;
                break;
            case 'u':
                udp_recv_string = optarg;
                break;
            case 't':
                udp_send_string = optarg;
                break;
            case 'i':
                noInit = true;
                break;
            case 'f':
                noFini = true;
                break;
            case 'o':
                numKraitCycles = atoi(optarg);
                kraitCycle = true;
                hexCycle = false;
                break;
            case 's':
                startSocketNow = true;
                break;
            case '?':
                return 1;
            default:
                print_usage();
                return 1;
        }
    }

    if (kraitCycle && hexCycle) {
        printf("o and c both specified - use one only\n");
        return 1;
    }

    signal(SIGINT,sighandler);
    signal(SIGSEGV,sighandler);
    signal(SIGTERM,sighandler);
    signal(SIGKILL,sighandler);
    signal(SIGPIPE, SIG_IGN);

    (void) printf("Hit Ctrl-C to quit\n");

    if (!llRouterDevices) {
#ifdef BUILD_SDFLIGHT
        if (!noInit) {
            hexref_init();
        }
#endif
    }
    
    constructApp(port_number, ll_port_number,
                 udp_recv_string, udp_send_string, zmq_image_string,
                 hostname, boot_count,
                 isHiresChild, isStereoChild,
                 llRouterDevices,
                 gncCamConnect, gncCloudConnect,
                 startSocketNow);
    //dumparch();

    Os::Task task;
    Os::Task waiter;

    if (!isHiresChild && !isStereoChild) {
        sdRosIface_ptr->startIntTask(30, 5*1000*1024);
        mrCtrlIface_ptr->startIntTask(30, 5*1000*1024);
        filterIface_ptr->startIntTask(30, 5*1000*1024);
        rosSeq_ptr->startIntTask(30, 5*1000*1024);
        ewok_ptr->startIntTask(30, 5*1000*1024);

        ros::console::shutdown();

        Os::TaskString waiter_task_name("WAITER");
        if (!llRouterDevices) {
#ifdef BUILD_SDFLIGHT
            // TODO(mereweth) - test that calling other functions before init has no effect
            //hexref_rpc_relay_buff_allocate(10);
            if (hexCycle) {
                Os::TaskString task_name("HEXRPC");
                DEBUG_PRINT("Starting cycler on hexagon\n");
                if (Os::Task::TASK_OK !=
                    task.start(task_name, 0, 10, 20*1024,
                               (Os::Task::taskRoutine) hexref_run, NULL)) {
                    DEBUG_PRINT("Error starting up DSP RUN task\n");
                    terminate = 1;
                }
            }
            if (Os::Task::TASK_OK !=
                waiter.start(waiter_task_name, 0, 10, 20*1024,
                             (Os::Task::taskRoutine) hexref_wait, NULL)) {
                DEBUG_PRINT("Error starting up DSP WAIT task\n");
                terminate = 1;
            }
#else
            if (hexCycle) {
                Os::TaskString task_name("DUMMY");
                FW_ASSERT(Os::Task::TASK_OK ==
                          task.start(task_name, 0, 10, 20*1024,
                                     (Os::Task::taskRoutine) dummy, NULL));
            }
            FW_ASSERT(Os::Task::TASK_OK ==
                      waiter.start(waiter_task_name, 0, 10, 20*1024,
                                   (Os::Task::taskRoutine) dummy, NULL));
#endif //BUILD_SDFLIGHT
        } //not LLROUTER_DEVICES

        if (!llRouterDevices) {
#ifdef BUILD_SDFLIGHT
            if (kraitCycle) {
                DEBUG_PRINT("Cycling from Krait\n");
                hexref_cycle(numKraitCycles);
            }
#endif //BUILD_SDFLIGHT
        }

    } //!isHiresChild && !isStereoChild

    int cycle = 0;

    while (!terminate) {
        if (isHiresChild) {
            //DEBUG_PRINT("Hires Child Cycle %d\n",cycle);
        }
        else if (isStereoChild) {
            //DEBUG_PRINT("Stereo Child Cycle %d\n",cycle);
        }
        else {
            //DEBUG_PRINT("Parent Cycle %d\n",cycle);
        }
        if (local_cycle && !isHiresChild && !isStereoChild) {
            runcycles(1);
        } else {
            Os::Task::delay(1000);
        }
        cycle++;
    }

    if (!isHiresChild && !isStereoChild) {
        if (!llRouterDevices) {
#ifdef BUILD_SDFLIGHT
            if (!noFini) {
                DEBUG_PRINT("Calling exit function for SDFLIGHT\n");
                hexref_fini();
            }
#endif //BUILD_SDFLIGHT
        }

        // stop tasks
        DEBUG_PRINT("Stopping tasks\n");
        sdRosIface_ptr->disableRos();
        mrCtrlIface_ptr->disableRos();
        filterIface_ptr->disableRos();
        rosSeq_ptr->disableRos();
        ros::shutdown();
    } // !isHiresChild && !isStereoChild

    exitTasks(isHiresChild, isStereoChild, llRouterDevices);
    
    if (!llRouterDevices && !isHiresChild && !isStereoChild) {
        if (hexCycle) {
            DEBUG_PRINT("Waiting for the runner to return\n");
            FW_ASSERT(task.join(NULL) == Os::Task::TASK_OK);
        }

        DEBUG_PRINT("Waiting for the Hexagon code to be unloaded - prevents hanging the board\n");
        FW_ASSERT(waiter.join(NULL) == Os::Task::TASK_OK);
    } // !isHiresChild && !isStereoChild

    // Give time for threads to exit
#if defined TGT_OS_TYPE_LINUX
    if (!isHiresChild && !isStereoChild) {
        (void) printf("Waiting for child...\n");
        wait(NULL);
#endif
        (void) printf("Waiting for threads...\n");
        Os::Task::delay(1000);

        (void) printf("Exiting...\n");
#if defined TGT_OS_TYPE_LINUX
    }
#endif
    return 0;
}
