#include <HEXREF/Rpc/hexref.h>
#include <HAP_farf.h>
#include <unistd.h>

#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImpl.hpp>

SnapdragonFlight::KraitRouterComponentImpl* kraitRouter_ptr = 0;

#include <Svc/ActiveRateGroup/ActiveRateGroupImpl.hpp>
#include <Svc/PassiveRateGroup/PassiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>
#include <Svc/RateGroupDecoupler/RateGroupDecouplerComponentImpl.hpp>

#include <Svc/PassiveConsoleTextLogger/ConsoleTextLoggerImpl.hpp>
#include <Svc/LinuxTime/LinuxTimeImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>

#include <HEXREF/Top/TargetInit.hpp>
#include <Svc/AssertFatalAdapter/AssertFatalAdapterComponentImpl.hpp>
#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>
#include <Svc/ActiveDecoupler/ActiveDecouplerComponentImpl.hpp>

#include <Drv/IMU/MPU9250/MPU9250ComponentImpl.hpp>
#include <Drv/LinuxSpiDriver/LinuxSpiDriverComponentImpl.hpp>
#include <Drv/LinuxI2CDriver/LinuxI2CDriverComponentImpl.hpp>
#include <Drv/LinuxGpioDriver/LinuxGpioDriverComponentImpl.hpp>
#include <Drv/LinuxPwmDriver/LinuxPwmDriverComponentImpl.hpp>
#include <Gnc/Utils/FrameTransform/FrameTransformComponentImpl.hpp>
#include <Gnc/Ctrl/Se3Ctrl/Se3CtrlComponentImpl.hpp>
#include <Gnc/Ctrl/WrenchMixer/WrenchMixerComponentImpl.hpp>
#include <Gnc/Ctrl/ActuatorAdapter/ActuatorAdapterComponentImpl.hpp>
#include <Gnc/Sysid/SigGen/SigGenComponentImpl.hpp>
#include <Gnc/Est/AttFilter/AttFilterComponentImpl.hpp>

#include <LLProc/ShortLogQueue/ShortLogQueueComponentImpl.hpp>
#include <LLProc/LLCmdDispatcher/LLCmdDispatcherComponentImpl.hpp>
#include <LLProc/LLTlmChan/LLTlmChanImpl.hpp>

Svc::RateGroupDecouplerComponentImpl* rgDecouple_ptr = 0;
Svc::ActiveDecouplerComponentImpl* actDecouple_ptr = 0;
Svc::RateGroupDriverImpl* rgGncDrv_ptr = 0;
Svc::PassiveRateGroupImpl* rgOp_ptr = 0;
Svc::PassiveRateGroupImpl* rgTlm_ptr = 0;
Svc::ConsoleTextLoggerImpl* textLogger_ptr = 0;
LLProc::ShortLogQueueComponentImpl* logQueue_ptr = 0;
Svc::LinuxTimeImpl* linuxTime_ptr = 0;
Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr = 0;
Svc::FatalHandlerComponentImpl* fatalHandler_ptr = 0;
LLProc::LLCmdDispatcherImpl* cmdDisp_ptr = 0;
LLProc::LLTlmChanImpl* tlmChan_ptr = 0;
Gnc::FrameTransformComponentImpl* ctrlXest_ptr = 0;
Gnc::Se3CtrlComponentImpl* se3Ctrl_ptr = 0;
Gnc::WrenchMixerComponentImpl* mixer_ptr = 0;
Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr = 0;
Gnc::SigGenComponentImpl* sigGen_ptr = 0;
Gnc::AttFilterComponentImpl* attFilter_ptr = 0;
Drv::MPU9250ComponentImpl* mpu9250_ptr = 0;
Drv::LinuxSpiDriverComponentImpl* spiDrv_ptr = 0;
Drv::LinuxI2CDriverComponentImpl* i2cDrv_ptr = 0;
Drv::LinuxGpioDriverComponentImpl* imuDRInt_ptr = 0;
Drv::LinuxGpioDriverComponentImpl* hwEnablePin_ptr = 0;
Drv::LinuxPwmDriverComponentImpl* escPwm_ptr = 0;

void allocComps() {
    rgDecouple_ptr = new Svc::RateGroupDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDECOUPLE",
#endif
                        5) // multiply by sleep duration in run1backupCycle
;

    actDecouple_ptr = new Svc::ActiveDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
                        "ACTDECOUPLE"
#endif
                                                            )
;

    NATIVE_UINT_TYPE rgTlmContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        Drv::MPU9250_SCHED_CONTEXT_TLM, // mpu9250
        Gnc::ATTFILTER_SCHED_CONTEXT_TLM, // attFilter
        Gnc::SIGGEN_SCHED_CONTEXT_TLM, // sigGen
        Gnc::SE3CTRL_SCHED_CONTEXT_TLM, // se3Ctrl
        0, // mixer
        Gnc::ACTADAP_SCHED_CONTEXT_TLM, // adapter
        0, // logQueue
        0, // chanTlm
    };

    rgTlm_ptr = new Svc::PassiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                            "RGTLM",
#endif
                            rgTlmContext,FW_NUM_ARRAY_ELEMENTS(rgTlmContext));
;

    NATIVE_INT_TYPE rgGncDivs[] = {10, 1000};

    rgGncDrv_ptr = new Svc::RateGroupDriverImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDRV",
#endif
                        rgGncDivs,FW_NUM_ARRAY_ELEMENTS(rgGncDivs));

    NATIVE_UINT_TYPE rgOpContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        Drv::MPU9250_SCHED_CONTEXT_OPERATE,
        Gnc::ATTFILTER_SCHED_CONTEXT_FILT, // attFilter
        Gnc::SIGGEN_SCHED_CONTEXT_OP, // sigGen
        Gnc::SE3CTRL_SCHED_CONTEXT_CTRL, // se3Ctrl
        0, // mixer
	Gnc::ACTADAP_SCHED_CONTEXT_ARM, // adapter - for arming
        0, // logQueue
        0, // kraitRouter
    };

    rgOp_ptr = new Svc::PassiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                            "RGOP",
#endif
                            rgOpContext,FW_NUM_ARRAY_ELEMENTS(rgOpContext));
;

    textLogger_ptr = new Svc::ConsoleTextLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("TLOG")
#endif
;

    logQueue_ptr = new LLProc::ShortLogQueueComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SLOG")
#endif
;

    linuxTime_ptr = new Svc::LinuxTimeImpl
#if FW_OBJECT_NAMES == 1
                        ("LTIME")
#endif
;

    kraitRouter_ptr = new SnapdragonFlight::KraitRouterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("KRAITRTR")
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

    cmdDisp_ptr = new LLProc::LLCmdDispatcherImpl
#if FW_OBJECT_NAMES == 1
                        ("CMDDISP")
#endif
;

    tlmChan_ptr = new LLProc::LLTlmChanImpl
#if FW_OBJECT_NAMES == 1
                        ("TLMCHAN")
#endif
;

    ctrlXest_ptr = new Gnc::FrameTransformComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("CTRLXEST")
#endif
;
 
    se3Ctrl_ptr = new Gnc::Se3CtrlComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SE3CTRL")
#endif
;

    mixer_ptr = new Gnc::WrenchMixerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("MIXER")
#endif
;

    actuatorAdapter_ptr = new Gnc::ActuatorAdapterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("ACTADAP")
#endif
;

    sigGen_ptr = new Gnc::SigGenComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SIGGEN")
#endif
;

    attFilter_ptr = new Gnc::AttFilterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("ATTFILTER")
#endif
;

    mpu9250_ptr = new Drv::MPU9250ComponentImpl(
#if FW_OBJECT_NAMES == 1
                        "MPU9250",
#endif
                         false) // don't use magnetometer for now
;

    spiDrv_ptr = new Drv::LinuxSpiDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("SPIDRV")
#endif
;

    i2cDrv_ptr = new Drv::LinuxI2CDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                    ("I2CDRV")
#endif
;

    imuDRInt_ptr = new Drv::LinuxGpioDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("IMUDRINT")
#endif
;

    hwEnablePin_ptr = new Drv::LinuxGpioDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("HWENPIN")
#endif
;

    escPwm_ptr = new Drv::LinuxPwmDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("ESCPWM")
#endif
;
}

volatile bool terminate = false;
volatile bool preinit = true;

int hexref_arm() {
    FARF(ALWAYS, "hexref_arm");
    if (preinit) {
        FARF(ALWAYS, "hexref_arm preinit - returning");
        return -1;
    }
    return 0;
}

int hexref_init() {
    FARF(ALWAYS, "hexref_init");
    allocComps();

    kraitRouter_ptr->init(50, 512);
    
    preinit = false;
    usleep(1000 * 1000);
    return 0;
}

void stress_router() {
    Fw::ExternalSerializeBuffer bufObj;
    char buf[200] = {"hi"};
    bufObj.setExtBuffer((U8*) buf, 200);
    bufObj.setBuffLen(12);
    Fw::InputSerializePort* serPort = kraitRouter_ptr->get_HexPortsIn_InputPort(0);
    serPort->invokeSerial(bufObj);

    Svc::InputSchedPort* port = kraitRouter_ptr->get_Sched_InputPort(0);
    port->invoke(0);
}

int hexref_run() {
    FARF(ALWAYS, "hexref_run");
    if (preinit) {
        FARF(ALWAYS, "hexref_run preinit - returning");
        return -1;
    }

    while (!terminate) {
        //FARF(ALWAYS, "hexref_run loop; terminate: %d", terminate);
        stress_router();
        usleep(1000);
    }
    return 0;
}

int hexref_cycle(unsigned int cycles) {
    FARF(ALWAYS, "hexref_cycle %d", cycles);
    if (preinit) {
        FARF(ALWAYS, "hexref_cycle preinit - returning");
        return -1;
    }

    for (unsigned int i = 0; i < cycles; i++) {
        FARF(ALWAYS, "hexref_cycle loop %d; terminate: %d", i, terminate);
        stress_router();
        usleep(1000);
    }
    return 0;
}

int hexref_wait() {
    FARF(ALWAYS, "hexref_wait");
    while (!terminate) {
        FARF(ALWAYS, "hexref_wait loop; terminate: %d", terminate);
        usleep(1000 * 1000);
    }
    return 0;
}

int hexref_fini() {
    FARF(ALWAYS, "hexref_fini");
    terminate = true;
    return 0;
}

int hexref_rpc_relay_buff_read(unsigned int* port, unsigned char* buff, int buffLen, int* bytes) {
    return kraitRouter_ptr->buffRead(port, buff, buffLen, bytes);
}

int hexref_rpc_relay_port_read(unsigned char* buff, int buffLen, int* bytes) {
    return kraitRouter_ptr->portRead(buff, buffLen, bytes);
}

int hexref_rpc_relay_buff_write(unsigned int port, const unsigned char* buff, int buffLen) {
    return kraitRouter_ptr->buffWrite(port, buff, buffLen);
}

int hexref_rpc_relay_port_write(const unsigned char* buff, int buffLen) {
    return kraitRouter_ptr->portWrite(buff, buffLen);
}
