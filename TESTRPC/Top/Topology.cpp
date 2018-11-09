#include <HEXREF/Rpc/hexref.h>
#include <HAP_farf.h>
#include <unistd.h>

#include <Svc/ActiveRateGroup/ActiveRateGroupImpl.hpp>
#include <Svc/PassiveRateGroup/PassiveRateGroupImpl.hpp>
#include <Svc/RateGroupDriver/RateGroupDriverImpl.hpp>
#include <Svc/RateGroupDecoupler/RateGroupDecouplerComponentImpl.hpp>

#include <Svc/PassiveConsoleTextLogger/ConsoleTextLoggerImpl.hpp>
#include <Svc/ActiveLogger/ActiveLoggerImpl.hpp>
#include <Svc/LinuxTime/LinuxTimeImpl.hpp>
#include <Fw/Obj/SimpleObjRegistry.hpp>

#include <HEXREF/Top/TargetInit.hpp>
#include <Svc/AssertFatalAdapter/AssertFatalAdapterComponentImpl.hpp>
#include <Svc/FatalHandler/FatalHandlerComponentImpl.hpp>

#include <SnapdragonFlight/KraitRouter/KraitRouterComponentImpl.hpp>

#include <Drv/IMU/MPU9250/MPU9250ComponentImpl.hpp>
#include <Drv/LinuxSpiDriver/LinuxSpiDriverComponentImpl.hpp>
#include <Drv/LinuxI2CDriver/LinuxI2CDriverComponentImpl.hpp>
#include <Drv/LinuxGpioDriver/LinuxGpioDriverComponentImpl.hpp>
#include <Drv/LinuxPwmDriver/LinuxPwmDriverComponentImpl.hpp>
#include <Gnc/Ctrl/LeeCtrl/LeeCtrlComponentImpl.hpp>
#include <Gnc/Ctrl/BasicMixer/BasicMixerComponentImpl.hpp>
#include <Gnc/Ctrl/ActuatorAdapter/ActuatorAdapterComponentImpl.hpp>
#include <Gnc/Est/ImuInteg/ImuIntegComponentImpl.hpp>

#include <LLProc/ShortLogQueue/ShortLogQueueComponentImpl.hpp>

Svc::RateGroupDecouplerComponentImpl* rgDecouple_ptr = 0;
Svc::ActiveRateGroupImpl* rg_ptr = 0;
Svc::RateGroupDriverImpl* rgGncDrv_ptr = 0;
Svc::PassiveRateGroupImpl* rgAtt_ptr = 0;
Svc::PassiveRateGroupImpl* rgPos_ptr = 0;
Svc::ConsoleTextLoggerImpl* textLogger_ptr = 0;
Svc::ActiveLoggerImpl* eventLogger_ptr = 0;
LLProc::ShortLogQueueComponentImpl* logQueue_ptr = 0;
Svc::LinuxTimeImpl* linuxTime_ptr = 0;
SnapdragonFlight::KraitRouterComponentImpl* kraitRouter_ptr = 0;
Svc::AssertFatalAdapterComponentImpl* fatalAdapter_ptr = 0;
Svc::FatalHandlerComponentImpl* fatalHandler_ptr = 0;
Gnc::LeeCtrlComponentImpl* leeCtrl_ptr = 0;
Gnc::BasicMixerComponentImpl* mixer_ptr = 0;
Gnc::ActuatorAdapterComponentImpl* actuatorAdapter_ptr = 0;
Gnc::ImuIntegComponentImpl* imuInteg_ptr = 0;
Drv::MPU9250ComponentImpl* mpu9250_ptr = 0;
Drv::LinuxSpiDriverComponentImpl* spiDrv_ptr = 0;
Drv::LinuxI2CDriverComponentImpl* i2cDrv_ptr = 0;
Drv::LinuxGpioDriverComponentImpl* imuDRInt_ptr = 0;
Drv::LinuxPwmDriverComponentImpl* escPwm_ptr = 0;

void allocComps() {
     rgDecouple_ptr = new Svc::RateGroupDecouplerComponentImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDECOUPLE",
#endif
                        5) // 50 dropped hardware cycles before an error
;

    NATIVE_UINT_TYPE rgContext[Svc::ActiveRateGroupImpl::CONTEXT_SIZE] = {
        0, // unused
        Gnc::IMUINTEG_SCHED_CONTEXT_TLM, // imuInteg
        Gnc::LCTRL_SCHED_CONTEXT_TLM, // leeCtrl
    };
    
    rg_ptr = new Svc::ActiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                        "RG",
#endif
                        rgContext,FW_NUM_ARRAY_ELEMENTS(rgContext));
;

    NATIVE_INT_TYPE rgGncDivs[] = {10, 1, 1000};
    
    rgGncDrv_ptr = new Svc::RateGroupDriverImpl(
#if FW_OBJECT_NAMES == 1
                        "RGDRV",
#endif
                        rgGncDivs,FW_NUM_ARRAY_ELEMENTS(rgGncDivs));

    NATIVE_UINT_TYPE rgAttContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        Drv::MPU9250_SCHED_CONTEXT_OPERATE,
        Gnc::IMUINTEG_SCHED_CONTEXT_ATT, // imuInteg
        Gnc::LCTRL_SCHED_CONTEXT_ATT, // leeCtrl
    };

    rgAtt_ptr = new Svc::PassiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                            "RGATT",
#endif
                            rgAttContext,FW_NUM_ARRAY_ELEMENTS(rgAttContext));
;

    NATIVE_UINT_TYPE rgPosContext[Svc::PassiveRateGroupImpl::CONTEXT_SIZE] = {
        0, //TODO(mereweth) - IMU?
        Gnc::IMUINTEG_SCHED_CONTEXT_POS, // imuInteg
        Gnc::LCTRL_SCHED_CONTEXT_POS, // leeCtrl
    };
    
    rgPos_ptr = new Svc::PassiveRateGroupImpl(
#if FW_OBJECT_NAMES == 1
                    "RGPOS",
#endif
                    rgPosContext,FW_NUM_ARRAY_ELEMENTS(rgPosContext));
;

    textLogger_ptr = new Svc::ConsoleTextLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("TLOG")
#endif
;

    eventLogger_ptr = new Svc::ActiveLoggerImpl
#if FW_OBJECT_NAMES == 1
                        ("ELOG")
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

    leeCtrl_ptr = new Gnc::LeeCtrlComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("LEECTRL")
#endif
;

    mixer_ptr = new Gnc::BasicMixerComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("MIXER")
#endif
;

    actuatorAdapter_ptr = new Gnc::ActuatorAdapterComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("ACTADAP")
#endif
;

    imuInteg_ptr = new Gnc::ImuIntegComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("IMUINTEG")
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

    escPwm_ptr = new Drv::LinuxPwmDriverComponentImpl
#if FW_OBJECT_NAMES == 1
                        ("ESCPWM")
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

volatile bool terminate = false;
volatile bool preinit = true;

int hexref_arm() {
    FARF(ALWAYS, "hexref_arm");
    if (preinit) {
        DEBUG_PRINT("hexref_arm preinit - returning");
        return -1;
    }
    Drv::InputI2CConfigPort* confPort = i2cDrv_ptr->get_I2CConfig_InputPort(0);
    Drv::InputI2CReadWritePort* rwPort = i2cDrv_ptr->get_I2CReadWrite_InputPort(0);
    for (U32 i = 0; i < 35; i++) {
        FARF(ALWAYS, "arm %u", i);
        for (U32 j = 11; j <= 14; j++) {
            confPort->invoke(400, j, 100);
            U8 readBuf[1] = { 0 };
            U8 writeBuf[1] = { 0 };
            Fw::Buffer writeObj = Fw::Buffer(0, 0, (U64) writeBuf, 1);
            Fw::Buffer readObj = Fw::Buffer(0, 0, (U64) readBuf, 1);
            rwPort->invoke(writeObj,
                           readObj);
            usleep(2500);
        }
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
