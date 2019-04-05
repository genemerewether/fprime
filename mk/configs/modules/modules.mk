# List of flight software modules to build

# NOTE: Must not be characters after continuation character "\" or will get "commands commence before first target" errors

FW_MODULES := \
	Fw/FilePacket \
	Fw/Cfg \
	Fw/Buffer \
	Fw/Comp \
	Fw/Obj \
	Fw/Port \
	Fw/Cmd \
	Fw/Tlm \
	Fw/Prm \
	Fw/Log \
	Fw/Time \
	Fw/Com \
	Fw/ComFile \
	Fw/SerializableFile \
	Fw/Types

FW_GTEST_MODULES := \
	Fw/SerializableFile/test/TestSerializable \
	Fw/FilePacket/GTest \
	Fw/Types/GTest

R5_MODULES := \
	R5/Ports \
	\
	Drv/GpioDriverPorts \
	Drv/PwmDriverPorts \
	Drv/SerialDriverPorts \
	Drv/SpiDriverPorts \
	Drv/I2CDriverPorts \
	\
	R5/GpioAdapter \
	\
	R5/A2DDrv \
	R5/DmaDrv \
	R5/GpioDrv \
	R5/R5Mem \
	R5/R5Prm \
	R5/R5Time \
	R5/SpiMasterDrv \
	R5/SpiSlaveDrv \
	R5/UartDrv \
	R5/I2CDrv \
	\
	R5/TiHal \
	R5/R5FlashApi

ZMQ_MODULES := \
	fprime-zmq/zmq \
	fprime-zmq/zmq-adapter \
	fprime-zmq/zmq-pub \
	fprime-zmq/zmq-sub

COMMON_MODULES := \
	Common/Ports

OS_MODULES := \
	Os

CFDP_MODULES := \
	CFDP/Checksum

CFDP_GTEST_MODULES := \
	CFDP/Checksum/GTest

UTILS_MODULES := \
	Utils/Hash

# dependent on turbojpeg and zmq
SVC_EXTRA_MODULES := \
	Svc/ImgComp \
	Svc/ImgTlm

SVC_MODULES := \
	Svc/BufferManager \
	Svc/BufferAccumulator \
	Svc/BufferLogger \
	Svc/CmdDispatcher \
	Svc/CmdSequencer \
	Svc/Seq \
	Svc/GndIf \
	Svc/ActiveRateGroup \
	Svc/PassiveRateGroup \
	Svc/RateGroupDriver \
	Svc/RateGroupDecoupler \
	Svc/Sched \
	Svc/ComLogger \
	Svc/SocketGndIf \
	Svc/BuffGndSockIf \
	Svc/TlmChan \
	Svc/SerLogger \
	Svc/ActiveFileLogger \
	Svc/PassiveTextLogger \
	Svc/PassiveConsoleTextLogger \
	Svc/Time \
	Svc/Cycle \
	Svc/LinuxTime \
	Svc/ActiveLogger \
	Svc/Fatal \
	Svc/PolyIf \
	Svc/PolyDb \
	Svc/PrmDb \
	Svc/Ping \
	Svc/Health \
	Svc/WatchDog \
	Svc/FileUplink \
	Svc/FileDownlink \
	Svc/AssertFatalAdapter \
	Svc/FatalHandler \
	Svc/FileManager \
	Svc/SerialTextConverter \
	Svc/ActiveTextLogger \
	Svc/Tee \
	Svc/ActiveDecoupler \
	Svc/UdpSender \
	Svc/UdpReceiver \
	Svc/CameraFrame \
	Svc/IPCRelay

DRV_MODULES := \
	Drv/DataTypes \
	Drv/BlockDriver \
	Drv/LinuxGpioDriver \
	Drv/LinuxPwmDriver \
	Drv/LinuxSerialDriver \
	Drv/LinuxSpiDriver \
	Drv/LinuxI2CDriver \
	Drv/GpioDriverPorts \
	Drv/PwmDriverPorts \
	Drv/SerialDriverPorts \
	Drv/SpiDriverPorts \
	Drv/I2CDriverPorts \
	Drv/Altimeter/AltimeterPorts \
	Drv/Altimeter/AltimeterTypes

LLPROC_MODULES := \
	LLProc/HLRouter \
	Utils/Hash \
	LLProc/ShortLogQueue \
	LLProc/Ports \
	LLProc/LLDebug \
	LLProc/LLCycle \
	LLProc/LLCmdDispatcher \
	LLProc/LLPrmDb \
	LLProc/LLTlmChan

HLPROC_MODULES := \
	HLProc/HLRosIface \
	HLProc/LLRouter \
	HLProc/EventExpander \
	HLProc/Cfg

SNAPDRAGON_MODULES := \
	SnapdragonFlight/RpcCommon \
	SnapdragonFlight/HexRouter \
	SnapdragonFlight/DspRpcAllocator \
	SnapdragonFlight/DspRelay \
	SnapdragonFlight/BlspSerialDriver \
	SnapdragonFlight/BlspGpioDriver \
	SnapdragonFlight/BlspSpiDriver \
	SnapdragonFlight/SnapdragonHealth \
	SnapdragonFlight/MVCam \
	SnapdragonFlight/HiresCam \
	SnapdragonFlight/MVVislam

HEXAGON_MODULES := \
	SnapdragonFlight/RpcCommon \
	SnapdragonFlight/KraitRouter

QUEST_GNC_MODULES := \
	Gnc/Ctrl/LeeCtrl \
	Gnc/Ctrl/BasicMixer \
	Gnc/Est/ImuInteg \
	Gnc/Est/AttFilter \
	Gnc/Sysid/SigGen \
	Gnc/Utils/FrameTransform \
	Gnc/quest_gnc/src/diffeo \
	Gnc/quest_gnc/src/traj \
	Gnc/quest_gnc/src/ctrl \
	Gnc/quest_gnc/src/est \
	Gnc/quest_gnc/src/mixer \
	Gnc/quest_gnc/src/sysid

QUEST_GNC_HW_MODULES := \
	Gnc/Ctrl/ActuatorAdapter

REF_MODULES := \
	Ref/Top \
	Ref/RecvBuffApp \
	Ref/SendBuffApp \
	Ref/SignalGen \
	Ref/PingReceiver

ROS_PORT_MODULES := \
	ROS/Gen/std_msgs/Ports  \
	ROS/Gen/geometry_msgs/Ports      \
	ROS/Gen/nav_msgs/Ports           \
	ROS/Gen/std_srvs/Ports           \
	ROS/Gen/rosgraph_msgs/Ports	 \
	ROS/Gen/actionlib_msgs/Ports     \
	ROS/Gen/mav_msgs/Ports		 \
	ROS/Gen/sensor_msgs/Ports

ROS_TYPE_MODULES := \
	ROS/Gen/std_msgs/Types  \
	ROS/Gen/geometry_msgs/Types      \
	ROS/Gen/nav_msgs/Types           \
	ROS/Gen/std_srvs/Types           \
	ROS/Gen/rosgraph_msgs/Types	 \
	ROS/Gen/actionlib_msgs/Types     \
	ROS/Gen/mav_msgs/Types		 \
	ROS/Gen/sensor_msgs/Types

ROS_TYPE_PORT_MODULES_ALL := \
	$(ROS_TYPE_MODULES) \
	$(ROS_PORT_MODULES) \
	\
	ROS/Gen/diagnostic_msgs/Types    \
	ROS/Gen/stereo_msgs/Types        \
	ROS/Gen/trajectory_msgs/Types    \
	ROS/Gen/mav_planning_msgs/Types	 \
	ROS/Gen/shape_msgs/Types         \
	ROS/Gen/sensor_msgs/Types        \
	ROS/Gen/control_msgs/Types       \
	\
	ROS/Gen/diagnostic_msgs/Ports    \
	ROS/Gen/stereo_msgs/Ports        \
	ROS/Gen/trajectory_msgs/Ports    \
	ROS/Gen/mav_planning_msgs/Ports	 \
	ROS/Gen/shape_msgs/Ports         \
	ROS/Gen/sensor_msgs/Ports	 \
	ROS/Gen/control_msgs/Ports
#	ROS/Gen/visualization_msgs/Types \
#	ROS/Gen/visualization_msgs/Ports \


ROS_MODULES := \
	ROS/fprime_ws/src/fprime \
	\
	ROS/RosCycle \
	\
	ROS/RosTime \
	\
	$(ROS_TYPE_MODULES) \
	\
	$(ROS_PORT_MODULES)

Ref_MODULES := \
	\
	$(REF_MODULES) \
	\
	$(SVC_MODULES) \
	\
	$(DRV_MODULES) \
	\
	$(FW_MODULES) \
	\
	$(OS_MODULES) \
	\
	$(CFDP_MODULES) \
	\
	$(UTILS_MODULES)

SDREF_DEPLOYMENT_MODULES := \
	HEXREF/Rpc \
	SDREF/Top

SDREF_MODULES := \
	\
	$(SDREF_DEPLOYMENT_MODULES) \
	\
	Drv/ForceTorque/ATINetbox \
	\
	$(ZMQ_MODULES) \
	\
	$(HLPROC_MODULES) \
	\
	$(COMMON_MODULES) \
	\
	$(QUEST_GNC_MODULES) \
	$(QUEST_GNC_HW_MODULES) \
	\
	$(SNAPDRAGON_MODULES) \
	\
	$(SVC_MODULES) \
	\
	$(SVC_EXTRA_MODULES) \
	\
	$(DRV_MODULES) \
	\
	$(ROS_MODULES) \
	\
	$(FW_MODULES) \
	\
	$(OS_MODULES) \
	\
	$(CFDP_MODULES) \
	\
	$(UTILS_MODULES)

BASEREF_DEPLOYMENT_MODULES := \
	BASEREF/Top

BASEREF_MODULES := \
	\
	$(BASEREF_DEPLOYMENT_MODULES) \
	\
	Drv/ForceTorque/ATINetbox \
	\
	$(COMMON_MODULES) \
	\
	$(SVC_MODULES) \
	\
	$(DRV_MODULES) \
	\
	$(ROS_MODULES) \
	\
	$(FW_MODULES) \
	\
	$(OS_MODULES) \
	\
	$(CFDP_MODULES) \
	\
	$(UTILS_MODULES)

SIMREF_DEPLOYMENT_MODULES := \
	SIMREF/RotorSDrv \
	SIMREF/GazeboManipIf \
	SIMREF/Top

SIMREF_MODULES := \
	\
	$(QUEST_GNC_MODULES) \
	\
	$(SIMREF_DEPLOYMENT_MODULES) \
	\
	$(SVC_MODULES) \
	\
	$(DRV_MODULES) \
	\
	$(ROS_MODULES) \
	\
	$(ROS_TYPE_PORT_MODULES_ALL) \
	\
	$(FW_MODULES) \
	\
	$(OS_MODULES) \
	\
	$(CFDP_MODULES) \
	\
	$(UTILS_MODULES)

HEXREF_DEPLOYMENT_MODULES := \
	HEXREF/Top \
	HEXREF/Rpc

HEXREF_MODULES := \
	$(ROS_TYPE_MODULES) \
	$(ROS_PORT_MODULES) \
	\
	$(QUEST_GNC_MODULES) \
	$(QUEST_GNC_HW_MODULES) \
	\
	$(HEXAGON_MODULES) \
	\
	$(HEXREF_DEPLOYMENT_MODULES) \
	\
	Drv/IMU/MPU9250 \
	Drv/Mavlink/ActuatorControls \
	Drv/PwmDriverPorts \
	Drv/GpioDriverPorts \
	Drv/SerialDriverPorts \
	Drv/SpiDriverPorts \
	Drv/I2CDriverPorts \
	Drv/LinuxGpioDriver \
	Drv/LinuxSpiDriver \
	Drv/LinuxI2CDriver \
	Drv/LinuxPwmDriver \
	\
	Svc/BufferManager \
	Svc/CmdDispatcher \
	Svc/CmdSequencer \
	Svc/Seq \
	Svc/ActiveRateGroup \
	Svc/PassiveRateGroup \
	Svc/RateGroupDriver \
	Svc/RateGroupDecoupler \
	Svc/Sched \
	Svc/PassiveTextLogger \
	Svc/PassiveConsoleTextLogger \
	Svc/Time \
	Svc/Cycle \
	Svc/LinuxTime \
	Svc/ActiveLogger \
	Svc/Fatal \
	Svc/PolyIf \
	Svc/PolyDb \
	Svc/PrmDb \
	Svc/Ping \
	Svc/Health \
	Svc/WatchDog \
	Svc/AssertFatalAdapter \
	Svc/FatalHandler \
	Svc/ActiveDecoupler \
	\
	$(FW_MODULES) \
	\
	$(UTILS_MODULES) \
	\
	$(OS_MODULES) \
	\
	$(CFDP_MODULES) \
	\
	$(UTILS_MODULES) \
	\
	$(COMMON_MODULES) \
	\
	LLProc/ShortLogQueue \
	LLProc/LLCmdDispatcher \
	LLProc/LLTlmChan
#Svc/ComLogger

DSPRELAY_MODULES := SnapdragonFlight/DspRelay \
	SnapdragonFlight/RpcCommon \
	DSPRELAY/Top

MINRPC_MODULES := \
	MINRPC/Top \
	HEXREF/Rpc \
	\
	$(FW_MODULES) \
	\
	$(OS_MODULES) \
	\
	$(UTILS_MODULES) \
	\
	$(CFDP_MODULES) \
	\
	Svc/Sched \
	\
	SnapdragonFlight/HexRouter \
	SnapdragonFlight/KraitRouter \
	SnapdragonFlight/RpcCommon

TESTRPC_MODULES := \
	TESTRPC/Top \
	HEXREF/Rpc \
	\
	$(ROS_TYPE_MODULES) \
	$(ROS_PORT_MODULES) \
	\
	$(QUEST_GNC_MODULES) \
	$(QUEST_GNC_HW_MODULES) \
	\
	$(HEXAGON_MODULES) \
	\
	Drv/IMU/MPU9250 \
	Drv/PwmDriverPorts \
	Drv/GpioDriverPorts \
	Drv/SerialDriverPorts \
	Drv/SpiDriverPorts \
	Drv/I2CDriverPorts \
	Drv/LinuxGpioDriver \
	Drv/LinuxSpiDriver \
	Drv/LinuxI2CDriver \
	Drv/LinuxPwmDriver \
	\
	Svc/BufferManager \
	Svc/CmdDispatcher \
	Svc/CmdSequencer \
	Svc/Seq \
	Svc/ActiveRateGroup \
	Svc/PassiveRateGroup \
	Svc/RateGroupDriver \
	Svc/RateGroupDecoupler \
	Svc/Sched \
	Svc/PassiveTextLogger \
	Svc/PassiveConsoleTextLogger \
	Svc/Time \
	Svc/Cycle \
	Svc/LinuxTime \
	Svc/ActiveLogger \
	Svc/Fatal \
	Svc/PolyIf \
	Svc/PolyDb \
	Svc/PrmDb \
	Svc/Ping \
	Svc/Health \
	Svc/WatchDog \
	Svc/AssertFatalAdapter \
	Svc/FatalHandler \
	\
	$(FW_MODULES) \
	\
	$(UTILS_MODULES) \
	\
	$(OS_MODULES) \
	\
	$(CFDP_MODULES) \
	\
	$(UTILS_MODULES) \
	\
	$(COMMON_MODULES) \
	\
	LLProc/ShortLogQueue

R5REF_DEPLOYMENT_MODULES := \
	R5REF/Top

R5REF_MODULES := \
	$(ROS_TYPE_MODULES) \
	$(ROS_PORT_MODULES) \
	\
	$(COMMON_MODULES) \
	\
	$(QUEST_GNC_MODULES) \
	$(QUEST_GNC_HW_MODULES) \
	\
	$(LLPROC_MODULES) \
	\
	$(R5REF_DEPLOYMENT_MODULES) \
	\
	$(R5_MODULES) \
	\
	Drv/IMU/MPU9250 \
	Drv/Mavlink/ActuatorControls \
	Drv/Altimeter/LIDARLiteV3 \
	\
	Svc/PassiveRateGroup \
	Svc/RateGroupDriver \
	\
	Svc/Sched \
	Svc/Time \
	Svc/Cycle \
	\
	Drv/PwmDriverPorts \
	Drv/SerialDriverPorts \
	Drv/SpiDriverPorts \
	Drv/I2CDriverPorts \
	Drv/Altimeter/AltimeterPorts \
	Drv/Altimeter/AltimeterTypes \
	\
	Os \
	\
	$(FW_MODULES)

R5RELAY_MODULES := \
	$(COMMON_MODULES) \
	\
	Drv/LinuxSerialDriver \
	Drv/SerialDriverPorts\
	\
	Svc/PassiveRateGroup \
	Svc/GndIf \
	Svc/SocketGndIf \
	\
	Svc/Sched \
	Svc/Cycle \
	Svc/Ping \
	Svc/PolyIf \
	Svc/PolyDb \
	Svc/Time \
	Svc/LinuxTime \
	Svc/ActiveLogger \
	Svc/Fatal \
	\
	HLProc/LLRouter \
	HLProc/EventExpander \
	\
	Os \
	\
	R5RELAY/Top \
	\
	$(FW_MODULES) \
	\
	$(UTILS_MODULES)


ACDEVTEST_MODULES := \
	Autocoders/test/active_tester \
	Autocoders/test/app1 \
	Autocoders/test/app2 \
	Autocoders/test/cnt_only \
	Autocoders/test/command1 \
	Autocoders/test/command2 \
	Autocoders/test/command_res \
	Autocoders/test/command_multi_inst \
	Autocoders/test/command_string \
	Autocoders/test/command_tester \
	Autocoders/test/comp_diff_namespace \
	Autocoders/test/comp_no_namespace \
	Autocoders/test/enum1port \
	Autocoders/test/enum_return_port \
	Autocoders/test/event1 \
	Autocoders/test/event2 \
	Autocoders/test/event_throttle \
	Autocoders/test/event_enum \
	Autocoders/test/event_multi_inst \
	Autocoders/test/event_string \
	Autocoders/test/ext_dict \
	Autocoders/test/log1 \
	Autocoders/test/log_tester \
	Autocoders/test/main \
	Autocoders/test/noargport \
	Autocoders/test/param1 \
	Autocoders/test/param2 \
	Autocoders/test/param_enum \
	Autocoders/test/param_multi_inst \
	Autocoders/test/param_string \
	Autocoders/test/param_tester \
	Autocoders/test/time_tester \
	#Autocoders/test/queued1 \
	\ # Autocoders/test/partition \
	Autocoders/test/pass_by_attrib \
	\ # Autocoders/test/passive \
	Autocoders/test/port_nogen \
	Autocoders/test/port_return_type \
	Autocoders/test/serialize_enum \
	Autocoders/test/serialize_stringbuffer \
	Autocoders/test/serialize_template \
	Autocoders/test/serialize_user \
	Autocoders/test/serialize1 \
	Autocoders/test/serialize2 \
	Autocoders/test/serialize3 \
	Autocoders/test/stress \
	Autocoders/test/string_port \
	Autocoders/test/telem_tester \
	Autocoders/test/tlm_enum \
	Autocoders/test/tlm_string \
	Autocoders/test/tlm1 \
	Autocoders/test/tlm2 \
	Autocoders/test/tlm_onchange \
	Autocoders/test/tlm_multi_inst \
	Autocoders/test/interface1 \
	Autocoders/test/port_loopback \
	Autocoders/test/serial_passive \
    \
	Autocoders/templates

acdev_MODULES := \
	$(FW_MODULES) \
	\
	$(OS_MODULES) \
	\
	$(ACDEVTEST_MODULES)


# Other modules to build, but not to link with deployment binaries
OTHER_MODULES := \
	gtest \
	Os/Stubs \
	Fw/Test \
	ROS/Gen \
	$(FW_GTEST_MODULES)

# List deployments

DEPLOYMENTS := Ref acdev SDREF SIMREF HEXREF TESTRPC R5REF BASEREF DSPRELAY MINRPC R5RELAY

# Location of ground/gse software. Autocoded dictionary elements are copied here.
GDS_MODULE := Gse
