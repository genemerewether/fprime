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
	Svc/LinuxTimer \
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
	Svc/QueuedDecoupler \
	Svc/UdpSender \
	Svc/UdpReceiver \
	Svc/CameraFrame \
	Svc/IPCRelay \
	Svc/TimeSyncOffset \
	Svc/TimeConvert

DEMO_DRV_MODULES := \
	Drv/DataTypes \
	Drv/BlockDriver \
	Drv/GpioDriverPorts

LINUX_DRV_MODULES := \
	Drv/LinuxGpioDriver \
	Drv/LinuxPwmDriver \
	Drv/LinuxSerialDriver \
	Drv/LinuxSpiDriver \
	Drv/LinuxI2CDriver \
	Drv/GpioDriverPorts \
	Drv/PwmDriverPorts \
	Drv/SerialDriverPorts \
	Drv/SpiDriverPorts \
	Drv/I2CDriverPorts

DEV_DRV_MODULES := \
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
	HLProc/LLRouter \
	HLProc/EventExpander \
	HLProc/Cfg

HLPROC_ROS_MODULES := \
	HLProc/HLRosIface

SNAPDRAGON_MODULES := \
	SnapdragonFlight/RpcCommon \
	SnapdragonFlight/HexRouter \
	SnapdragonFlight/DspOffset \
	SnapdragonFlight/DspRpcAllocator \
	SnapdragonFlight/DspRelay \
	SnapdragonFlight/BlspSerialDriver \
	SnapdragonFlight/BlspGpioDriver \
	SnapdragonFlight/BlspSpiDriver \
	SnapdragonFlight/BlspI2CDriver \
	SnapdragonFlight/BlspPwmDriver \
	SnapdragonFlight/SnapdragonHealth \
	SnapdragonFlight/MVCam \
	SnapdragonFlight/StereoCam \
	SnapdragonFlight/HiresCam \
	SnapdragonFlight/MVVislam \
	SnapdragonFlight/MVDFS

HEXAGON_MODULES := \
	SnapdragonFlight/RpcCommon \
	SnapdragonFlight/KraitRouter

QUEST_GNC_ROSIFACE_MODULES := \
	Gnc/Ctrl/MultirotorCtrlIface \
	Gnc/Utils/AckermannIface \
	Gnc/Est/FilterIface \
	Gnc/Est/GroundTruthIface

QUEST_GNC_MODULES := \
	Gnc/Ctrl/LeeCtrl \
	Gnc/Ctrl/Se3Ctrl \
	Gnc/Ctrl/BasicMixer \
	Gnc/Ctrl/WrenchMixer \
	Gnc/Est/ImuInteg \
	Gnc/Est/AttFilter \
	Gnc/Sysid/SigGen \
	Gnc/Utils/FixedAxisSe3Adapter \
	Gnc/Utils/AckermannConverter \
	Gnc/Utils/FrameTransform \
	Gnc/Utils/ImuProc \
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
	ROS/Gen/sensor_msgs/Ports	 \
	ROS/Gen/ackermann_msgs/Ports

ROS_TYPE_MODULES := \
	ROS/Gen/std_msgs/Types  \
	ROS/Gen/geometry_msgs/Types      \
	ROS/Gen/nav_msgs/Types           \
	ROS/Gen/std_srvs/Types           \
	ROS/Gen/rosgraph_msgs/Types	 \
	ROS/Gen/actionlib_msgs/Types     \
	ROS/Gen/mav_msgs/Types		 \
	ROS/Gen/sensor_msgs/Types	 \
	ROS/Gen/ackermann_msgs/Types

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
	ROS/RosSeq \
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
	$(DEMO_DRV_MODULES) \
	\
	$(FW_MODULES) \
	\
	$(OS_MODULES) \
	\
	$(CFDP_MODULES) \
	\
	$(UTILS_MODULES)

BLIMPREF_DEPLOYMENT_MODULES := \
	BLIMPREF/Top

BLIMPREF_MODULES := \
	\
	$(BLIMPREF_DEPLOYMENT_MODULES) \
	\
	Drv/ForceTorque/ATINetbox \
	Drv/IMU/MPU9250 \
	\
	$(ZMQ_MODULES) \
	\
	$(HLPROC_MODULES) \
	$(HLPROC_ROS_MODULES) \
	\
	$(COMMON_MODULES) \
	\
	$(QUEST_GNC_MODULES) \
	$(QUEST_GNC_HW_MODULES) \
	$(QUEST_GNC_ROSIFACE_MODULES) \
	\
	$(SNAPDRAGON_MODULES) \
	\
	$(SVC_MODULES) \
	\
	$(SVC_EXTRA_MODULES) \
	\
	$(LINUX_DRV_MODULES) \
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

CARREF_DEPLOYMENT_MODULES := \
	CARREF/Top

CARREF_MODULES := \
	\
	$(CARREF_DEPLOYMENT_MODULES) \
	\
	Drv/ForceTorque/ATINetbox \
	Drv/IMU/MPU9250 \
	\
	$(ZMQ_MODULES) \
	\
	$(HLPROC_MODULES) \
	$(HLPROC_ROS_MODULES) \
	\
	$(COMMON_MODULES) \
	\
	$(QUEST_GNC_MODULES) \
	$(QUEST_GNC_HW_MODULES) \
	$(QUEST_GNC_ROSIFACE_MODULES) \
	\
	$(SNAPDRAGON_MODULES) \
	\
	$(SVC_MODULES) \
	\
	$(SVC_EXTRA_MODULES) \
	\
	$(LINUX_DRV_MODULES) \
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

SDREF_DEPLOYMENT_MODULES := \
	HEXREF/Rpc \
	SDREF/Top

SDREF_MODULES := \
	\
	$(SDREF_DEPLOYMENT_MODULES) \
	\
	Drv/Mavlink/GPSPosAdapter \
	Drv/ForceTorque/ATINetbox \
	\
	$(ZMQ_MODULES) \
	\
	$(HLPROC_MODULES) \
	$(HLPROC_ROS_MODULES) \
	\
	$(COMMON_MODULES) \
	\
	$(QUEST_GNC_MODULES) \
	$(QUEST_GNC_HW_MODULES) \
	$(QUEST_GNC_ROSIFACE_MODULES) \
	\
	$(SNAPDRAGON_MODULES) \
	\
	$(SVC_MODULES) \
	\
	$(SVC_EXTRA_MODULES) \
	\
	$(LINUX_DRV_MODULES) \
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
	$(LINUX_DRV_MODULES) \
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

SIMREF_GENERAL_MODULES := \
	SIMREF/RotorSDrv \
	SIMREF/GazeboManipIf \
	\
	$(QUEST_GNC_MODULES) \
	$(QUEST_GNC_ROSIFACE_MODULES) \
	\
	$(SVC_MODULES) \
	\
	$(LINUX_DRV_MODULES) \
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

SIMREF_MODULES := \
	SIMREF/Top \
	$(SIMREF_GENERAL_MODULES)

HEXREF_GENERAL_MODULES := \
	\
	ROS/Gen/std_msgs/Ports  \
	ROS/Gen/geometry_msgs/Ports      \
	ROS/Gen/nav_msgs/Ports           \
	ROS/Gen/mav_msgs/Ports		 \
	ROS/Gen/sensor_msgs/Ports	\
	ROS/Gen/ackermann_msgs/Ports	\
	\
	ROS/Gen/std_msgs/Types  \
	ROS/Gen/geometry_msgs/Types      \
	ROS/Gen/nav_msgs/Types           \
	ROS/Gen/mav_msgs/Types		 \
	ROS/Gen/sensor_msgs/Types	\
	ROS/Gen/ackermann_msgs/Types	\
	\
	$(QUEST_GNC_MODULES) \
	$(QUEST_GNC_HW_MODULES) \
	\
	$(HEXAGON_MODULES) \
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
	Svc/QueuedDecoupler \
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

HEXREF_MODULES := \
	HEXREF/Top \
	HEXREF/Rpc \
	$(HEXREF_GENERAL_MODULES)

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
	$(HEXREF_GENERAL_MODULES)

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
	Svc/Fatal \
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
	Autocoders/Python/test/active_tester \
	Autocoders/Python/test/app1 \
	Autocoders/Python/test/app2 \
	Autocoders/Python/test/cnt_only \
	Autocoders/Python/test/command1 \
	Autocoders/Python/test/command2 \
	Autocoders/Python/test/command_res \
	Autocoders/Python/test/command_multi_inst \
	Autocoders/Python/test/command_string \
	Autocoders/Python/test/command_tester \
	Autocoders/Python/test/comp_diff_namespace \
	Autocoders/Python/test/comp_no_namespace \
	Autocoders/Python/test/enum1port \
	Autocoders/Python/test/enum_return_port \
	Autocoders/Python/test/event1 \
	Autocoders/Python/test/event2 \
	Autocoders/Python/test/event_throttle \
	Autocoders/Python/test/event_enum \
	Autocoders/Python/test/event_multi_inst \
	Autocoders/Python/test/event_string \
	Autocoders/Python/test/ext_dict \
	Autocoders/Python/test/log1 \
	Autocoders/Python/test/log_tester \
	Autocoders/Python/test/main \
	Autocoders/Python/test/noargport \
	Autocoders/Python/test/param1 \
	Autocoders/Python/test/param2 \
	Autocoders/Python/test/param_enum \
	Autocoders/Python/test/param_multi_inst \
	Autocoders/Python/test/param_string \
	Autocoders/Python/test/param_tester \
	Autocoders/Python/test/time_tester \
	#Autocoders/Python/test/queued1 \
	\ # Autocoders/Python/test/partition \
	Autocoders/Python/test/pass_by_attrib \
	\ # Autocoders/Python/test/passive \
	Autocoders/Python/test/port_nogen \
	Autocoders/Python/test/port_return_type \
	Autocoders/Python/test/serialize_enum \
	Autocoders/Python/test/serialize_stringbuffer \
	Autocoders/Python/test/serialize_template \
	Autocoders/Python/test/serialize_user \
	Autocoders/Python/test/serialize1 \
	Autocoders/Python/test/serialize2 \
	Autocoders/Python/test/serialize3 \
	Autocoders/Python/test/stress \
	Autocoders/Python/test/string_port \
	Autocoders/Python/test/telem_tester \
	Autocoders/Python/test/tlm_enum \
	Autocoders/Python/test/tlm_string \
	Autocoders/Python/test/tlm1 \
	Autocoders/Python/test/tlm2 \
	Autocoders/Python/test/tlm_onchange \
	Autocoders/Python/test/tlm_multi_inst \
	Autocoders/Python/test/interface1 \
	Autocoders/Python/test/port_loopback \
	Autocoders/Python/test/serial_passive \
    \
	Autocoders/Python/templates

ACDEVTEST_MODULES := \
	Autocoders/Python/test/active_tester \
	Autocoders/Python/test/app1 \
	Autocoders/Python/test/app2 \
	Autocoders/Python/test/cnt_only \
	Autocoders/Python/test/command1 \
	Autocoders/Python/test/command2 \
	Autocoders/Python/test/command_res \
	Autocoders/Python/test/command_multi_inst \
	Autocoders/Python/test/command_string \
	Autocoders/Python/test/command_tester \
	Autocoders/Python/test/comp_diff_namespace \
	Autocoders/Python/test/comp_no_namespace \
	Autocoders/Python/test/enum1port \
	Autocoders/Python/test/enum_return_port \
	Autocoders/Python/test/event1 \
	Autocoders/Python/test/event2 \
	Autocoders/Python/test/event_throttle \
	Autocoders/Python/test/event_enum \
	Autocoders/Python/test/event_multi_inst \
	Autocoders/Python/test/event_string \
	Autocoders/Python/test/ext_dict \
	Autocoders/Python/test/log1 \
	Autocoders/Python/test/log_tester \
	Autocoders/Python/test/main \
	Autocoders/Python/test/noargport \
	Autocoders/Python/test/param1 \
	Autocoders/Python/test/param2 \
	Autocoders/Python/test/param_enum \
	Autocoders/Python/test/param_multi_inst \
	Autocoders/Python/test/param_string \
	Autocoders/Python/test/param_tester \
	Autocoders/Python/test/time_tester \
	#Autocoders/Python/test/queued1 \
	\ # Autocoders/Python/test/partition \
	Autocoders/Python/test/pass_by_attrib \
	\ # Autocoders/Python/test/passive \
	Autocoders/Python/test/port_nogen \
	Autocoders/Python/test/port_return_type \
	Autocoders/Python/test/serialize_enum \
	Autocoders/Python/test/serialize_stringbuffer \
	Autocoders/Python/test/serialize_template \
	Autocoders/Python/test/serialize_user \
	Autocoders/Python/test/serialize1 \
	Autocoders/Python/test/serialize2 \
	Autocoders/Python/test/serialize3 \
	Autocoders/Python/test/stress \
	Autocoders/Python/test/string_port \
	Autocoders/Python/test/telem_tester \
	Autocoders/Python/test/tlm_enum \
	Autocoders/Python/test/tlm_string \
	Autocoders/Python/test/tlm1 \
	Autocoders/Python/test/tlm2 \
	Autocoders/Python/test/tlm_onchange \
	Autocoders/Python/test/tlm_multi_inst \
	Autocoders/Python/test/interface1 \
	Autocoders/Python/test/port_loopback \
	Autocoders/Python/test/serial_passive \
    \
	Autocoders/Python/templates
	
RPI_APP_MODULES := \
	RPI/Top \
	RPI/RpiDemo
	
RPI_MODULES := \
	\
	$(RPI_APP_MODULES) \
	\
	$(SVC_MODULES) \
	\
	$(LINUX_DRV_MODULES) \
	\
	$(CFDP_MODULES) \
  	\
	$(FW_MODULES) \
	\
	$(OS_MODULES) \
	\
  	$(UTILS_MODULES)

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

DEPLOYMENTS := Ref acdev SDREF SIMREF HEXREF TESTRPC R5REF BASEREF DSPRELAY MINRPC R5RELAY BLIMPREF CARREF RPI

