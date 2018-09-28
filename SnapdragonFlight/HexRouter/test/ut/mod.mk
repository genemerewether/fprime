TEST_SRC_LINUX = main.cpp TesterBase.cpp GTestBase.cpp Tester.cpp
TEST_SRC_DARWIN = main.cpp TesterBase.cpp GTestBase.cpp Tester.cpp
TEST_SRC_CYGWIN = main.cpp TesterBase.cpp GTestBase.cpp Tester.cpp
TEST_SRC_SDFLIGHT = ut_hexrtr_stub.c main.cpp TesterBase.cpp GTestBase.cpp Tester.cpp

TEST_MODS = \
	Svc/Sched \
	Fw/Cmd \
	Fw/Tlm \
	Fw/Comp \
	Fw/Com \
	Fw/Log \
	Fw/Obj \
	Fw/Port \
	Fw/Time \
	Fw/Buffer \
	Fw/Types \
	Utils/Hash \
	Os

TEST_MODS_LINUX = SnapdragonFlight/HexRouter gtest
TEST_MODS_DARWIN = SnapdragonFlight/HexRouter gtest
TEST_MODS_CYGWIN = SnapdragonFlight/HexRouter gtest
TEST_MODS_SDFLIGHT = SnapdragonFlight/HexRouter gtest

TEST_MODS_DSPAL = SnapdragonFlight/KraitRouter SnapdragonFlight/RpcCommon
TEST_SRC_DSPAL = ut_hexrtr_skel.c dspal.cpp
