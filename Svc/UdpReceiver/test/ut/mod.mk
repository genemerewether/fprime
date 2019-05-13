
TEST_SRC = main.cpp \
	Tester.cpp \
<<<<<<< HEAD
	GTestBase.cpp \
	TesterBase.cpp
=======
	Handcode/GTestBase.cpp \
	Handcode/TesterBase.cpp
>>>>>>> upstream-pub-fprime/master
	
TEST_MODS = Svc/UdpReceiver \
			Svc/Sched \
			Fw/Tlm \
			Fw/Comp \
			Fw/Log \
			Fw/Obj \
			Fw/Port \
			Fw/Time \
			Fw/Buffer \
			Fw/Types \
			Os \
<<<<<<< HEAD
			gtest
=======
			gtest


COMPARGS = -I$(CURDIR)/test/ut/Handcode
>>>>>>> upstream-pub-fprime/master
