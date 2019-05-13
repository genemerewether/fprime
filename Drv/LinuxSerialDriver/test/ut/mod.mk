TEST_SRC = 	Handcode/TesterBase.cpp \
			Tester.cpp \
			main.cpp

TEST_MODS = Drv/LinuxSerialDriver \
			Drv/SerialDriverPorts \
			Fw/Tlm \
			Fw/Comp \
			Fw/Cmd \
			Fw/Log \
			Fw/Obj \
			Fw/Port \
			Fw/Time \
			Fw/Types \
			Fw/Com \
			Fw/Buffer \
			Svc/Sched \
			Os



COMPARGS = -I$(CURDIR)/test/ut/Handcode
