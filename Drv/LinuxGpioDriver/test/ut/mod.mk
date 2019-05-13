TEST_SRC = 	Handcode/TesterBase.cpp \
			Tester.cpp \
			main.cpp

TEST_MODS = Drv/LinuxGpioDriver \
			Drv/GpioDriverPorts \
			Svc/Cycle \
			Fw/Tlm \
			Fw/Comp \
			Fw/Cmd \
			Fw/Log \
			Fw/Time \
			Fw/Obj \
			Fw/Port \
			Fw/Types \
			Os \
			gtest



COMPARGS = -I$(CURDIR)/test/ut/Handcode
