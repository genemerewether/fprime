TEST_SRC = 	Handcode/TesterBase.cpp \
			Tester.cpp \
			main.cpp

TEST_MODS = Drv/LinuxSpiDriver \
			Drv/SpiDriverPorts \
			Fw/Buffer \
			Fw/Tlm \
			Fw/Comp \
			Fw/Cmd \
			Fw/Log \
			Fw/Obj \
			Fw/Port \
			Fw/Time \
			Fw/Types \
			Os



COMPARGS = -I$(CURDIR)/test/ut/Handcode
