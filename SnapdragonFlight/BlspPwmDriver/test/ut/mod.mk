#
#   Copyright 2004-2008, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged. Any commercial use must be negotiated with the Office
#   of Technology Transfer at the California Institute of Technology.

TEST_SRC = GTestBase.cpp \
			TesterBase.cpp \
			Tester.cpp \
			main.cpp

TEST_MODS = SnapdragonFlight/BlspPwmDriver \
			Drv/PwmDriverPorts \
			SnapdragonFlight/DspRelay \
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
