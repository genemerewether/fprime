#
#   Copyright 2004-20014, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged.
#
#

SRC_DARWIN = IPCActiveComponentBase.cpp \
             IPCQueuedComponentBase.cpp

SRC_LINUX = IPCActiveComponentBase.cpp \
            IPCQueuedComponentBase.cpp

SRC_LINUXRT = IPCActiveComponentBase.cpp \
              IPCQueuedComponentBase.cpp

SRC = \
    ActiveComponentBase.cpp \
    QueuedComponentBase.cpp \
	PassiveComponentBase.cpp

HDR = \
    ActiveComponentBase.hpp \
    QueuedComponentBase.hpp \
	PassiveComponentBase.hpp \
    IPCActiveComponentBase.hpp \
    IPCQueuedComponentBase.hpp

