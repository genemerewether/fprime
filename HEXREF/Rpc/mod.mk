#
#   Copyright 2004-2008, by the California Institute of Technology.
#   ALL RIGHTS RESERVED. United States Government Sponsorship
#   acknowledged. Any commercial use must be negotiated with the Office
#   of Technology Transfer at the California Institute of Technology.
#


SRC = 

SRC_SDFLIGHT = hexref_stub.c krait_wrap_rpc.c

SRC_DSPAL = hexref_skel.c #hex_wrap_rpc.c

# TODO (mereweth) - replace with some ipc bridge 
SRC_DARWIN =
SRC_LINUX =
SRC_CYGWIN =

HDR = hexref.h 

SUBDIRS = test
