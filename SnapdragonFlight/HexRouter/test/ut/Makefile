# TODO(mereweth) - replace this with cleaner dependencies
idl: ut_hexrtr_skel.c ut_hexrtr_stub.c ut_hexrtr.h

ut_hexrtr_skel.c ut_hexrtr_stub.c ut_hexrtr.h: ut_hexrtr.idl
	$(HEXAGON_SDK_ROOT)/tools/qaic/Ubuntu14/qaic -mdll -I $(HEXAGON_SDK_ROOT)/incs/stddef ut_hexrtr.idl

.PHONY: idl
