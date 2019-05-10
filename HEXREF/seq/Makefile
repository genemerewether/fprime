# Create sequence files using tinyseqgen.sh

# ----------------------------------------------------------------------
# Makefile 
# ----------------------------------------------------------------------

SHELL = /bin/bash

SEQ_TXT := $(shell find . -type f -name '*.seq')
SEQ_BIN := $(SEQ_TXT:%.seq=%.bin)

MKFILE_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

all: $(SEQ_BIN)

clean:
	$(RM) $(SEQ_BIN)

%.bin: %.seq
	@echo "=== Compiling sequence $< ===" 	
	@$(MKFILE_DIR)/run_tinyseqgen_HEXREF.sh $<

print_time:
	@date -u +A"%Y-%jT%H:%M:%S%Z"
	@date +A"%Y-%jT%H:%M:%S%Z"

print_targets:
	@echo -e '$(subst MY_UNIQUE_TOKEN,\nGenerate,$(shell for f in $$(find . -type f -name "*.seq"); do echo MY_UNIQUE_TOKEN $${f/\.seq/\.bin} from $$f; done))'

hash: $(SEQ_BIN)
	$(foreach SEQ,$(SEQ_BIN),md5sum $(SEQ); )

load: $(SEQ_BIN)
	$(foreach SEQ,$(SEQ_BIN),adb push $(SEQ) /seq/; md5sum $(SEQ); adb shell md5sum /seq/$(notdir $(SEQ));)
	adb shell sync
