# This Makefile goes in each module, and allows building of an individual module library.
# It is expected that each developer will add targets of their own for building and running
# tests, for example.

# derive module name from directory

MODULE_DIR = HLProc/LLRouter
MODULE = $(subst /,,$(MODULE_DIR))

BUILD_ROOT ?= $(subst /$(MODULE_DIR),,$(CURDIR))
export BUILD_ROOT

include $(BUILD_ROOT)/mk/makefiles/module_targets.mk

# Add module specific targets here

runtime_test: runtime_test_$(NATIVE_BUILD)

# Add module specific targets here
runtime_test_LINUX:
	@echo "Compiling runtime_test for Linux"
	@$(TIME) $(MAKE) -f $(BUILD_ROOT)/mk/makefiles/Makefile COMP=comp-ut BUILD=LINUX test_$(MODULE)testruntime_test

runtime_test_CYGWIN:
	@echo "Compiling runtime_test for Cygwin"
	@$(TIME) $(MAKE) -f $(BUILD_ROOT)/mk/makefiles/Makefile COMP=comp-ut BUILD=CYGWIN test_$(MODULE)testruntime_test

runtime_test_SDFLIGHT:
	@echo "Compiling runtime_test for SD Flight"
	@$(TIME) $(MAKE) -f $(BUILD_ROOT)/mk/makefiles/Makefile COMP=comp-ut-nocov BUILD=SDFLIGHT test_$(MODULE)testruntime_test

runtime_test_TIR5:
	@echo "Compiling runtime_test for R5"
	@$(TIME) $(MAKE) -f $(BUILD_ROOT)/mk/makefiles/Makefile COMP=comp-ut BUILD=TIR5 test_$(MODULE)testruntime_test

runtime_test_TIR5_clean:
	@echo "Cleaning runtime_test for R5"
	@$(TIME) $(MAKE) -f $(BUILD_ROOT)/mk/makefiles/Makefile COMP=comp-ut BUILD=TIR5 test_$(MODULE)testruntime_test_clean
