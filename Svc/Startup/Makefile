# DO NOT ADD settings_local.bash TO THIS LIST
STARTUP_SRC=boot_counter.bash hash_validation.bash startup_helpers.bash \
						startup.bash logging.bash
STARTUP_TEST=test_hash_parse_compare.bats test_hash_load_compare.bats \
						 test_boot_counter.bats test_startup.bats test_startup_helpers.bats \
						 test_logging.bats

define VALIDATE
$(info Comparing checksum of local $(1) and remote $(2))
@bash -c '. ./hash_validation.bash 2> /dev/null \
				  && get_crc32_from_cksum local `cksum $(1)` 2> /dev/null \
				  && get_crc32_from_cksum remote `adb shell cksum $(2)` 2> /dev/null \
				  && (( $$local == $$remote ))'
endef

# DO NOT ADD settings_local.bash TO THIS LIST
load: settings.bash $(STARTUP_SRC)
	@$(foreach FILE,$^,adb push $(FILE) /startup/;)
	@$(foreach FILE,$^,$(call VALIDATE,$(FILE),/startup/$(FILE));)
	@adb push verbosity.bash /seq/verbosity.bash
	@adb push startup.conf /etc/init/startup.conf
	@$(call VALIDATE,startup.conf,/etc/init/startup.conf)

load_test: settings_local.bash $(STARTUP_SRC)
	$(info Copying working dir scripts to test dir)
	@$(foreach FILE,$(filter-out $<,$^),cp $(FILE) test/ut/$(FILE);)
	@cp settings_local.bash test/ut/settings.bash
	@cp verbosity.bash test/ut/seq/verbosity.bash

shell_test: load_test
	cd test/ut; /bin/bash ./startup.bash; cd ../../

test_%: load_test
	cd test/ut; bats $@.bats; cd ../..

test: load_test
	@$(foreach FILE,$(STARTUP_TEST),cd test/ut; echo "Testing $(FILE)"; \
		 															bats $(FILE); cd ../..; \
																	echo;)

load_sdref: load
	@adb push deployment_sdref.bash /startup/deployment.bash
	$(call VALIDATE,deployment_sdref.bash,/startup/deployment.bash)

load_carref: load
	@adb push deployment_carref.bash /startup/deployment.bash
	$(call VALIDATE,deployment_carref.bash,/startup/deployment.bash)

.PHONY: load load_sdref load_carref load_test test #$(foreach FILE,$(STARTUP_TEST),$(FILE:.bats=))
