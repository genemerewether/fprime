# This file can be filled with known hosts that support a particular BUILD_SITE

# get the hostname

QUEST_HOST_LIST := genotype genie eneg dronezz

QUESTHOST := $(findstring $(HOSTNAME),$(QUEST_HOST_LIST))

ifneq ("$(QUESTHOST)","")
  BUILD_SITE := quest
endif
