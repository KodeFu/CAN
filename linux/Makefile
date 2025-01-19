#########################################################################
#
#  Targets supported
#  all     - builds everything and installs
#  install - identical to all
#  clean   - clears derived objects except the .depend files
#  
# @par
# <COPYRIGHT_TAG>
############################################################################


####################Common variables and definitions########################
# Ensure The ENV_DIR environmental var is defined.
ifndef ENV_DIR
$(error ENV_DIR is undefined. Please set the path to your environment makefile \
        "-> setenv ENV_DIR <path>")
endif

#Add your project environment Makefile
include $(ENV_DIR)/environment.mk

#include the makefile with all the default and common Make variable definitions
include $(BUILDSYSTEM_PATH)/build_files/common.mk

#Add the name for the executable, Library or Module output definitions
OUTPUT_NAME=can

# List of Source Files to be compiled (to be in a single line or on different lines separated by a "\" and tab.
MODULE_SOURCES=can_main.c can_fifo.c icp_can.c

EXTRA_LDFLAGS += -whole-archive

#common includes between all supported OSes
INCLUDES= #e.g.-I $(API_DIR)\

# set KSRC, KOBJ to default values if not already set
#
KOBJ ?= /usr/src/kernels/linux
KSRC ?= /usr/src/kernels/linux

OUTPUT_PATH ?= /
EXTRA_LDFLAGS += -whole-archive

VERSION_FILE := $(KOBJ)/include/linux/version.h
UTS_REL_FILE := $(KSRC)/include/linux/utsrelease.h
CONFIG_FILE  := $(KOBJ)/include/linux/autoconf.h

 
# as of 2.6.16, kernel define UTS_RELEASE has been moved to utsrelease.h
# so check that file for kernel version string instead of version.h
USE_UTS_REL := $(shell [ -f $(UTS_REL_FILE) ] && echo "1")

# get the kernel version - we use this to find the correct install path
KVER := $(shell $(CC) $(CFLAGS) -E -dM $(UTS_REL_FILE) | grep UTS_RELEASE | \
          awk '{ print $$3 }' | sed 's/\"//g')

# set the install path
INSTDIR := /lib/modules/$(KVER)/kernel/drivers/$(OUTPUT_NAME)

driver: module
install:
	@echo installing $(OUTPUT_NAME);
	# remove all old versions of the driver
	find $(INSTALL_MOD_PATH)/lib/modules/$(KVER) -name $(OUTPUT_NAME).ko -exec rm -f {} \; || true
	install -D -m 644 $(FINAL_OUTPUT_DIR)$(OUTPUT_NAME).ko $(INSTALL_MOD_PATH)$(INSTDIR)/$(OUTPUT_NAME).ko
	/sbin/depmod -a
	/sbin/modprobe $(OUTPUT_NAME)

uninstall:
	@echo uninstalling $(OUTPUT_NAME);
	rm -f $(INSTDIR)/$(OUTPUT_NAME).ko ; \
	/sbin/depmod -a
	/sbin/rmmod $(OUTPUT_NAME)


###################Include rules makefiles########################
include $(BUILDSYSTEM_PATH)/build_files/rules.mk
###################End of Rules inclusion#########################
