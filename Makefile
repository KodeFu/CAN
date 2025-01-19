#####
#
# Naviagtion Makefile, to kick-off a build for all sub-dirs
#

define LOOP
@for dir in $(SUBDIRS); do \
      (echo ; echo $$dir :; cd $$dir && \
            make KSRC=$(KSRC) KOBJ=$(KOBJ) $@ || return 1) \
      done
endef

#If the TARGET_OS is not defined the check the Host machine and build for that OS, e.g. build the Linux code if on a Linux Box.
TARGET_OS ?= $(shell uname)

#Check the TARGET_OS and go into the correct SUBDIR
ifeq ($(TARGET_OS),Linux)
SUBDIRS=linux
else
ifeq ($(TARGET_OS),FreeBSD)
SUBDIRS=freebsd
endif
endif

#Check if there was in an error in finding the SUBDIRS
ifndef SUBDIRS
$(error There are no SUBDIRS specified, check TARGET_OS is correct)
endif

#List the rules that the makefile will passdown through the LOOP
default clean:
	$(LOOP)

