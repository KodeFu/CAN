
# <COPYRIGHT_TAG>

KMOD = can
SRCS = can_main.c icp_can.c can_fifo.c 
SRCS +=  device_if.h bus_if.h pci_if.h

# set the install path
INSTDIR := /boot/kernel/

default:
	make

clean:
	rm -f device_if.h bus_if.h pci_if.h export_syms
	rm -f *.o *.kld *.ko
	rm -f @ machine

install:
	@echo installing $(KMOD);
	echo $(KMOD)_load=\"YES\" >> /boot/loader.conf
	find $(INSTDIR) -name $(KMOD).ko -delete
	install -m 555 $(KMOD).ko $(INSTDIR)
	kldload $(KMOD).ko

uninstall:
	@echo uninstalling $(KMOD);
	kldunload $(KMOD).ko
	rm -f $(INSTDIR)$(KMOD).ko
	-sed -i -e '/$(KMOD)_load="YES"/d;' /boot/loader.conf

.include <bsd.kmod.mk>
