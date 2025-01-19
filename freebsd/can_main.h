/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/

#ifndef __CAN_MAIN_H__
#define __CAN_MAIN_H__

#include <sys/types.h>
#include <sys/module.h>
#include <sys/systm.h>
#include <sys/errno.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/conf.h>
#include <sys/uio.h>
#include <sys/malloc.h>
#include <sys/bus.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <sys/fcntl.h>
#include <sys/sleepqueue.h>
#include <sys/ioccom.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include "icp_can.h"
#include "can_fifo.h"
#include "icp_can_user.h"

/* Parasoft fixes */
#define VERSION   0x10000 /* hex is used because leading 0 is octal representation */
#define WORD      2
#define PERMS     0600
#define NUM_NODES 2000

#define MAX_CAN_DEVICES      2
#define DRIVER_NAME          "can"

/*****************************************************************************
 * CAN OS context structure.
 *****************************************************************************/
typedef struct can_os {
    icp_can_handle_t can;        /* Handle to the CAN device      */
    unsigned int can_num;         /* CAN number                    */
    unsigned int opened;          /* CAN open state                */
    device_t dev;                 /* BSD: PCI device               */
    struct cdev *cdev;            /* BSD: OS device                */
    int mem_rid;                  /* BSD: Memory resource id       */
    struct resource *mem_res;     /* BSD: Memory resource          */
    bus_space_tag_t mem_tag;      /* BSD: Memory tag               */
    bus_space_handle_t mem_bus;   /* BSD: Memory bus handle        */
    int irq_rid;                  /* BSD: IRQ resource id          */
    struct resource *irq_res;     /* BSD: IRQ resource             */
    void *irq_tag;                /* BSD: IRQ cookie               */
    void *pci_remap;              /* Remapped memory mapped regs   */
    int block_mode;               /* Blocking / non-blocking       */
    icp_can_handle_t rx_fifo;     /* Rx buffer FIFO                */
    int read_wait_id;             /* BSD: Read sleep/wakeup id     */
    int read_wait_flag;           /* BSD: Read sleep/wakeup flag   */
    int write_wait_id;            /* BSD: Write sleep/wakeup id    */
    int write_wait_flag;          /* BSD: Write sleep/wakeup flag  */
    struct mtx int_spinlock;      /* BSD: Interrupt spinlock       */
} can_os_t;

/*****************************************************************************
 * Standard file operation and PCI callbacks.
 *****************************************************************************/
static d_open_t can_open;
static d_close_t can_close;
static d_read_t can_read;
static d_write_t can_write;
static d_ioctl_t can_dev_io;

/*****************************************************************************
 * PCI functions.
 *****************************************************************************/
static int can_pci_probe(device_t dev);
static int can_pci_attach(device_t dev);
static int can_pci_detach(device_t dev);
void can_irq_handler(void *arg);

int icp_can_reset(can_os_t *can_os);

int can_ioctl(
    can_os_t *can_os,
    unsigned int ctl_code,
    void *in,
    void *out
    );

void can_ioctl_get_size(
    unsigned int ctl_code,
    size_t *in_size,
    size_t *out_size
    );

#endif /* ifndef __CAN_MAIN_H__ */

