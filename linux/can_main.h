/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/
 
#ifndef __CAN_MAIN_H__
#define __CAN_MAIN_H__

#include <linux/interrupt.h>
#include <linux/pci.h>
#include <asm/semaphore.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/pci.h>

#include "icp_can.h"
#include "can_fifo.h"
#include "icp_can_user.h"

/* Parasoft fixes */
#define NUM_NODES 2000

#define MAX_CAN_DEVICES        2
#define DRIVER_NAME            "can"

/*****************************************************************************
 * CAN OS context structure.
 *****************************************************************************/
typedef struct can_os {
     icp_can_handle_t can;                  /* CAN: device handle        */
    unsigned int opened;                    /* Linux opened device       */
    unsigned int can_num;                   /* Linux: CAN Number         */
    void *pci_remap;                        /* Linux: MMap regs          */
    struct pci_dev *dev;                    /* Linux: PCI Device         */
    unsigned int irq;                       /* Linux: IRQ                */
    unsigned int int_status;                /* Interrupt status          */
    int block_mode;                         /* Blocking / non-blocking   */
    icp_can_handle_t rx_fifo;               /* Rx FIFO                   */
    wait_queue_head_t read_wait_queue;      /* Linux: Read wait queue    */
    wait_queue_head_t write_wait_queue;     /* Linux: Write wait queue   */
    unsigned int write_wait_flag;           /* Linux: Write wait flag    */
    wait_queue_head_t pm_wait_queue;        /* Linux: PM wait queue      */
    spinlock_t int_spinlock;                /* Linux: Interrupt sl       */
    unsigned long int_flags;                /* Linux: Interrupt flags    */
    spinlock_t open_spinlock;               /* Linux: Open sl            */
    unsigned char *pm_cfg[64];              /* Linux: PM - PCI Cfg Space */
	unsigned int is_suspending;				/* Linux: Is suspending state*/
	struct inode *inode;					/* Linux: inode			     */
    icp_can_timing_t timing;                /* CAN: timing               */
    icp_can_run_mode_t run_mode;            /* CAN: run mode             */
    icp_can_listen_mode_t listen_mode;      /* CAN: listen mode          */
    icp_can_arbiter_t arbiter_mode;         /* CAN: arbiter mode         */
    unsigned int tx_enable[NUM_TX_BUFFS];   /* CAN: Tx buffer state      */
    unsigned int rx_enable[NUM_RX_BUFFS];   /* CAN: Rx buffer state      */
    unsigned int rx_link[NUM_RX_BUFFS];     /* CAN: Rx link set          */
    unsigned int int_enables;               /* CAN: ints enabled         */
	icp_can_rx_filter_t rx_filter[NUM_RX_BUFFS]; /* CAN: Rx filters      */
} can_os_t;
  

/*****************************************************************************
 * PCI callbacks and data structures.
 *****************************************************************************/
int  can_pci_probe(
    struct pci_dev *dev, 
    const struct pci_device_id *id);
    
void can_pci_remove(struct pci_dev *dev);

int  can_pci_suspend(
    struct pci_dev *dev, 
    pm_message_t state);
    
int  can_pci_resume(struct pci_dev *dev);

/*****************************************************************************
 * Standard file operations and structures.
 *****************************************************************************/
int can_open(
    struct inode *inode, 
    struct file *filp);

int can_release(
    struct inode *inode, 
    struct file *filp);

ssize_t can_read(
    struct file *filep, 
    char __user *buf, 
    size_t count, 
    loff_t *f_pos);

ssize_t can_write(
    struct file *filp, 
    const char __user *buf, 
    size_t count, 
    loff_t *f_pos);

int icp_can_reset(
    can_os_t *can_os);
    
int can_dev_io(
    struct inode *inode, 
    struct file *filp, 
    unsigned int cmd, 
    unsigned long arg);

irqreturn_t can_irq_handler(
    int irq, 
    void *dev_id, 
    struct pt_regs *regs);

void can_tasklet(
    unsigned long arg
    );
    
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

