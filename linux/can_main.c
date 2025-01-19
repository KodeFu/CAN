/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/
 /**************************************************************************
 * @ingroup CAN_GENERAL
 *
 * @file can_main.c
 *
 * @description
 *
 **************************************************************************/

#include "can_main.h"
#include "can_ioctl.h"

MODULE_AUTHOR("Intel(R) Corporation");
MODULE_DESCRIPTION("Controller Area Network Driver");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_VERSION("1.0.0");

dev_t dev;                    /* dynamically alloc'd dev nums */
struct cdev *can_dev;         /* char device reg'd by the driver */
can_os_t g_can_os[MAX_CAN_DEVICES];

#define CAN_PROC_0   "can0"  /* Names for proc interface, since request_irq */
#define CAN_PROC_1   "can1"  /* requires const char */

DECLARE_TASKLET(can_tasklet0, can_tasklet, (unsigned int) &(g_can_os[0]));
DECLARE_TASKLET(can_tasklet1, can_tasklet, (unsigned int) &(g_can_os[1]));

static struct pci_device_id can_pci_ids[] = {
    { PCI_DEVICE(ICP_CAN_PCI_VENDOR_ID, ICP_CAN_PCI_DEVICE_ID_0) },
    { PCI_DEVICE(ICP_CAN_PCI_VENDOR_ID, ICP_CAN_PCI_DEVICE_ID_1) },
    { 0, },
};

MODULE_DEVICE_TABLE(pci, can_pci_ids);

static struct pci_driver pci_ops = {
    .name = DRIVER_NAME,
    .id_table  = can_pci_ids,
    .probe     = can_pci_probe,
    .remove    = can_pci_remove,
    .suspend   = can_pci_suspend,
    .resume    = can_pci_resume
};

struct file_operations file_ops = {
    .owner        = THIS_MODULE,
    .read        = can_read,
    .write        = can_write,
    .ioctl        = can_dev_io,
    .open        = can_open,
    .release     = can_release
};

/*****************************************************************************
 * Driver's "main" functions. Init the device and register w/PCI subsystem.
 *****************************************************************************/
static int can_init(void)
{
    int err;

    /* Get a major device number */
    err = alloc_chrdev_region(&dev, 0, MAX_CAN_DEVICES, DRIVER_NAME);
    if (err) {
        printk("Couldn't allocate major/minor numbers. Exiting.\n");
        return err;
    }

    can_dev = NULL;
    can_dev = cdev_alloc();

    if (!can_dev) {
        printk("Couldn't allocate device. Exiting.\n");
        return -ENOMEM;
    }

    cdev_init(can_dev, &file_ops);
    can_dev->owner = THIS_MODULE;
    can_dev->ops = &file_ops;

    err = cdev_add(can_dev, dev, MAX_CAN_DEVICES);
    if (err) {
        printk("Couldn't add device %d. Exiting.\n", err);
        return err;
    }

    err = pci_register_driver(&pci_ops);
    if (err<0) {
        printk("Couldn't register driver %d. Exiting.\n", err);
        return err;
    }

    printk("Controller Area Network Driver\n");

    return 0;
}

/*****************************************************************************
 * Called when the driver exists. Undoes what init() does.
 *****************************************************************************/
static void can_exit(void)
{
    cdev_del(can_dev);
    pci_unregister_driver(&pci_ops);
    unregister_chrdev_region(dev, MAX_CAN_DEVICES);
}


/*****************************************************************************
 * Probe is called when a device is found. If it's a CAN, the device is
 * initialized and enabled.
 *****************************************************************************/
int can_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
    unsigned int pci_bar;
    unsigned int can_num;

    switch (dev->device) {
        case ICP_CAN_PCI_DEVICE_ID_0:
            can_num = 0;
            break;
        case ICP_CAN_PCI_DEVICE_ID_1:
            can_num = 1;
            break;
        default:
            printk("Unrecognized CAN device id 0x%x. Exiting\n",
                dev->device);
            return -ENODEV;
    }

    if (pci_enable_device(dev)) {
        printk("Couldn't enable PCI device v:0x%x d:0x%x. Exiting\n",
          dev->vendor, dev->device);
        return -ENODEV;
    }

    pci_bar = pci_resource_start(dev, 0);

    g_can_os[can_num].pci_remap = ioremap(pci_bar, MM_REG_SIZE);
    g_can_os[can_num].can_num = can_num;
    g_can_os[can_num].irq = dev->irq;
    g_can_os[can_num].can = icp_can_create(g_can_os[can_num].pci_remap);
    g_can_os[can_num].dev = dev;
    g_can_os[can_num].opened = 0;
    g_can_os[can_num].is_suspending = 0;

    icp_can_reset(&(g_can_os[can_num]));

    init_waitqueue_head(&(g_can_os[can_num].read_wait_queue));
    init_waitqueue_head(&(g_can_os[can_num].write_wait_queue));
    init_waitqueue_head(&(g_can_os[can_num].pm_wait_queue));
    spin_lock_init(&(g_can_os[can_num].int_spinlock));
    spin_lock_init(&(g_can_os[can_num].open_spinlock));

    dev->dev.driver_data = (void *) &(g_can_os[can_num]);
    if (!dev->dev.driver_data)
    {
        printk("Couldn't create CAN device %d. Exiting.\n",
            dev->device);
        return -ENODEV;
    }

    printk("Probe completed for CAN %d [0x%x:0x%x] base 0x%08x irq 0x%x.\n",
        can_num,
        dev->vendor,
        dev->device,
        (unsigned int) g_can_os[can_num].pci_remap,
        g_can_os[can_num].irq);

    return 0;
}

/*****************************************************************************
 * Called when device is removed. Undoes what probe does.
 *****************************************************************************/
void can_pci_remove(struct pci_dev *dev)
{
    can_os_t *can_os = dev->dev.driver_data;

    iounmap(can_os->pci_remap);
    icp_can_destroy(can_os->can);
    pci_disable_device(dev);
}

/*****************************************************************************
 * Called when the device is suspended. Saves CAN state.
 *****************************************************************************/
int can_pci_suspend(struct pci_dev *dev, pm_message_t state)
{
    unsigned int i;
    unsigned int int_status;
    can_os_t *can_os = dev->dev.driver_data;
    int err;

	/* Indicate that we are suspending */
	can_os->is_suspending = 1;

	if (can_os->opened) {
		/* Save the Run Mode. */
	    icp_can_get_run_mode(can_os->can, &(can_os->run_mode));
	}

	/* Stop the CAN controller */
	icp_can_set_run_mode(can_os->can, ICP_CAN_STOP);

    /* Clear pending interrupts */
    int_status = icp_can_int_pending(can_os->can);
    if (int_status) {
        icp_can_int_clr(can_os->can, int_status);
	}

	if (can_os->opened) {
		/* Save interrupt configuration and then disable them */
		icp_can_get_int_enables(can_os->can, &(can_os->int_enables));
	    icp_can_set_int_enables(can_os->can, ICP_CAN_DISABLE);

		/* Save Tx buffer enable state */
	    for (i=0; i<NUM_TX_BUFFS; i++) {
	        icp_can_get_tx_enable(can_os->can, i, &(can_os->tx_enable[i]));
	    }

		/* Disable all Transmit buffers */
		icp_can_tx_disable_all(can_os->can);

		/* Save Rx buffer enable state */
	    for (i=0; i<NUM_RX_BUFFS; i++) {
	        icp_can_get_rx_enable(can_os->can, i, &(can_os->rx_enable[i]));
	        icp_can_get_rx_buffer_link(can_os->can, i, &(can_os->rx_link[i]));

			/* Save Rx Filters */
		    can_os->rx_filter[i].num = i;
			icp_can_get_rx_filter(can_os->can, &(can_os->rx_filter[i]));
	    }

		/* Disable all Receive buffers */
		icp_can_rx_disable_all(can_os->can);

	    /* Save Context */
	    icp_can_get_baud(can_os->can, &(can_os->timing));
	    icp_can_get_listen_mode(can_os->can, &(can_os->listen_mode));
	    icp_can_get_arbiter_mode(can_os->can, &(can_os->arbiter_mode));

		/* Free any waiting write threads */
        can_os->write_wait_flag = 1;
        wake_up_interruptible(&(can_os->write_wait_queue));

		/* Destroy the FIFO */
    	can_fifo_destroy(can_os->rx_fifo);
	}

	/* Save PCI CFG space */
    err = pci_save_state(dev);

    return err;
}

/*****************************************************************************
 * Called when the device is resumed. Restores the CAN state.
 *****************************************************************************/
int can_pci_resume(struct pci_dev *dev)
{
    unsigned int i;
    can_os_t *can_os = dev->dev.driver_data;

	/* Restore PCI CFG space */
	pci_restore_state(dev);

	/* Reset the CAN */
    icp_can_reset(can_os);
    icp_can_clear_buffers(can_os->can);
    icp_can_set_int_enables(can_os->can, ICP_CAN_DISABLE);
    icp_can_set_run_mode(can_os->can, ICP_CAN_STOP);

    /* Create the FIFO */
	if (can_os->opened) {
		/* Reset CAN int status */
		can_os->int_status = 0;

		/* Reset write file operation wait flag */
	    can_os->write_wait_flag = 0;

		/* Create the Rx Fifo */
    	can_os->rx_fifo = can_fifo_create(NUM_NODES);

		/* Restore the CAN state */
	    icp_can_set_baud_custom(can_os->can, &(can_os->timing));
   		icp_can_set_listen_mode(can_os->can, can_os->listen_mode);
	    icp_can_set_arbiter_mode(can_os->can, can_os->arbiter_mode);

	    for (i=0; i<NUM_TX_BUFFS; i++) {
	        icp_can_set_tx_enable(can_os->can, i, can_os->tx_enable[i]);
	    }

	    for (i=0; i<NUM_RX_BUFFS; i++) {
			/* Restore buffer enables */
	        icp_can_set_rx_enable(can_os->can, i, can_os->rx_enable[i]);

			/* Restore buffer link */
	        icp_can_set_rx_buffer_link(can_os->can, i, can_os->rx_link[i]);

			/* Init Rx Filters */
            icp_can_rx_init_filter(can_os->can, i);

			/* Restore Rx Filters */
		    can_os->rx_filter[i].num = i;
			icp_can_set_rx_filter(can_os->can, &(can_os->rx_filter[i]));
	    }

		/* Enable CAN Interrupts */
	    icp_can_set_int_custom(can_os->can, can_os->int_enables);

		/* Restore Run Mode */
	    icp_can_set_run_mode(can_os->can, can_os->run_mode);
	}

	can_os->is_suspending = 0;

    return 0;
}

/*****************************************************************************
 * Standard file open. Open the CAN and enable interrupts.
 *****************************************************************************/
int can_open(struct inode *inode, struct file *filp)
{
    int err;
    can_os_t *can_os = (can_os_t *) &(g_can_os[iminor(inode)]);

    /* Check if CAN is already open */
    spin_lock(&(can_os->open_spinlock));
    if (!(can_os->opened)) {
        can_os->opened = 1;
        err = 0;
    } else {
        err = -1;
    }
    spin_unlock(&(can_os->open_spinlock));

    if (err) {
        return err;
    }

    err = icp_can_open(
        can_os->can,
        filp->f_flags & O_RDONLY ? ICP_CAN_LISTEN : ICP_CAN_ACTIVE,
        ICP_CAN_ROUND_ROBIN
        );

    if (err) {
        printk("Couldn't open CAN device %d. Exiting.\n", iminor(inode));
        return err;
    }

    filp->private_data = can_os;

    can_os->write_wait_flag = 0;
    can_os->block_mode = 1;

    /* Creat the FIFO */
    can_os->rx_fifo = can_fifo_create(NUM_NODES);

	can_os->inode = inode;

    err = request_irq(
        can_os->irq,
        can_irq_handler,
        SA_SHIRQ,
        iminor(can_os->inode) ? CAN_PROC_1 : CAN_PROC_0,
        &(g_can_os[iminor(can_os->inode)])
        );

    if (err) {
        printk("IRQ request failed on IRQ %d. Exiting.\n", can_os->irq);
        return err;
    }

    printk("IRQ %d successfully assigned to CAN device %d.\n", can_os->irq,
        iminor(inode));

    return 0;
}

/*****************************************************************************
 * Standard file close. Release resources.
 *****************************************************************************/
int can_release(struct inode *inode, struct file *filp)
{
    int err;
    can_os_t *can_os = (can_os_t *) filp->private_data;

    err = icp_can_release(can_os->can);
    free_irq(can_os->irq, &(g_can_os[iminor(can_os->inode)]));

    can_fifo_destroy(can_os->rx_fifo);

    /* Check if CAN is already open */
    spin_lock(&(can_os->open_spinlock));
    can_os->opened = 0;
    spin_unlock(&(can_os->open_spinlock));

    if (err) {
        printk("Release failed on CAN device %d.\n", iminor(inode));
        return err;
    }

    return 0;
}

/*****************************************************************************
 * Standard file read. Reads a message from the CAN device.

data is getting dequed very fast. the read operation is not keeping up.
sleep/check empty status.
 *****************************************************************************/
ssize_t can_read(struct file *filp, char __user *buf, size_t count,
    loff_t *f_pos)
{
    int err = 0;
    size_t result;
    icp_can_msg_t msg;
    size_t bytes_read = 0;

    can_os_t *can_os = (can_os_t *) filp->private_data;

	if (can_os->is_suspending) {
		return -EAGAIN;
	}

    if ((!(can_os->block_mode)) && (can_fifo_empty(can_os->rx_fifo))) {
        return -EAGAIN;
    }

    if ((can_os->block_mode) && (can_fifo_empty(can_os->rx_fifo))) {
        wait_event_interruptible(can_os->read_wait_queue,
            !can_fifo_empty(can_os->rx_fifo));
    }

    err = can_fifo_get(can_os->rx_fifo, &msg);

    if (err) {
        /*printk("Read from CAN device %d failed. FIFO read failed.\n",
            iminor(filp->f_dentry->d_inode));*/
            return -EIO;
    }

    bytes_read = sizeof(icp_can_msg_t);

    result = copy_to_user(buf, &msg, sizeof(msg));

    if (result) {
        printk("Copy to user failed for CAN %d in write operation.\n",
            iminor(filp->f_dentry->d_inode));
        return -ENOMEM;
    }

    return bytes_read;
}

/*****************************************************************************
 * Standard file write. Writes a message to the CAN device.
 *****************************************************************************/
ssize_t can_write(struct file *filp, const char __user *buf, size_t count,
          loff_t *f_pos)
{
    can_os_t *can_os = (can_os_t *) filp->private_data;

    icp_can_msg_t msg;
    size_t result;
    int err;

	if (can_os->is_suspending) {
		return -EAGAIN;
	}

    if (count!=sizeof(icp_can_msg_t)) {
        printk("Write user buffer size invalid for CAN %d.\n",
            iminor(filp->f_dentry->d_inode));
        return -EINVAL;
    }

    result = copy_from_user(&msg, buf, count);

    if (result) {
        printk("Copy from user failed for CAN %d in write \
            operation.\n", iminor(filp->f_dentry->d_inode));
        return -ENOMEM;
    }

    err = icp_can_msg_tx(can_os->can, &msg);

    if ((!err) && (can_os->block_mode)) {
        wait_event_interruptible(can_os->write_wait_queue,
            can_os->write_wait_flag != 0);
        can_os->write_wait_flag = 0;
    }

    if (err) {
        /*printk("Write from CAN device %d failed.\n",
            iminor(filp->f_dentry->d_inode));*/
        return -EIO;
    }

    return sizeof(icp_can_msg_t);
}

/*****************************************************************************
 * Reset CAN device.
 *****************************************************************************/
int icp_can_reset(can_os_t *can_os)
{
    unsigned short pci_pm_csr;

    pci_read_config_word(can_os->dev, 0xE0, &pci_pm_csr);

    /* Enter D3; power off state */
    pci_pm_csr |=  0x3;
    pci_write_config_word(can_os->dev, 0xE0, pci_pm_csr);

    /* Enter D0; power on state; reset */
    pci_pm_csr &=  ~0x3;
    pci_write_config_word(can_os->dev, 0xE0, pci_pm_csr);

    /* Set interrupts to target the driver */
    pci_write_config_word(can_os->dev, 0xE8, 0x0003);

    printk("Device CAN %d was reset.\n", can_os->can_num);

    return 0;
}

/*****************************************************************************
 * Device IO control function. Used by user apps to configure CAN device.
 *****************************************************************************/
int can_dev_io(struct inode *inode, struct file *filp, unsigned int cmd,
      unsigned long arg)
{
    can_os_t *can_os;
    unsigned int err=0;
    void *in=NULL;
    void *out=NULL;
    size_t in_buff_size=0;
    size_t out_buff_size=0;

    can_os = (can_os_t *) filp->private_data;

    can_ioctl_get_size(cmd, &in_buff_size, &out_buff_size);

    if (in_buff_size) {
        in = kmalloc(in_buff_size, GFP_KERNEL);

        if (in) {
            err = copy_from_user(in, (void *) arg, in_buff_size);
            if (err) {
                printk("Copy from user failed for for CAN %d in can_dev_io \
                        operation.\n", iminor(filp->f_dentry->d_inode));
                return -ENOMEM;
            }
        } else {
            return -EAGAIN;
        }
    }

    if (out_buff_size) {
        out = kmalloc(out_buff_size, GFP_KERNEL);

        if (!out) {
            kfree(in);
            return -EAGAIN;
        }
    }

    err = can_ioctl(can_os, cmd, in, out);

    if (!err) {
        err = copy_to_user((void *) arg, out, out_buff_size);
        if (err) {
            kfree(in);
            kfree(out);
                printk("Copy to user failed for for CAN %d in can_dev_io \
                            operation.\n", iminor(filp->f_dentry->d_inode));
                return -ENOMEM;
            }
    }

    kfree(in);
    kfree(out);

    return err;
}


/*****************************************************************************
 * Interrupt handler.
 *****************************************************************************/
irqreturn_t can_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
    can_os_t *can_os = (can_os_t *) dev_id;
    unsigned int int_status;
    unsigned int err;
    unsigned int i;
    icp_can_msg_t msg;
    unsigned int buffer_status;

    spin_lock_irqsave(&(can_os->int_spinlock), can_os->int_flags);

    /* Get the interrupt status */
    int_status = icp_can_int_pending(can_os->can);

    if (int_status) {

        /* Clear the interrupt status */
        icp_can_int_clr(can_os->can, int_status);

        if (int_status & MSK_IS_RXMSG) {

	        /* Read buffer status and dequeue all arrived messages */
	        buffer_status = icp_can_get_buffer_status(can_os->can) & \
	            MSK_BS_RXALL;
	        i=0;

	        while (buffer_status) {
	            if (buffer_status & 0x1) {
	                icp_can_rx_dequeue(can_os->can, &msg, i);

					if (!can_os->is_suspending) {
		                can_fifo_put(can_os->rx_fifo, &msg);
					}
	            }
	            buffer_status >>= 1;
	            i++;
	        }

	        if ( (i) && (!can_os->is_suspending) ) {
	            wake_up_interruptible(&(can_os->read_wait_queue));
            }

            int_status &= ~MSK_IS_RXMSG;
        }

        /* Only schedule the tasklet if there are still pending interrupts. */
        if (int_status) {
            /* Save the interrupt status */
            can_os->int_status |= int_status;

            /* Schedule the Tasklet */
            if (can_os->can_num==0) {
                tasklet_schedule(&can_tasklet0);
            } else {
                tasklet_schedule(&can_tasklet1);
            }
        }

        err = IRQ_HANDLED;
    } else {
        err = IRQ_NONE;
    }

    spin_unlock_irqrestore(&(can_os->int_spinlock), can_os->int_flags);

    return err;

}

void can_tasklet(unsigned long arg)
{
    can_os_t *can_os = (can_os_t *) arg;

    if (can_os->int_status & MSK_IS_TXMSG) {
        can_os->write_wait_flag = 1;
        wake_up_interruptible(&(can_os->write_wait_queue));
        can_os->int_status &= ~MSK_IS_TXMSG;
    }

    if (can_os->int_status & MSK_IS_RXMSGLOSS) {
        icp_can_log_message(MSK_IS_RXMSGLOSS);
        can_os->int_status &= ~MSK_IS_RXMSGLOSS;
    }

    if (can_os->int_status & MSK_IS_BUSOFF) {
        icp_can_log_message(MSK_IS_BUSOFF);
        can_os->int_status &= ~MSK_IS_BUSOFF;
    }

    if (can_os->int_status & MSK_IS_CRCERR) {
        icp_can_log_message(MSK_IS_CRCERR);
        can_os->int_status &= ~MSK_IS_CRCERR;
    }

    if (can_os->int_status & MSK_IS_FORMERR) {
        icp_can_log_message(MSK_IS_FORMERR);
        can_os->int_status &= ~MSK_IS_FORMERR;
    }

    if (can_os->int_status & MSK_IS_ACKERR) {
        icp_can_log_message(MSK_IS_ACKERR);
        can_os->int_status &= ~MSK_IS_ACKERR;
    }

    if (can_os->int_status & MSK_IS_STUFFERR) {
        icp_can_log_message(MSK_IS_STUFFERR);
        can_os->int_status &= ~MSK_IS_STUFFERR;
    }

    if (can_os->int_status & MSK_IS_BITERR) {
        icp_can_log_message(MSK_IS_BITERR);
        can_os->int_status &= ~MSK_IS_BITERR;
    }

    if (can_os->int_status & MSK_IS_OVRLOAD) {
        icp_can_log_message(MSK_IS_OVRLOAD);
        can_os->int_status &= ~MSK_IS_OVRLOAD;
    }

    if (can_os->int_status & MSK_IS_ARLOSS) {
        icp_can_log_message(MSK_IS_ARLOSS);
        can_os->int_status &= ~MSK_IS_ARLOSS;
    }
}

module_init(can_init);
module_exit(can_exit);
