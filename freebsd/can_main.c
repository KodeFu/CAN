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

static struct cdevsw file_ops = {
    .d_version = D_VERSION,
    .d_flags = D_NEEDGIANT,
    .d_open = can_open,
    .d_close = can_close,
    .d_read = can_read,
    .d_write = can_write,
    .d_ioctl = can_dev_io,
    .d_name = DRIVER_NAME
};

static device_method_t pci_ops[] = {
    DEVMETHOD(device_probe, can_pci_probe),
    DEVMETHOD(device_attach, can_pci_attach),
    DEVMETHOD(device_detach, can_pci_detach),
    {0, 0}
};

MODULE_VERSION(can, VERSION);

int icp_can_reset(can_os_t *can_os)
{
    unsigned short pci_pm_csr;

    pci_pm_csr = pci_read_config(can_os->dev, 0xE0, WORD);

    /* Enter D3; power off state */
    pci_pm_csr |= 0x3;
    pci_write_config(can_os->dev, 0xE0, pci_pm_csr, WORD);

    /* Enter D0; power on state; reset */
    pci_pm_csr &= ~0x3;
    pci_write_config(can_os->dev, 0xE0, pci_pm_csr, WORD);

    /* Set interrupts to target the driver */
    pci_write_config(can_os->dev, 0xE8, 0x1, WORD);

    device_printf(can_os->dev, "Device CAN reset.\n");

    return 0;
}

/*****************************************************************************
 * Probe gets called by the OS whenever a device is discoverd. We answer to
 * CAN devices only.
 *****************************************************************************/
static int can_pci_probe(device_t dev)
{
    int err = ENXIO;
    unsigned int vendor_id;
    unsigned int device_id;

    device_printf(dev, "PCI Probe called.\n");

    vendor_id = pci_get_vendor(dev);
    device_id = pci_get_device(dev);

    if ( (vendor_id==ICP_CAN_PCI_VENDOR_ID) && 
         ((device_id==ICP_CAN_PCI_DEVICE_ID_0) ||
         (device_id==ICP_CAN_PCI_DEVICE_ID_1)) ) {

        device_set_desc(dev, "CAN Controller");
        device_printf(dev, "Found CAN vevndor id:%x device id:%x\n", 
            vendor_id, device_id);
        err = 0;
    }

    return err;
}

/*****************************************************************************
 * Called for each CAN device (after probe). Get the devices resources
 * and "create" the CAN device.
 *****************************************************************************/
static int can_pci_attach(device_t dev)
{
    int err = 0;
    can_os_t *can_os;
    unsigned int device_id;

    can_os = (can_os_t *) device_get_softc(dev);

    device_id = pci_get_device(dev);

    can_os->cdev = make_dev(&file_ops, device_get_unit(dev), 
        UID_ROOT, GID_WHEEL, PERMS, "can%u", device_get_unit(dev));

    if (can_os->cdev) {
        /* Save device and CAN number */
        can_os->dev = dev;
        can_os->can_num = device_get_unit(dev);

        /* Save the context structure for file ops */
        can_os->cdev->si_drv1 = can_os;
        
        /* Get the memory mapped registers resource. */
        can_os->mem_rid = PCIR_BAR(0);
        can_os->mem_res = bus_alloc_resource_any(dev, 
            SYS_RES_MEMORY, &(can_os->mem_rid), RF_ACTIVE);

        if (can_os->mem_res==NULL) {
            device_printf(dev, "Couldn't get device memory\n");
            return ENXIO;
        }

        can_os->mem_tag = rman_get_bustag(can_os->mem_res);
        can_os->mem_bus = rman_get_bushandle(can_os->mem_res);

        /* Get the interrupt resource */
        can_os->irq_rid = 0x0;
        can_os->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
            &(can_os->irq_rid), RF_SHAREABLE | RF_ACTIVE);

        if (can_os->irq_res==NULL) {
            device_printf(dev, "Couldn't get interrupt\n");
            return ENXIO;
        }

        /* Connect the interrupt handler */
        err = bus_setup_intr(can_os->dev, can_os->irq_res,
            INTR_TYPE_MISC,
            can_irq_handler, can_os, &(can_os->irq_tag));

        if (err) {
            device_printf(can_os->dev, "Couldn't install interrupt handler\n");
            return ENXIO;
        }

        /* Finally, initialize the CAN device. */
        can_os->can = icp_can_create(rman_get_virtual(can_os->mem_res));

        icp_can_reset(can_os);

        mtx_init(&(can_os->int_spinlock), "can_int", NULL, MTX_SPIN);

        can_os->opened = 0;

        device_printf(dev, "Attach completed for CAN %d base 0x%08x.\n",
                    can_os->can_num,
                    (unsigned int) rman_get_virtual(can_os->mem_res));
    }

    device_printf(dev, "Controller Area Network Driver\n");

    return err;
}

/*****************************************************************************
 * Called when device is no longer available. Undoes what pci_attach does.
 *****************************************************************************/
static int can_pci_detach(device_t dev)
{
    int err = 0;
    can_os_t *can_os;

    can_os = device_get_softc(dev);

    /* Release the device nodes */
    destroy_dev(can_os->cdev);

    /* Release the device memory */
    if (can_os->mem_res) {
        bus_release_resource(dev, SYS_RES_MEMORY, 
            can_os->mem_rid, can_os->mem_res);
    }

    /* Release the interrupt */
    if (can_os->irq_tag) {
        bus_teardown_intr(dev, can_os->irq_res, 
            can_os->irq_tag);
    }

    /* Release the interrupt resource */
    if (can_os->irq_res) {
        bus_release_resource(dev, SYS_RES_IRQ, 
            can_os->irq_rid, can_os->irq_res);
    }

    /* Destroy the interrupt spinlock */
    mtx_destroy(&(can_os->int_spinlock));

    /* Destroy the CAN hardware resources */
    icp_can_destroy(can_os->can);    

    return err;
}

/*****************************************************************************
 * Standard file open.
*****************************************************************************/
static int can_open(struct cdev *dev, int oflags, int devtype, struct thread *p)
{
    int err = 0;
    can_os_t *can_os;

    can_os = dev->si_drv1;

    if (can_os->opened) {
        return -1;
    } else {
        can_os->opened = 1;
    }

    err = icp_can_open(
        can_os->can, 
        oflags & O_RDONLY ? ICP_CAN_LISTEN : ICP_CAN_ACTIVE,
        ICP_CAN_ROUND_ROBIN 
        );
        
    if (err) {
        device_printf(can_os->dev, "Couldn't open CAN device. Exiting.");
        return err;
    }
    
    can_os->write_wait_flag = 0;
    can_os->read_wait_flag = 0;
    can_os->block_mode = 1;

    can_os->rx_fifo = can_fifo_create(NUM_NODES);

    return err;
}

/*****************************************************************************
 * Standard file close.
 *****************************************************************************/
static int can_close(struct cdev *dev, int oflags, int devtype, 
    struct thread *p)
{
    int err = 0;
    can_os_t *can_os;

    can_os = dev->si_drv1;

    err = icp_can_release(can_os->can);

    can_fifo_destroy(can_os->rx_fifo);

    can_os->opened = 0;

    return err;
}

/*****************************************************************************
 * Standard file read. Reads a CAN message. Blocks if set to block.
 *****************************************************************************/
static int can_read(struct cdev *dev, struct uio *uio, int ioflag)
{
    int err = 0;
    unsigned int result;
    icp_can_msg_t msg;
    can_os_t *can_os;

    can_os = dev->si_drv1;

    if ((!(can_os->block_mode)) && (can_fifo_empty(can_os->rx_fifo))) {
        return -EAGAIN;
    }

    if ((can_os->block_mode) && (can_fifo_empty(can_os->rx_fifo))) {
        while (can_fifo_empty(can_os->rx_fifo)) {
            tsleep(&(can_os->read_wait_id), PWAIT, "can_rd", 0xa);
        }
        can_os->read_wait_flag = 0;
    }

    err = can_fifo_get(can_os->rx_fifo, &msg);
    if (err) {
        device_printf(can_os->dev, "Read from CAN device failed.\n");
        return -EIO;
    }

    result = uiomove(&msg, sizeof(icp_can_msg_t), uio);
    if (result) {
        device_printf(can_os->dev, 
            "Copy to user failed for for CAN in write operation.\n");
        return -ENOMEM;
    }

    return 0;

}

/*****************************************************************************
 * Standard file write. Writes a CAN message. Blocks if set to block.
 *****************************************************************************/
static int can_write(struct cdev *dev, struct uio *uio, int ioflag)
{
    int err = 0;
    can_os_t *can_os;
    icp_can_msg_t msg;
    unsigned int result;

    can_os = dev->si_drv1;
    
    if (uio->uio_iov->iov_len!=sizeof(icp_can_msg_t)) {
        device_printf(can_os->dev, 
            "Write user buffer size invalid for CAN.\n");
        return -EINVAL;
    }
    
    result = uiomove(&msg, sizeof(icp_can_msg_t), uio);    

    if (result) {
        device_printf(can_os->dev, 
            "Copy from user failed for for CAN in write operation.\n");
        return -ENOMEM;
    }

    err = icp_can_msg_tx(can_os->can, &msg);

    if ((!err) && (can_os->block_mode)) {
        if (can_os->write_wait_flag != 0) {
            tsleep(&(can_os->write_wait_id), PWAIT, "can_wr", 0);
            can_os->write_wait_flag = 0;
        }
    }

    if (err) {
        device_printf(can_os->dev, "Write from CAN device failed.\n");
        return -EIO;
    }
    
    return 0;
}


/*****************************************************************************
 * Device IO Control. Control CAN device 
 *****************************************************************************/
int can_dev_io(struct cdev *dev, unsigned long cmd, caddr_t data, 
    int flag, struct thread *td)
{
    int err = 0;
    can_os_t *can_os;
    size_t in_buff_size = 0;
    size_t out_buff_size = 0;

    can_os = dev->si_drv1;

    can_ioctl_get_size(cmd, &in_buff_size, &out_buff_size);
    
    err = can_ioctl(can_os, cmd, data, data);

    return err;
}

/*****************************************************************************
 * Interrupt handler.
 *****************************************************************************/
void can_irq_handler(void *arg)
{
    can_os_t *can_os = (can_os_t *) arg;
    unsigned int int_status;    
    unsigned int i;
    icp_can_msg_t msg;
    unsigned int buffer_status;

    mtx_lock_spin(&(can_os->int_spinlock));

    /* Get interrupt status */
    int_status = icp_can_int_pending(can_os->can);

    if (int_status) {
        /* Clear the interrupt status */
        icp_can_int_clr(can_os->can, int_status);
    
        if (int_status & MSK_IS_RXMSG) {
            /* Read buffer status */
            buffer_status = icp_can_get_buffer_status(can_os->can) \
                & MSK_BS_RXALL;
            i=0;

            while (buffer_status) {
                if (buffer_status & 0x1) {
                    icp_can_rx_dequeue(can_os->can, &msg, i);
                    /*can_os->read_wait_flag = 1;*/
                    can_fifo_put(can_os->rx_fifo, &msg);
                    /*wakeup(&(can_os->read_wait_id));*/
                }
                buffer_status >>= 1;
                i++;
            }

            if (i) {
                can_os->read_wait_flag = 1;
                wakeup(&(can_os->read_wait_id));
            }

            int_status &= ~MSK_IS_RXMSG;
        }
        
        if (int_status & MSK_IS_TXMSG) {
            can_os->write_wait_flag = 1;
            wakeup(&(can_os->write_wait_id));
        }
        
        if (int_status & MSK_IS_RXMSGLOSS) {
            icp_can_log_message(MSK_IS_RXMSGLOSS);
        }
        
        if (int_status & MSK_IS_BUSOFF) {
            icp_can_log_message(MSK_IS_BUSOFF);
        }
        
        if (int_status & MSK_IS_CRCERR) {
            icp_can_log_message(MSK_IS_CRCERR);
        }
        
        if (int_status & MSK_IS_FORMERR) {
            icp_can_log_message(MSK_IS_FORMERR);
        }
        
        if (int_status & MSK_IS_ACKERR) {
            icp_can_log_message(MSK_IS_ACKERR);
        }
        
        if (int_status & MSK_IS_STUFFERR) {
            icp_can_log_message(MSK_IS_STUFFERR);
        }
        
        if (int_status & MSK_IS_BITERR) {
            icp_can_log_message(MSK_IS_BITERR);
        }
        
        if (int_status & MSK_IS_OVRLOAD) {
            icp_can_log_message(MSK_IS_OVRLOAD);
        }
    
        if (int_status & MSK_IS_ARLOSS) {
            icp_can_log_message(MSK_IS_ARLOSS);
        }
    }

    mtx_unlock_spin(&(can_os->int_spinlock));
}

static devclass_t can_devclass;

DEFINE_CLASS_0(can, can_pci_driver, pci_ops, sizeof(struct can_os));
DRIVER_MODULE(can, pci, can_pci_driver, can_devclass, 0, 0);

