/*****************************************************************************
 * %LICENSE_PROPRIETARY%
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




/*****************************************************************************
 * Driver "main" functions.
 *****************************************************************************/
NTSTATUS
DriverEntry(
    IN PDRIVER_OBJECT DriverObject,
    IN PUNICODE_STRING RegistryPath
    )
{
    NTSTATUS status = STATUS_SUCCESS;
    WDF_DRIVER_CONFIG config;

    WDF_DRIVER_CONFIG_INIT(
        &config,
        CANEvtDeviceAdd
        );

    status = WdfDriverCreate(
        DriverObject,
        RegistryPath,
        WDF_NO_OBJECT_ATTRIBUTES,
        &config,
        WDF_NO_HANDLE
        );

    if (!NT_SUCCESS(status)) {
        DbgPrint("DriverEntry: WdfDriverCreate failed.\n");
    }

    DbgPrint("Controller Area Network Driver\n");

    return status;
}

/*****************************************************************************
 * Called when a new CAN is discovered.
 *****************************************************************************/
NTSTATUS
CANEvtDeviceAdd(
    IN WDFDRIVER Driver,
    IN PWDFDEVICE_INIT DeviceInit
    )
{
    NTSTATUS status = STATUS_SUCCESS;
    WDFDEVICE hDevice;
    WDF_IO_QUEUE_CONFIG queueConfig;
    WDF_OBJECT_ATTRIBUTES objAttributes;
    WDF_PNPPOWER_EVENT_CALLBACKS pnpPowerCallbacks;
    WDF_INTERRUPT_CONFIG interruptConfig;
    WDF_FILEOBJECT_CONFIG fileConfig;
    DECLARE_CONST_UNICODE_STRING(SimpleDeviceNameCan0, L"\\Device\\Can0") ;
    DECLARE_CONST_UNICODE_STRING(SimpleDeviceNameCan1, L"\\Device\\Can1") ;
    DECLARE_CONST_UNICODE_STRING(DosDeviceNameCan0, L"\\DosDevices\\Can0");
    DECLARE_CONST_UNICODE_STRING(DosDeviceNameCan1, L"\\DosDevices\\Can1");
    PDEVICE_OBJECT pdo;
    ULONG can_num;
    ULONG vendor_id;
    ULONG device_id;

    can_os_t *can_os;

    DbgPrint("CANEvtDeviceAdd: Started.\n");
    
    /************************************************************
     * Set up plug-n-play event callbacks
     ************************************************************/
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpPowerCallbacks);

    pnpPowerCallbacks.EvtDevicePrepareHardware = CANEvtDevicePrepareHardware;
    pnpPowerCallbacks.EvtDeviceReleaseHardware = CANEvtDeviceReleaseHardware;
    
    pnpPowerCallbacks.EvtDeviceD0Entry = CANEvtDeviceD0Entry;
    pnpPowerCallbacks.EvtDeviceD0Exit = CANEvtDeviceD0Exit;

    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpPowerCallbacks);

    /************************************************************
     * Add file Create / Close callbacks.
     ************************************************************/
    WDF_FILEOBJECT_CONFIG_INIT(
        &fileConfig,
        CANEvtDeviceFileCreate,
        CANEvtFileClose,
        WDF_NO_EVENT_CALLBACK
        );

    WdfDeviceInitSetFileObjectConfig(
        DeviceInit,
        &fileConfig,
        WDF_NO_OBJECT_ATTRIBUTES);
    
    /************************************************************
     * Set up the Device Context
     ************************************************************/
    WDF_OBJECT_ATTRIBUTES_INIT(&objAttributes);

    WDF_OBJECT_ATTRIBUTES_SET_CONTEXT_TYPE(
        &objAttributes, 
        can_os_t
        );

    objAttributes.EvtCleanupCallback = CANEvtDeviceContextCleanup;

    /************************************************************
     * SychronizationScope - each handler is called serially. No
     * locking necessary.
     * ExecutionLevel - IO callbacks will never run at dispatch 
     * level.
     ************************************************************/
    objAttributes.SynchronizationScope = WdfSynchronizationScopeDevice;
    objAttributes.ExecutionLevel = WdfExecutionLevelPassive;

    /************************************************************
     * Create the device and context
     ************************************************************/
    status = WdfDeviceCreate(
        &DeviceInit,
        &objAttributes,
        &hDevice
        );

    if (!NT_SUCCESS(status)) {
        DbgPrint("CANEvtDeviceAdd: WdfDeviceCreate failed.\n");
                return status;
    }
    
    /************************************************************
     * Get the OS context for use later.
     ************************************************************/
    can_os = WdfObjectGet_can_os_t(hDevice);

    /************************************************************
     * Set a symbolic link from the CAN device name to
     * the link specified (so we can access it from user space).
     ************************************************************/
    pdo = WdfDeviceWdmGetDeviceObject(hDevice);
    PCIReadConfigWord(pdo, 0x00, &vendor_id);
    PCIReadConfigWord(pdo, 0x02, &device_id);

    vendor_id &= 0x0000ffff;
    device_id &= 0x0000ffff;

    DbgPrint("CANEvtDeviceAdd: Found vendor id 0x%x, device id 0x%x\n", 
        vendor_id, device_id);

    switch (device_id) {
        case ICP_CAN_PCI_DEVICE_ID_0:
            can_os->can_num = 0;
            break;
        case ICP_CAN_PCI_DEVICE_ID_1:
            can_os->can_num = 1;
            break;
        default:
            DbgPrint("Unrecognized CAN device id. Exiting.\n");
            status = STATUS_UNSUCCESSFUL;
            return status;
    }

    status = WdfDeviceCreateSymbolicLink(
        hDevice,
        (can_os->can_num) ? &DosDeviceNameCan1 : &DosDeviceNameCan0
        );

    if (!NT_SUCCESS(status)) {
        DbgPrint("CANEvtDeviceAdd: WdfDeviceCreateSymbolicLink failed.\n");
        return status;
    }


    /************************************************************
     * Create the Device Interface
     ************************************************************/
    status = WdfDeviceCreateDeviceInterface(
        hDevice,
        (can_os->can_num) ? &GUID_DEVINTERFACE_CAN1 : &GUID_DEVINTERFACE_CAN0,
        NULL
        );

    if (!NT_SUCCESS(status)) {
        DbgPrint("CANEvtDeviceAdd: WdfDeviceCreateDeviceInterface failed.\n");
        return status;
    }

    /************************************************************
     * Create the R/W/IOCTL Queue
     ************************************************************/
    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(
        &queueConfig,
        WdfIoQueueDispatchSequential
        );

    queueConfig.PowerManaged = TRUE;

    queueConfig.EvtIoRead = CANEvtIoRead;
    queueConfig.EvtIoWrite = CANEvtIoWrite;
    queueConfig.EvtIoDeviceControl = CANEvtIoDeviceControl;

    status = WdfIoQueueCreate(
        hDevice,
        &queueConfig,
        WDF_NO_OBJECT_ATTRIBUTES,
        NULL
        );

    if (!NT_SUCCESS(status)) {
        DbgPrint("CANEvtDeviceAdd: CreateQueue failed.\n");
        return status;
    }

    /************************************************************
     * Create the Interrupt
     ************************************************************/
    WDF_INTERRUPT_CONFIG_INIT(
        &interruptConfig, 
        CANIsr, 
        CANDpc
        );

    interruptConfig.EvtInterruptEnable = CANEvtInterruptEnable;
    interruptConfig.EvtInterruptDisable = CANEvtInterruptDisable;

    status = WdfInterruptCreate(
        hDevice,
        &interruptConfig,
        WDF_NO_OBJECT_ATTRIBUTES,
        &(can_os->WdfInterrupt)
        );

    if (!NT_SUCCESS(status)) {
        DbgPrint("CANEvtDeviceAdd: InterruptCreate failed.\n");
        return status;
    }

	/* Init the is_suspending flag. */
	can_os->is_suspending = 0;

    DbgPrint("CANEvtDeviceAdd: Completed.\n");

    return status;
}

/*****************************************************************************
 * Called when CAN is no longer available.
 *****************************************************************************/
VOID
CANEvtDeviceContextCleanup(
    IN WDFOBJECT  Object
    )
{
    /************************************************************
     * Clean up allocated context. N/A, placeholder for now.
     ************************************************************/
}

/*****************************************************************************
 * Standard file open.
 *****************************************************************************/
VOID
CANEvtDeviceFileCreate (
    IN WDFDEVICE Device,
    IN WDFREQUEST Request,
    IN WDFFILEOBJECT FileObject
    )
{
    NTSTATUS status = STATUS_SUCCESS;
    can_os_t *can_os;
    int err;
    can_os = WdfObjectGet_can_os_t(Device);

    /* Make sure only one application has the CAN device open. */
    KeAcquireSpinLock(&(can_os->spin_open), &(can_os->spin_open_irql));
    if (!(can_os->ref_cnt)) {
        can_os->ref_cnt = 1;
        err = icp_can_open(can_os->can, ICP_CAN_ACTIVE, ICP_CAN_ROUND_ROBIN);

        if (err) {
            status = STATUS_UNSUCCESSFUL;
        } else {
            can_os->rx_fifo = can_fifo_create(NUM_NODES);
            can_os->block_mode = 1;
        }
    } else {
        status = STATUS_UNSUCCESSFUL;
    }

    KeReleaseSpinLock(&(can_os->spin_open), can_os->spin_open_irql);
    WdfRequestComplete(Request, status);
}

VOID
CANEvtFileClose(
    IN WDFFILEOBJECT FileObject
    )
{
    can_os_t *can_os;
    int err;

    can_os = WdfObjectGet_can_os_t(WdfFileObjectGetDevice(FileObject));

    can_os->ref_cnt = 0;

    icp_can_release(can_os->can);

    can_fifo_destroy(can_os->rx_fifo); 

}

/*****************************************************************************
 * Standard file read.
 *****************************************************************************/
VOID
CANEvtIoRead(
    WDFQUEUE Queue,
    WDFREQUEST Request,
    size_t Length
    )
{
    int err;
    icp_can_msg_t msg;
    can_os_t *can_os;
    WDFMEMORY memory;
    NTSTATUS status = STATUS_UNSUCCESSFUL;
    ULONG_PTR bytes_read = 0;
    PVOID out;
	NTSTATUS alert_status;
	LARGE_INTEGER timeout;
    
    can_os = WdfObjectGet_can_os_t(WdfIoQueueGetDevice(Queue));

	if (can_os->is_suspending) {
		goto Done;
	}

    if ((!(can_os->block_mode)) && can_fifo_empty(can_os->rx_fifo)) {
        status = STATUS_SUCCESS;
        bytes_read = 0;
        goto Done;
    }

    status = WdfRequestRetrieveOutputBuffer(Request, sizeof(icp_can_msg_t), \
        &out, NULL);

    if (NT_SUCCESS(status)) {
        #define LIMIT -50000000 /* timeout.QuadPart = LIMIT */               
		if ((can_os->block_mode) && (can_fifo_empty(can_os->rx_fifo))) {
			timeout.QuadPart = LIMIT;

			alert_status = KeWaitForSingleObject(&(can_os->rx_event), Executive, \
				KernelMode, FALSE, &timeout);

			KeResetEvent(&(can_os->rx_event));

			if (!NT_SUCCESS(alert_status)) {
				/* Receive timed-out or error, try again */
				status = STATUS_CANCELLED;
				goto Done;
			}
        }

		err = can_fifo_get(can_os->rx_fifo, out);

        if (err) {
            status = STATUS_UNSUCCESSFUL;
            bytes_read = 0;
            goto Done;
        }
        
        status = STATUS_SUCCESS;
        bytes_read = sizeof(icp_can_msg_t);
    }

Done:
    WdfRequestCompleteWithInformation(Request, status, bytes_read);
}

/*****************************************************************************
 * Standard file write.
 *****************************************************************************/
VOID
CANEvtIoWrite(
    WDFQUEUE Queue,
    WDFREQUEST Request,
    size_t Length
    )
{
    can_os_t *can_os;
    icp_can_msg_t *msg = NULL;
    int err;
    NTSTATUS status = STATUS_UNSUCCESSFUL;
    ULONG_PTR bytes_written = 0;
    unsigned int req_len;

    can_os = WdfObjectGet_can_os_t(WdfIoQueueGetDevice(Queue));

	if (can_os->is_suspending) {
		goto Error;
	}

    status = WdfRequestRetrieveInputBuffer(Request, sizeof(icp_can_msg_t), 
        (PVOID) &msg, &req_len);

    if (!NT_SUCCESS(status)) {
        goto Error;
    }

    err = icp_can_msg_tx(can_os->can, msg);

	if ((!err) && (can_os->block_mode)) {
        KeWaitForSingleObject(&(can_os->tx_event), \
            Executive, KernelMode, TRUE, 0);
        KeResetEvent(&(can_os->tx_event));
    }

    if (!err) {
        bytes_written = sizeof(icp_can_msg_t);
        status = STATUS_SUCCESS;
    }

Error:

    WdfRequestCompleteWithInformation(Request, status, bytes_written);
}

/*****************************************************************************
 * Device IO control.
 *****************************************************************************/
VOID
CANEvtIoDeviceControl(
    IN WDFQUEUE Queue,
    IN WDFREQUEST Request,
    IN size_t OutputBufferLength,
    IN size_t InputBufferLength,
    IN ULONG IoControlCode
    )
{
    can_os_t *can_os;
    NTSTATUS status = STATUS_SUCCESS;
    WDFMEMORY Memory;
    unsigned int err = 0;
    void *in;
    void *out;
    size_t in_buff_size = 0;
    size_t out_buff_size = 0;
    size_t out_size_returned = 0;
        
    /*DbgPrint("CANEvtIoDeviceControl: Entered... %x\n", 
            (IoControlCode>>2) & 0x00000fff);*/

    can_os = WdfObjectGet_can_os_t(WdfIoQueueGetDevice(Queue));

    can_ioctl_get_size(IoControlCode, &in_buff_size, &out_buff_size);

    if ((InputBufferLength<in_buff_size) || (OutputBufferLength<out_buff_size))
    {
        goto Exit;
    }

    if (InputBufferLength) {
        status = WdfRequestRetrieveInputBuffer(Request, in_buff_size, &in, \
            NULL);
        if (!NT_SUCCESS(status)) {
            goto Exit;
        }
    }

    if (OutputBufferLength) {
        status = WdfRequestRetrieveOutputBuffer(Request, out_buff_size, \
            &out, NULL);
        if (!NT_SUCCESS(status)) {
            goto Exit;
        }
        out_size_returned = out_buff_size;
    }

    err = can_ioctl(can_os, IoControlCode, in, out);

    if (err) {
        status = STATUS_UNSUCCESSFUL;
        out_size_returned = 0;
    }

Exit:
    WdfRequestCompleteWithInformation(Request, status, out_size_returned);
}

/*****************************************************************************
 * Interrupt handler.
 *****************************************************************************/
BOOLEAN
CANIsr(
    IN WDFINTERRUPT  Interrupt,
    IN ULONG  MessageID
    )
{
    can_os_t *can_os;
    BOOLEAN err = FALSE;
    unsigned int int_status;
    unsigned int i;
    icp_can_msg_t msg; 
    unsigned int buffer_status;
    
    can_os = WdfObjectGet_can_os_t(WdfInterruptGetDevice(Interrupt));

    int_status = icp_can_int_pending(can_os->can);

    if (int_status) {
        /* Save the status */
        can_os->int_status = int_status;

        /* Clear the status */
        icp_can_int_clr(can_os->can, int_status);

        /* Save the interrupt status */
        can_os->int_status |= int_status;

        /* Schedule the DPC */
        WdfInterruptQueueDpcForIsr(Interrupt);

        /* Serviced the interrupt */
        err = TRUE;
    }

    return err;
}

/*****************************************************************************
 * Interrupt handler DPC.
 *****************************************************************************/
VOID
  CANDpc(
    IN WDFINTERRUPT  Interrupt,
    IN WDFOBJECT  AssociatedObject
    )
{
    can_os_t *can_os;
    unsigned int status;
    int err;
    unsigned int buffer_status;
    unsigned int i;
    icp_can_msg_t msg; 


    can_os = WdfObjectGet_can_os_t(WdfInterruptGetDevice(Interrupt));

    if (can_os->int_status & MSK_IS_RXMSG) {
        /* Read buffer status */
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

        if ((i) && (!can_os->is_suspending)) {
            KeSetEvent(&(can_os->rx_event), IO_NO_INCREMENT, FALSE);
        }

        can_os->int_status &= ~MSK_IS_RXMSG;
    }

    if (can_os->int_status & MSK_IS_TXMSG) {
        KeSetEvent(&(can_os->tx_event), IO_NO_INCREMENT, FALSE);
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
        can_os->int_status &= ~MSK_IS_STUFFERR;
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

/*****************************************************************************
 * Get the devices resources.
 *****************************************************************************/
NTSTATUS
  CANEvtDevicePrepareHardware(
    IN WDFDEVICE  Device,
    IN WDFCMRESLIST  ResourcesRaw,
    IN WDFCMRESLIST  ResourcesTranslated
    )
{

    NTSTATUS status = STATUS_SUCCESS;
    ULONG i;
    PCM_PARTIAL_RESOURCE_DESCRIPTOR descriptor;
    BOOLEAN got_interrupt = FALSE;
    BOOLEAN got_memory = FALSE;
    can_os_t *can_os = WdfObjectGet_can_os_t(Device);

    for (i=0; i<WdfCmResourceListGetCount(ResourcesTranslated); i++) {

        descriptor = WdfCmResourceListGetDescriptor(ResourcesTranslated, i);

        switch (descriptor->Type) {

        case CmResourceTypeMemory:
            DbgPrint("Memory start %8X%8.8lX length %X\n",
                descriptor->u.Port.Start.HighPart, 
                descriptor->u.Port.Start.LowPart,
                descriptor->u.Port.Length
                );

            can_os->pci_remap = MmMapIoSpace(
                        descriptor->u.Memory.Start, 
                        descriptor->u.Memory.Length,
                        MmNonCached
                        );
            can_os->pci_mmap_size = descriptor->u.Memory.Length;

            got_memory = TRUE;
            break;

        case CmResourceTypeInterrupt:
            DbgPrint("Interrupt  level %X, vector %X, affinity %X\n",
                descriptor->u.Interrupt.Level, 
                descriptor->u.Interrupt.Vector,
                descriptor->u.Interrupt.Affinity);

            can_os->irq = (KIRQL) descriptor->u.Interrupt.Level;
            /* vector = resource->u.Interrupt.Vector;
               affinity = resource->u.Interrupt.Affinity;    */

            got_interrupt = TRUE;
            break;

        default:
            break;
        }

    }

    if (got_memory && got_interrupt) {
        can_os->can = icp_can_create(can_os->pci_remap);
        can_os->hDevice = Device;
        can_os->ref_cnt = 0;

        icp_can_reset(can_os);
        icp_can_clear_buffers(can_os->can);

        KeInitializeEvent(&(can_os->rx_event), NotificationEvent, FALSE);
        KeInitializeEvent(&(can_os->tx_event), NotificationEvent, FALSE);
        KeInitializeSpinLock(&(can_os->spin_open));
    } else {
        status = STATUS_DEVICE_CONFIGURATION_ERROR;
    }

    return status;
}
 
/*****************************************************************************
 * Device is no longer available.
 *****************************************************************************/
NTSTATUS
  CANEvtDeviceReleaseHardware(
    IN WDFDEVICE  Device,
    IN WDFCMRESLIST  ResourcesTranslated
    )
{
    can_os_t *can_os = WdfObjectGet_can_os_t(Device);

    icp_can_destroy(can_os->can);

    MmUnmapIoSpace(can_os->pci_remap, can_os->pci_mmap_size);

    return STATUS_SUCCESS;
}

/*****************************************************************************
 * Called when power is about to be shut-off to the device.
 *****************************************************************************/
NTSTATUS
  CANEvtDeviceD0Exit(
    IN WDFDEVICE  Device,
    IN WDF_POWER_DEVICE_STATE  TargetState
    )
{
    unsigned int buffer_status;
    KEVENT tmp_event;
    can_os_t *can_os;
    LARGE_INTEGER timeout;
    unsigned int i;
	unsigned int int_status;
DbgPrint("CANEvtDeviceD0Exit ... entered.\n");
    can_os = WdfObjectGet_can_os_t(Device);

	/* Indicate that we are suspending */
	can_os->is_suspending = 1;

	/* Set Read event so, suspend can continue */
	KeSetEvent(&(can_os->rx_event), IO_NO_INCREMENT, FALSE);

	if (can_os->ref_cnt) {
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

	if (can_os->ref_cnt) {
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
		KeResetEvent(&(can_os->tx_event));
	}
    
    return STATUS_SUCCESS;
}

/*****************************************************************************
 * Called when power is restored.
 *****************************************************************************/
NTSTATUS
  CANEvtDeviceD0Entry(
    IN WDFDEVICE  Device,
    IN WDF_POWER_DEVICE_STATE  PreviousState
    )
{
	unsigned int i;
    can_os_t *can_os = WdfObjectGet_can_os_t(Device);

	/* Reset the CAN */
    icp_can_reset(can_os);
    icp_can_clear_buffers(can_os->can);
    icp_can_set_int_enables(can_os->can, ICP_CAN_DISABLE);
    icp_can_set_run_mode(can_os->can, ICP_CAN_STOP);

    /* Create the FIFO */
	if (can_os->ref_cnt) {

		/* Reset CAN int status */
		can_os->int_status = 0;

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

    return STATUS_SUCCESS;
}


/*****************************************************************************
 * Enables interrupts.
 *****************************************************************************/
NTSTATUS
  CANEvtInterruptEnable(
    IN WDFINTERRUPT  Interrupt,
    IN WDFDEVICE  AssociatedDevice
    )
{
    can_os_t *can_os = WdfObjectGet_can_os_t(AssociatedDevice);

    icp_can_set_int_enables(can_os->can, ICP_CAN_ENABLE);
    return STATUS_SUCCESS;
}

/*****************************************************************************
 * Disables interrupts.
 *****************************************************************************/
NTSTATUS
  CANEvtInterruptDisable(
    IN WDFINTERRUPT  Interrupt,
    IN WDFDEVICE  AssociatedDevice
    )
{
    can_os_t *can_os = WdfObjectGet_can_os_t(AssociatedDevice);

    icp_can_set_int_enables(can_os->can, ICP_CAN_DISABLE);
    return STATUS_SUCCESS;
}

int icp_can_reset(can_os_t *can_os)
{
    PDEVICE_OBJECT pdo;
    unsigned short pci_pm_csr;
    unsigned short driver_ints;

    pdo = WdfDeviceWdmGetDeviceObject(can_os->hDevice);

    PCIReadConfigWord(pdo, 0xE0, &pci_pm_csr);

    /* Enter D3; power off state */
    pci_pm_csr |=  0x3;
    PCIWriteConfigWord(pdo, 0xE0, &pci_pm_csr);
    
    /* Enter D0; power on state; reset */
    pci_pm_csr &=  ~0x3;
    PCIWriteConfigWord(pdo, 0xE0, &pci_pm_csr);

    /* Set interrupts to target the driver */
    driver_ints = 0x1;
    PCIWriteConfigWord(pdo, 0xE8, &driver_ints);

    DbgPrint("CAN reset\n");
    
    return 0;

}

/*****************************************************************************
 * Direct R/W from config space.
 *****************************************************************************/
INT
PCIReadConfigWord(
    IN PDEVICE_OBJECT DeviceObject,
    IN ULONG          Offset,
    IN PVOID          Value
    )
{
    PDEVICE_OBJECT TargetObject;
    PIRP pIrp;
    IO_STATUS_BLOCK IoStatusBlock;
    PIO_STACK_LOCATION IrpStack;
    KEVENT ConfigReadWordEvent;
    INT error = 0;

    TargetObject = IoGetAttachedDeviceReference(DeviceObject);
    KeInitializeEvent(&ConfigReadWordEvent, NotificationEvent, FALSE);
    
    pIrp = IoBuildSynchronousFsdRequest(IRP_MJ_PNP, TargetObject, NULL,
        0, NULL, &ConfigReadWordEvent, &IoStatusBlock );

    if (pIrp) {
        /* Create the config space read IRP */
        IrpStack = IoGetNextIrpStackLocation(pIrp);
        IrpStack->MinorFunction = IRP_MN_READ_CONFIG;
        IrpStack->Parameters.ReadWriteConfig.WhichSpace = \
            PCI_WHICHSPACE_CONFIG;
        IrpStack->Parameters.ReadWriteConfig.Offset = Offset;
        IrpStack->Parameters.ReadWriteConfig.Length = 0x2;
        IrpStack->Parameters.ReadWriteConfig.Buffer = Value;
        pIrp->IoStatus.Status = STATUS_NOT_SUPPORTED ;
     
        /* Send the IRP */
        if (IoCallDriver(TargetObject, pIrp)==STATUS_PENDING) {
            KeWaitForSingleObject(&ConfigReadWordEvent, Executive, \
                KernelMode, FALSE, NULL);
        }
    } else {
        error = -1;
    }

    ObDereferenceObject(TargetObject);
    return error;
} 

INT
PCIWriteConfigWord(
    IN PDEVICE_OBJECT DeviceObject,
    IN ULONG          Offset,
    IN PVOID          Value
    )
{
    PDEVICE_OBJECT TargetObject;
    PIRP pIrp;
    IO_STATUS_BLOCK IoStatusBlock;
    PIO_STACK_LOCATION IrpStack;
    KEVENT ConfigWriteWordEvent;
    INT error = 0;

    TargetObject = IoGetAttachedDeviceReference(DeviceObject);
    KeInitializeEvent(&ConfigWriteWordEvent, NotificationEvent, FALSE);
    
    pIrp = IoBuildSynchronousFsdRequest(IRP_MJ_PNP, TargetObject, NULL,
        0, NULL, &ConfigWriteWordEvent, &IoStatusBlock );

    if (pIrp) {
        /* Create the config space write IRP */
        IrpStack = IoGetNextIrpStackLocation(pIrp);
        IrpStack->MinorFunction = IRP_MN_WRITE_CONFIG;
        IrpStack->Parameters.ReadWriteConfig.WhichSpace = PCI_WHICHSPACE_CONFIG;
        IrpStack->Parameters.ReadWriteConfig.Offset = Offset;
        IrpStack->Parameters.ReadWriteConfig.Length = 0x2;
        IrpStack->Parameters.ReadWriteConfig.Buffer = Value;
        pIrp->IoStatus.Status = STATUS_NOT_SUPPORTED ;
     
        /* Send the IRP */
        if (IoCallDriver(TargetObject, pIrp)==STATUS_PENDING) {
            KeWaitForSingleObject(&ConfigWriteWordEvent, Executive, \
                KernelMode, FALSE, NULL);
        }
    } else {
        error = -1;
    }

    ObDereferenceObject(TargetObject);
    return error;
} 
