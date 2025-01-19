/*******************************************************************************
 * %LICENSE_PROPRIETARY%
 * <COPYRIGHT_TAG>
 *******************************************************************************/

#include <ntddk.h>
#include <wdf.h>
#include <initguid.h>
#include <errno.h>

#include "icp_can.h"
#include "can_fifo.h"
#include "icp_can_user.h"

/* Parasoft fixes */
#define NUM_NODES 2000
#define QUAD_PART -10

/*****************************************************************************
 * CAN OS context structure.
 *****************************************************************************/
typedef struct can_os {
     icp_can_handle_t can;                /* Handle to CAN device         */
    unsigned int can_num;                 /* CAN instance                 */
    void *pci_remap;                      /* Memory mapped registers      */
    unsigned int pci_mmap_size;           /* Memory mapped register size  */
    unsigned char irq;                    /* Interrupt                    */
    WDFDEVICE hDevice;                    /* Handle to WDF device         */
    int block_mode;                       /* Blocking / non-blocking      */
    WDFINTERRUPT WdfInterrupt;            /* WDF: Interrupt handler       */
    unsigned int int_status;              /* Interrupt status             */
    icp_can_handle_t rx_fifo;             /* Handle to Rx FIFO            */
    KEVENT rx_event;                      /* Rx message event             */
    KEVENT tx_event;                      /* Tx message event             */
    unsigned int ref_cnt;                 /* Reference count              */
    KSPIN_LOCK spin_open;                 /* Open lock                    */
    KIRQL spin_open_irql;                 /* Open irql                    */
    icp_can_timing_t timing;              /* CAN timing                   */
    icp_can_run_mode_t run_mode;          /* CAN run mode                 */
    icp_can_listen_mode_t listen_mode;    /* CAN listen mode              */
    icp_can_arbiter_t arbiter_mode;       /* CAN: arbiter mode            */
    unsigned int tx_enable[NUM_TX_BUFFS]; /* CAN: Tx buffer state         */
    unsigned int rx_enable[NUM_RX_BUFFS]; /* CAN: Rx buffer state         */
    unsigned int rx_link[NUM_RX_BUFFS];   /* CAN: Rx link set             */
    unsigned int int_enables;             /* CAN: ints enabled            */
	unsigned int is_suspending;			  /* Is suspending state		  */
	icp_can_rx_filter_t rx_filter[NUM_RX_BUFFS]; /* CAN: Rx filters       */
} can_os_t;

WDF_DECLARE_CONTEXT_TYPE(can_os_t)

// {1F48355B-8417-4c7a-83BB-87C670342ABC}
DEFINE_GUID(GUID_DEVINTERFACE_CAN0, 
    0x1f48355b, 0x8417, 0x4c7a, 0x83, 0xbb, 0x87, 0xc6, 0x70, 0x34, 0x2a, 0xbc);

// {1F48355B-8417-4c7a-83BB-87C670342ABD}
DEFINE_GUID(GUID_DEVINTERFACE_CAN1, 
    0x1f48355b, 0x8417, 0x4c7a, 0x83, 0xbb, 0x87, 0xc6, 0x70, 0x34, 0x2a, 0xbd);

int icp_can_reset(can_os_t *can_os);

/*****************************************************************************
 * WDF/OS functions.
 *****************************************************************************/
NTSTATUS
DriverEntry(IN PDRIVER_OBJECT DriverObject,
    IN PUNICODE_STRING RegistryPath
    );

NTSTATUS
CANEvtDeviceAdd(
    IN WDFDRIVER Driver,
    IN PWDFDEVICE_INIT DeviceInit
    );

VOID
CANEvtDeviceFileCreate (
    IN WDFDEVICE Device,
    IN WDFREQUEST Request,
    IN WDFFILEOBJECT FileObject
    );

VOID
CANEvtFileClose(
    IN WDFFILEOBJECT FileObject
    );

VOID
CANEvtIoRead(
    WDFQUEUE Queue,
    WDFREQUEST Request,
    size_t Length
    );

VOID
CANEvtIoWrite(
    WDFQUEUE Queue,
    WDFREQUEST Request,
    size_t Length
    );

VOID
CANEvtIoDeviceControl(
    IN WDFQUEUE Queue,
    IN WDFREQUEST Request,
    IN size_t OutputBufferLength,
    IN size_t InputBufferLength,
    IN ULONG IoControlCode
    );

BOOLEAN
CANIsr(
    IN WDFINTERRUPT  Interrupt,
    IN ULONG  MessageID
    );

VOID
CANDpc(
    IN WDFINTERRUPT  Interrupt,
    IN WDFOBJECT  AssociatedObject
    );

NTSTATUS
CANEvtDevicePrepareHardware(
    IN WDFDEVICE  Device,
    IN WDFCMRESLIST  ResourcesRaw,
    IN WDFCMRESLIST  ResourcesTranslated
    );

NTSTATUS
CANEvtDeviceReleaseHardware(
    IN WDFDEVICE  Device,
    IN WDFCMRESLIST  ResourcesTranslated
    );

NTSTATUS
CANEvtDeviceD0Entry(
    IN WDFDEVICE  Device,
    IN WDF_POWER_DEVICE_STATE  PreviousState
    );

NTSTATUS
CANEvtDeviceD0Exit(
    IN WDFDEVICE  Device,
    IN WDF_POWER_DEVICE_STATE  TargetState
    );

NTSTATUS
CANEvtInterruptEnable(
    IN WDFINTERRUPT  Interrupt,
    IN WDFDEVICE  AssociatedDevice
    );

NTSTATUS
CANEvtInterruptDisable(
    IN WDFINTERRUPT  Interrupt,
    IN WDFDEVICE  AssociatedDevice
    );

VOID
CANEvtDeviceContextCleanup(
    IN WDFOBJECT  Object
    );

INT
PCIReadConfigWord(
    IN PDEVICE_OBJECT DeviceObject,
    IN ULONG          Offset,
    IN PVOID          Value
    );

INT
PCIWriteConfigWord(
    IN PDEVICE_OBJECT DeviceObject,
    IN ULONG          Offset,
    IN PVOID          Value
    );

/*****************************************************************************
 * CAN user interface functions.
 *****************************************************************************/
int can_ioctl(
    can_os_t *can_os,
    unsigned int ctl_code,
    void *in,
    void *out
    );

void can_ioctl_get_size(
    unsigned int    ctl_code,
    size_t            *in_size,
    size_t            *out_size
    );
