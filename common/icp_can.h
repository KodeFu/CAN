/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/

/**
 *****************************************************************************
 * @file icp_can.h
 *
 * @defgroup icp_CanAcc Public APIs for the CAN I/O Unit.
 * 
 * @description
 *
 *          The Controller Area Network (CAN) I/O Access Library is the
 *          software that provides an interface to the Controller Area
 *          Netwrok I/O Unit to higher level software such as
 *          device drivers.  The CAN I/O Unit APIs provide all of the 
 *          the CAN configuration and data processing functionality.
 *****************************************************************************/

/*****************************************************************************/

#ifndef __ICP_CAN_H__
#define __ICP_CAN_H__

/*****************************************************************************/

#include "icp_can_types.h"
#include "can_port.h"
#include "icp_can_regs.h"

/*
 ******************************************************************************
 * Defines
 ******************************************************************************
 */

/* Parasoft fixes */
#define MAX_BITRATE 32768
/**
 * @ingroup icp_CanAcc
 * PCI Vendor ID.
 * @description
 * Used to define the PCI vendor ID for the CAN device.
 */
#define ICP_CAN_PCI_VENDOR_ID        0x8086

/**
 * @ingroup icp_CanAcc
 * PCI Device ID.
 * @description
 * Used to define the PCI device ID for the CAN device 0.
 */
#define ICP_CAN_PCI_DEVICE_ID_0      0x5039

/**
 * @ingroup icp_CanAcc
 * PCI Device ID.
 * @description
 * Used to define the PCI device ID for the CAN device 1.
 */
#define ICP_CAN_PCI_DEVICE_ID_1      0x503a

/**
 * @ingroup icp_CanAcc
 * Generic error output string.
 * @description
 * Used to define a generic user specified error output string.
 */
#define ICP_CAN_ERR_GENERIC          "CAN: %s.\n"

/**
 * @ingroup icp_CanAcc
 * Allocation error output string.
 * @description
 * Used to define an allocation error output string.
 */
#define ICP_CAN_ERR_ALLOC            "CAN: Allocate failed on %s.\n"

/**
 * @ingroup icp_CanAcc
 * Free error output string.
 * @description
 * Used to define a free error output string.
 */
#define ICP_CAN_ERR_FREE            "CAN: Free failed on %s.\n"

/**
 * @ingroup icp_CanAcc
 * Parameter error output string.
 * @description
 * Used to define a parameter error output string.
 */
#define ICP_CAN_ERR_PARAM            "CAN: Parameter %s invalid.\n"

/**
 * @ingroup icp_CanAcc
 * Operation error output string.
 * @description
 * Used to define an operation error output string.
 */
#define ICP_CAN_ERR_OPERATION         "CAN: Operation %s failed.\n"

/**
 * @ingroup icp_CanAcc
 * Device error output string.
 * @description
 * Used to define a device error output string.
 */
#define ICP_CAN_ERR_DEVICE             "CAN: Detected %s condition.\n"

/**
 * @ingroup icp_CanAcc
 * Queue full output string.
 * @description
 * Used to define queue full output string.
 */
#define ICP_CAN_ERR_QUEUE_FULL        "CAN: Queue full.\n"

/**
 * @ingroup icp_CanAcc
 * Queue empty output string.
 * @description
 * Used to define queue empty output string.
 */
#define ICP_CAN_ERR_QUEUE_EMPTY      "CAN: Queue empty.\n"

/*
 *****************************************************************************
 * Function Declarations
 *****************************************************************************
 */

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Creates a CAN device object.
 * @description
 *           This funciton initializes and configures a CAN device for use.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         io_base [IN] - IO registers of the CAN device.
 * @retval
 *         Handle to CAN device.
 * @retval
 *         0 The function did not execute successfully.
 *
 *****************************************************************************/
icp_can_handle_t
icp_can_create(
    unsigned char *io_base);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Destroys a CAN device object.
 * @description
 *         Deallocates all resources held by the CAN device object.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @retval
 *         None.
 *
 *****************************************************************************/
void 
icp_can_destroy(
    icp_can_handle_t handle);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Set run/stop mode of the CAN device
 *
 * @description
 *         This function starts or stops the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         mode [IN] - Run/stop mode.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_run_mode(
    icp_can_handle_t handle, 
    icp_can_run_mode_t    mode);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get run mode.
 *
 * @description
 *         This function gets the run/stop mode of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         mode [OUT] - Run/stop mode.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_get_run_mode(
    icp_can_handle_t handle,
    icp_can_run_mode_t *mode);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Set listen mode.
 *
 * @description
 *         This function sets the listen/active mode of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         mode [IN] - Listen/active mode.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_listen_mode(
    icp_can_handle_t handle, 
    icp_can_listen_mode_t mode);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get listen mode.
 *
 * @description
 *         This function gets the listen/active mode of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         mode [OUT] - Listen/active mode.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_get_listen_mode(
    icp_can_handle_t handle,
    icp_can_listen_mode_t *mode);
    
/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Set arbiter mode.
 *
 * @description
 *         This function set the arbiter mode of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         mode [IN] - Arbiter mode.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_arbiter_mode(
    icp_can_handle_t handle, 
    icp_can_arbiter_t mode);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get arbiter mode.
 *
 * @description
 *         This function gets the arbiter mode of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         mode [OUT] - Run/stop mode.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_get_arbiter_mode(
    icp_can_handle_t handle,
    icp_can_arbiter_t *mode);
    
    
/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Set the restart mode.
 *
 * @description
 *         This function sets the restart mode of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         mode [IN] - Restart mode.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_restart_mode(
    icp_can_handle_t handle, 
    icp_can_auto_restart_t mode);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get the restart mode.
 *
 * @description
 *         This function sets the restart mode of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         mode [OUT] - Restart mode.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_get_restart_mode(
    icp_can_handle_t handle,
    icp_can_auto_restart_t *mode);


/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Set the baud rate (simple).
 *
 * @description
 *         This function sets the baud rate of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         baud [IN] - Baud rate.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_baud_simple(
    icp_can_handle_t handle, 
    icp_can_baud_t baud);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Set the baud rate (custom).
 *
 * @description
 *         This function sets a custom baud rate for the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         timing [IN] - CAN timing info.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_baud_custom(
    icp_can_handle_t handle, 
    icp_can_timing_t *timing);


/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get the baud rate.
 *
 * @description
 *         This function gets the baud rate for the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         timing [OUT] - CAN timing info.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_get_baud(
    icp_can_handle_t handle, 
    icp_can_timing_t *timing);
        
/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Set the receive filter.
 *
 * @description
 *         This function sets the receive filter for a receive buffer of the
 *         CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         filter [IN] - Receive filter.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_rx_filter(
    icp_can_handle_t handle, 
    icp_can_rx_filter_t    *filter);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get the receive filter.
 *
 * @description
 *         This function sets the receive filter for a receive buffer of the
 *         CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         filter [OUT] - Receive filter.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_get_rx_filter(
    icp_can_handle_t handle, 
    icp_can_rx_filter_t    *filter);
    
    
/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Message transmit.
 *
 * @description
 *         This function transmits a CAN message.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         msg [IN] - Message.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_msg_tx(
    icp_can_handle_t handle, 
    icp_can_msg_t *msg);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Open CAN device.
 *
 * @description
 *         This function opens the CAN device for read/write.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         listen [IN] - Listen/active mode.
 * @param
 *         arbiter [IN] - Aribter mode.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_open(
    icp_can_handle_t handle, 
    icp_can_listen_mode_t listen,
    icp_can_arbiter_t arbiter);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Interrupt pending.
 *
 * @description
 *         This function returns whether or not interrupts are pending for the
 *         CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @retval
 *         >1 Interrupts are pending.
 * @retval
 *         0 No interrupts pending.
 *
 *****************************************************************************/
unsigned int 
icp_can_int_pending(
    icp_can_handle_t handle);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Clear interrupt(s).
 *
 * @description
 *         This function clears interrupt(s) from the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         mask [IN] - Interrupt mask (interrupts to clear).
 * @retval
 *         None.
 *
 *****************************************************************************/
void 
icp_can_int_clr(
    icp_can_handle_t handle, 
    unsigned int mask);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Clear hardware buffers.
 *
 * @description
 *         This function clears all transmit and receive buffer of the CAN
 *         device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_clear_buffers(
    icp_can_handle_t handle);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get a message.
 *
 * @description
 *         This function gets a pending message from the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         msg [OUT] - Message.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_rx_dequeue(
    icp_can_handle_t handle,
    icp_can_msg_t *msg,
    unsigned int buff_num);


/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Interrupt enable (custom).
 *
 * @description
 *         This function sets which interrupts to enable.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         interrupts [IN] - Interrupt mask.
 *           31:19 Reserved
 *           12     Message received
 *           11     Message sent
 *           10     Set when new message arrives but MsgAv is set
 *           9     CAN reached the bus off state
 *           8     CRC error occured while receiving or transmitting data
 *           7     Form error occured while receiving or transmitting data
 *           6     Acknowledge error occured while transmitting data
 *           5     Stuff error occured while transmitting data
 *           4     Bit error occured while receiving or transmitting data
 *           3     Overload condition has occured
 *           2     Arbitration was lost while sending a message
 *           1     N/A
 *           0     0=All interrupts disabled; 1=All interrupt sources available
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_int_custom(
    icp_can_handle_t handle, 
    unsigned int interrupts);
    
    
/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Set interrupt enables (simple).
 *
 * @description
 *         This function sets the interrupt enales of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         interrupt [IN] - Interrupt type.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_int_enables(
    icp_can_handle_t handle, 
    icp_can_interrupt_t    interrupt);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get interrupt enables.
 *
 * @description
 *         This function gets interrupt enables of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         enables [OUT] - Interrupt mask.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_get_int_enables(
    icp_can_handle_t handle, 
    unsigned int *enables);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Log messages.
 *
 * @description
 *         This function outputs CAN error/info messages.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         status [IN] - Status.
 * @retval
 *         None.
 *
 *****************************************************************************/
void 
icp_can_log_message(
    unsigned int status);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Release device.
 *
 * @description
 *         This function releases (closes) the CAN device. Call to close an
 *         opened CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_release(
    icp_can_handle_t handle);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Set buffer linking.
 *
 * @description
 *         This function sets receive buffer linking of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         buffer_num [IN] - Receive buffer.
 * @param
 *         set [IN] - Link=1, No Link=0.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_rx_buffer_link(
    icp_can_handle_t handle,
    unsigned int buffer_num,
    unsigned int set);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get buffer linking.
 *
 * @description
 *         This function gets receive buffer linking of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         buffer_num [IN] - Receive buffer.
 * @param
 *         link [OUT] - Link status.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/    
int 
icp_can_get_rx_buffer_link(
    icp_can_handle_t handle,
    unsigned int buffer_num,
    unsigned int *link);
    
/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Buffer status.
 *
 * @description
 *         This function gets the buffer status of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @retval
 *         >0 Transmit and/or receive buffers available.
 * @retval
 *         0 No buffers availabe.
 *
 *****************************************************************************/
unsigned int 
icp_can_get_buffer_status(
    icp_can_handle_t handle);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Initialize a receive filter.
 *
 * @description
 *         This function initialize a receive filter of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         buff_num [IN] - Receive buffer.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_rx_init_filter(
    icp_can_handle_t handle,
    unsigned int buff_num);
    
/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Enable all receive buffers.
 *
 * @description
 *         This function enables all receive filters of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_rx_enable_all(
    icp_can_handle_t handle);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Disable all receive filters.
 *
 * @description
 *         This function disables all receive filters of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_rx_disable_all(
    icp_can_handle_t handle);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Enable all transmit buffers.
 *
 * @description
 *         This function enables all transmit buffers of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int
icp_can_tx_enable_all(
    icp_can_handle_t handle);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Disable all transmit buffers.
 *
 * @description
 *         This function enables all transmit buffers of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_tx_disable_all(
    icp_can_handle_t handle);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Enable/disable a receive buffer.
 *
 * @description
 *         This function enables or disables a particular receive buffer 
 *         of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         buff_num [IN] - Receive buffer.
 * @param
 *         set [IN] - 1=Enable, 0=Disable.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_rx_enable(
    icp_can_handle_t handle, 
    unsigned int buff_num,
    unsigned int set);
    
/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get enable state of a receive buffer.
 *
 * @description
 *         This function gets the enable state of a receive buffer.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         buff_num [IN] - Receive buffer.
 * @param
 *         enable [OUT] - 1=Enable, 0=Disable.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_get_rx_enable(
    icp_can_handle_t handle, 
    unsigned int buff_num,
    unsigned int *enable);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Enable/disalbe a tranmit buffer.
 *
 * @description
 *         This function enables/disables a transmit buffer.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         buff_num [IN] - Receive buffer.
 * @param
 *         set [IN] - 1=Enable, 0=Disable.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_set_tx_enable(
    icp_can_handle_t handle, 
    unsigned int buff_num,
    unsigned int set);

/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get enable/disable status of a transmit buffer.
 *
 * @description
 *         This function gets the enable/disable status of a transmit buffer.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         buff_num [IN] - Receive buffer.
 * @param
 *         enable [OUT] - 1=Enable, 0=Disable.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_get_tx_enable(
    icp_can_handle_t handle, 
    unsigned int buff_num,
    unsigned int *enable);


/**
 *****************************************************************************
 * @ingroup icp_CanAcc
 *         Get error statistics.
 *
 * @description
 *         This function gets the error statics of the CAN device.
 *
 * @reentrant
 *         No
 * @context
 *         Calling function thread.
 * @param
 *         handle [IN] - Handle of CAN device.
 * @param
 *         error [OUT] - Error stats.
 * @retval
 *         0 The function executed successfully.
 * @retval
 *         <0 The function did not execute successfully.
 *
 *****************************************************************************/
int 
icp_can_get_error_stats(
        icp_can_handle_t handle,
        icp_can_error_t *error);
    
#endif /* __ICP_CAN_H__ */
