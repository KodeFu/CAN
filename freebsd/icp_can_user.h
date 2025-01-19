/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/

#ifndef __CAN_USER_H__
#define __CAN_USER_H__

/*****************************************************************************
 * Device IO control codes.
 *****************************************************************************/
#define MAGIC				0xdd

#define ICP_CAN_IO_RESET		_IO(MAGIC, 0)

#define ICP_CAN_IO_RUN			_IO(MAGIC, 1)
#define ICP_CAN_IO_STOP			_IO(MAGIC, 2)
#define ICP_CAN_IO_RUN_GET		_IOR(MAGIC, 3, icp_can_run_mode_t)

#define ICP_CAN_IO_FILTER		_IOW(MAGIC, 4, icp_can_rx_filter_t)
#define ICP_CAN_IO_FILTER_GET		_IOWR(MAGIC, 5, icp_can_rx_filter_t)

#define ICP_CAN_IO_CUSTOM		_IOW(MAGIC, 6, icp_can_timing_t)
#define ICP_CAN_IO_SIMPLE		_IOW(MAGIC, 7, icp_can_baud_t)
#define ICP_CAN_IO_TIMING_GET		_IOR(MAGIC, 8, icp_can_timing_t)

#define ICP_CAN_IO_BLOCK		_IO(MAGIC, 9)
#define ICP_CAN_IO_NON_BLOCK		_IO(MAGIC, 10)
#define ICP_CAN_IO_BLOCK_GET		_IOR(MAGIC, 11, unsigned int)

#define ICP_CAN_IO_LISTEN		_IO(MAGIC, 12)
#define ICP_CAN_IO_ACTIVE		_IO(MAGIC, 13)
#define ICP_CAN_IO_LISTEN_GET		_IOR(MAGIC, 14, icp_can_listen_mode_t)

#define ICP_CAN_IO_ARBITER_ROUND_ROBIN	_IO(MAGIC, 15)
#define ICP_CAN_IO_ARBITER_FIXED_PRIORITY _IO(MAGIC, 16)
#define ICP_CAN_IO_ARBITER_GET		_IOR(MAGIC, 17, icp_can_arbiter_t)

#define ICP_CAN_IO_ERROR_STATS_GET	_IOR(MAGIC, 18, icp_can_error_t)

#define ICP_CAN_IO_RESTART_MODE_AUTO	_IO(MAGIC, 19)
#define ICP_CAN_IO_RESTART_MODE_MANUAL	_IO(MAGIC, 20)
#define ICP_CAN_IO_RESTART_MODE_GET	_IOR(MAGIC, 21, icp_can_auto_restart_t)

#define ICP_CAN_IO_BUFFER_LINK_SET	_IOW(MAGIC, 22, unsigned int)
#define ICP_CAN_IO_BUFFER_LINK_CLEAR	_IOW(MAGIC, 23, unsigned int)
#define ICP_CAN_IO_BUFFER_LINK_GET	_IOWR(MAGIC, 24, unsigned int)

#define ICP_CAN_IO_RX_ENABLE_SET	_IOW(MAGIC, 25, unsigned int)
#define ICP_CAN_IO_RX_ENABLE_CLEAR	_IOW(MAGIC, 26, unsigned int)
#define ICP_CAN_IO_RX_ENABLE_GET	_IOWR(MAGIC, 27, unsigned int)

#define ICP_CAN_IO_TX_ENABLE_SET	_IOW(MAGIC, 28, unsigned int)
#define ICP_CAN_IO_TX_ENABLE_CLEAR	_IOW(MAGIC, 29, unsigned int)
#define ICP_CAN_IO_TX_ENABLE_GET	_IOWR(MAGIC, 30, unsigned int)
#endif /* __CAN_USER_H__ */

