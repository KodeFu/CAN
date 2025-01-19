/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/

/*****************************************************************************
 * Device IO control.
 *****************************************************************************/
int can_ioctl(
	can_os_t *can_os,
	unsigned int ctl_code,
	void *in,
	void *out
	)
{
	unsigned int err = 0;

	switch (ctl_code) {

	case ICP_CAN_IO_RESET:
		icp_can_set_run_mode(can_os->can, ICP_CAN_STOP);
		err = icp_can_reset(can_os);
		icp_can_set_run_mode(can_os->can, ICP_CAN_RUN);
		break;
	case ICP_CAN_IO_RUN:
		err = icp_can_set_run_mode(can_os->can, ICP_CAN_RUN);
		break;
	case ICP_CAN_IO_RUN_GET:
		err = icp_can_get_run_mode(can_os->can, (icp_can_run_mode_t *) out);
		break;			
	case ICP_CAN_IO_STOP:
		err = icp_can_set_run_mode(can_os->can, ICP_CAN_STOP);
		break;
	case ICP_CAN_IO_SIMPLE:
		icp_can_set_run_mode(can_os->can, ICP_CAN_STOP);
		err = icp_can_set_baud_simple(can_os->can, *((icp_can_baud_t *) in));
		icp_can_set_run_mode(can_os->can, ICP_CAN_RUN);
		break;
	case ICP_CAN_IO_CUSTOM:
		icp_can_set_run_mode(can_os->can, ICP_CAN_STOP);
		err = icp_can_set_baud_custom(can_os->can, (icp_can_timing_t *) in);
		icp_can_set_run_mode(can_os->can, ICP_CAN_RUN);
		break;
	case ICP_CAN_IO_TIMING_GET:		
		err = icp_can_get_baud(can_os->can, (icp_can_timing_t *) out);
		break;
	case ICP_CAN_IO_FILTER:	
		icp_can_set_rx_filter(can_os->can, (icp_can_rx_filter_t *) in);
		break;
	case ICP_CAN_IO_FILTER_GET:
		err = icp_can_get_rx_filter(can_os->can, (icp_can_rx_filter_t *) out);
		break;
	case ICP_CAN_IO_BLOCK:
		can_os->block_mode = 1;
		break;
	case ICP_CAN_IO_NON_BLOCK:
		can_os->block_mode = 0;
		break;
	case ICP_CAN_IO_BLOCK_GET:
		*((unsigned int *) out) = can_os->block_mode;
		break;
	case ICP_CAN_IO_LISTEN:
		err = icp_can_set_listen_mode(can_os->can, ICP_CAN_LISTEN);
		break;
	case ICP_CAN_IO_ACTIVE:
		err = icp_can_set_listen_mode(can_os->can, ICP_CAN_ACTIVE);
		break;
	case ICP_CAN_IO_LISTEN_GET:
		err = icp_can_get_listen_mode(can_os->can, (icp_can_listen_mode_t *) out);
		break;	
	case ICP_CAN_IO_ARBITER_ROUND_ROBIN:
		err = icp_can_set_arbiter_mode(can_os->can, ICP_CAN_ROUND_ROBIN);
		break;
	case ICP_CAN_IO_ARBITER_FIXED_PRIORITY:
		err = icp_can_set_arbiter_mode(can_os->can, ICP_CAN_FIXED_PRIORITY);
		break;
	case ICP_CAN_IO_ARBITER_GET:
		err = icp_can_get_arbiter_mode(can_os->can, (icp_can_arbiter_t *) out);
		break;
	case ICP_CAN_IO_ERROR_STATS_GET:
		err = icp_can_get_error_stats(can_os->can, (icp_can_error_t *) out);
		break;
	case ICP_CAN_IO_RESTART_MODE_AUTO:
		err = icp_can_set_restart_mode(can_os->can, ICP_CAN_AUTO);
		break;
	case ICP_CAN_IO_RESTART_MODE_MANUAL:
		err = icp_can_set_restart_mode(can_os->can, ICP_CAN_MANUAL);
		break;
	case ICP_CAN_IO_RESTART_MODE_GET:
		err = icp_can_get_restart_mode(can_os->can, (icp_can_auto_restart_t *) out);
		break;
	case ICP_CAN_IO_BUFFER_LINK_SET:
		err = icp_can_set_rx_buffer_link(can_os->can, *((int *) in), 1);
		break;
	case ICP_CAN_IO_BUFFER_LINK_CLEAR:
		err = icp_can_set_rx_buffer_link(can_os->can, *((int *) in), 0);
		break;
	case ICP_CAN_IO_BUFFER_LINK_GET:
		err = icp_can_get_rx_buffer_link(can_os->can, *((int *) in), (unsigned int *) out);
		break;
	case ICP_CAN_IO_RX_ENABLE_SET:
		err = icp_can_set_rx_enable(can_os->can, *((int *) in), 1);
		break;
	case ICP_CAN_IO_RX_ENABLE_CLEAR:
		err = icp_can_set_rx_enable(can_os->can, *((int *) in), 0);
		break;
	case ICP_CAN_IO_RX_ENABLE_GET:
		err = icp_can_get_rx_enable(can_os->can, *((int *) in), (unsigned int *) out);
		break;
	case ICP_CAN_IO_TX_ENABLE_SET:
		err = icp_can_set_tx_enable(can_os->can, *((int *) in), 1);
		break;
	case ICP_CAN_IO_TX_ENABLE_CLEAR:
		err = icp_can_set_tx_enable(can_os->can, *((int *) in), 0);
		break;
	case ICP_CAN_IO_TX_ENABLE_GET:
		err = icp_can_get_tx_enable(can_os->can, *((int *) in), (unsigned int *) out);
		break;
	default:
		CAN_PRINT_DEBUG("Unrecognizined IOCTL, skipping 0x%x. \n", ctl_code);
		break;
	}

	return err;
}

void can_ioctl_get_size(
	unsigned int ctl_code,
	size_t *in_size,
	size_t *out_size
	)
{
	*in_size = 0;
	*out_size = 0;

	switch (ctl_code) {

	case ICP_CAN_IO_SIMPLE:
		*in_size = sizeof(icp_can_baud_t);
		break;
	case ICP_CAN_IO_CUSTOM:
		*in_size = sizeof(icp_can_timing_t);
		break;
	case ICP_CAN_IO_FILTER:	
		*in_size = sizeof(icp_can_rx_filter_t);
		break;
	case ICP_CAN_IO_RUN_GET:
		*out_size = sizeof(icp_can_run_mode_t);
		break;
	case ICP_CAN_IO_TIMING_GET:
		*out_size = sizeof(icp_can_timing_t);
		break;
	case ICP_CAN_IO_FILTER_GET:
		*out_size = sizeof(icp_can_rx_filter_t);
		break;
	case ICP_CAN_IO_BLOCK_GET:
		*out_size = sizeof(unsigned int);
		break;
	case ICP_CAN_IO_LISTEN_GET:
		*out_size = sizeof(icp_can_listen_mode_t);
		break;
	case ICP_CAN_IO_ARBITER_GET:
		*out_size = sizeof(icp_can_arbiter_t);
		break;
	case ICP_CAN_IO_ERROR_STATS_GET:
		*out_size = sizeof(icp_can_error_t);
		break;
	case ICP_CAN_IO_RESTART_MODE_GET:
		*out_size = sizeof(icp_can_auto_restart_t);
		break;
	case ICP_CAN_IO_BUFFER_LINK_SET:
	case ICP_CAN_IO_BUFFER_LINK_CLEAR:
	case ICP_CAN_IO_RX_ENABLE_SET:
	case ICP_CAN_IO_RX_ENABLE_CLEAR:
	case ICP_CAN_IO_TX_ENABLE_SET:
	case ICP_CAN_IO_TX_ENABLE_CLEAR:
		*in_size = sizeof(unsigned int);
		break;
	case ICP_CAN_IO_BUFFER_LINK_GET:
	case ICP_CAN_IO_RX_ENABLE_GET:
	case ICP_CAN_IO_TX_ENABLE_GET:
		*in_size = sizeof(unsigned int);
		*out_size = sizeof(unsigned int);
		break;
	default:
		break;
	}
}
