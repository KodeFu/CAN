/*******************************************************************************
 * %LICENSE_PROPRIETARY%
 * <COPYRIGHT_TAG>
 *******************************************************************************/

#ifndef __CAN_USER_H__
#define __CAN_USER_H__

/*******************************************************************************
 * Device IO control codes.
 *******************************************************************************/

#define IOCTL_CAN_BASE	0x8000

#define ICP_CAN_IO_RESET \
	CTL_CODE(IOCTL_CAN_BASE, 0x800, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_RUN \
   CTL_CODE(IOCTL_CAN_BASE, 0x801, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_STOP \
   CTL_CODE(IOCTL_CAN_BASE, 0x802, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_RUN_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x803, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_FILTER \
   CTL_CODE(IOCTL_CAN_BASE, 0x804, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_FILTER_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x805, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_CUSTOM \
   CTL_CODE(IOCTL_CAN_BASE, 0x806, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_SIMPLE \
   CTL_CODE(IOCTL_CAN_BASE, 0x807, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_TIMING_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x808, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_BLOCK \
   CTL_CODE(IOCTL_CAN_BASE, 0x809, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_NON_BLOCK \
   CTL_CODE(IOCTL_CAN_BASE, 0x80a, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_BLOCK_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x80b, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_LISTEN \
   CTL_CODE(IOCTL_CAN_BASE, 0x80c, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_ACTIVE \
   CTL_CODE(IOCTL_CAN_BASE, 0x80d, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_LISTEN_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x80e, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_ARBITER_ROUND_ROBIN \
   CTL_CODE(IOCTL_CAN_BASE, 0x80f, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_ARBITER_FIXED_PRIORITY \
   CTL_CODE(IOCTL_CAN_BASE, 0x810, METHOD_BUFFERED, FILE_ANY_ACCESS)
	 
#define ICP_CAN_IO_ARBITER_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x811, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_ERROR_STATS_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x812, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_RESTART_MODE_AUTO \
   CTL_CODE(IOCTL_CAN_BASE, 0x813, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_RESTART_MODE_MANUAL \
   CTL_CODE(IOCTL_CAN_BASE, 0x814, METHOD_BUFFERED, FILE_ANY_ACCESS)	 

#define ICP_CAN_IO_RESTART_MODE_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x815, METHOD_BUFFERED, FILE_ANY_ACCESS)	 
	 
#define ICP_CAN_IO_BUFFER_LINK_SET \
   CTL_CODE(IOCTL_CAN_BASE, 0x816, METHOD_BUFFERED, FILE_ANY_ACCESS)	 

#define ICP_CAN_IO_BUFFER_LINK_CLEAR \
   CTL_CODE(IOCTL_CAN_BASE, 0x817, METHOD_BUFFERED, FILE_ANY_ACCESS)	 

 #define ICP_CAN_IO_BUFFER_LINK_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x818, METHOD_BUFFERED, FILE_ANY_ACCESS)
	 
#define ICP_CAN_IO_RX_ENABLE_SET \
   CTL_CODE(IOCTL_CAN_BASE, 0x819, METHOD_BUFFERED, FILE_ANY_ACCESS)	 

#define ICP_CAN_IO_RX_ENABLE_CLEAR \
   CTL_CODE(IOCTL_CAN_BASE, 0x81a, METHOD_BUFFERED, FILE_ANY_ACCESS)	 

 #define ICP_CAN_IO_RX_ENABLE_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x81b, METHOD_BUFFERED, FILE_ANY_ACCESS)

#define ICP_CAN_IO_TX_ENABLE_SET \
   CTL_CODE(IOCTL_CAN_BASE, 0x81c, METHOD_BUFFERED, FILE_ANY_ACCESS)	 

#define ICP_CAN_IO_TX_ENABLE_CLEAR \
   CTL_CODE(IOCTL_CAN_BASE, 0x81d, METHOD_BUFFERED, FILE_ANY_ACCESS)	 

 #define ICP_CAN_IO_TX_ENABLE_GET \
   CTL_CODE(IOCTL_CAN_BASE, 0x81e, METHOD_BUFFERED, FILE_ANY_ACCESS)

#endif /* __CAN_USER_H__ */
