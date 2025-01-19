/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/

/**
 *****************************************************************************
 * @file icp_can_regs.h         
 *
 * @defgroup icp_CanAcc_Regs Registers for CAN I/O Unit.
 * 
 * @description
 *         This file contains the defition of the CAN device memory
 *	       mapped registers.
 * 
 *****************************************************************************/

#ifndef __ICP_CAN_REGS_H__
#define __ICP_CAN_REGS_H__

/*****************************************************************************
 * Device characteristics
 *****************************************************************************/
#define MM_REG_SIZE		0x29c   	/**< All CAN regs */
#define NUM_RX_BUFFS		16      /**< Num CAN Rx buffers */
#define NUM_TX_BUFFS		8		/**< Num CAN Tx buffers */

/*****************************************************************************
 * Memory map offset definitions 
 *****************************************************************************/
#define MM_INTSTAT		0x000		/**< Interrupt status */
#define MM_INTENBL		0x004		/**< Interrupt enables */
#define MM_BUFFSTAT		0x008		/**< Buffer status */
#define MM_ERRSTAT		0x00c		/**< Error status */
#define MM_CMD			0x010		/**< Command reg */
#define MM_CFG			0x014		/**< Config reg */

#define MM_TX0			0x020		/**< Tx reg 0 */
#define MM_TX1			0x030		/**< Tx reg 1 */
#define MM_TX2			0x040		/**< Tx reg 2 */
#define MM_TX3			0x050		/**< Tx reg 3 */
#define MM_TX4			0x060		/**< Tx reg 4 */
#define MM_TX5			0x070		/**< Tx reg 5 */
#define MM_TX6			0x080		/**< Tx reg 6 */
#define MM_TX7			0x090		/**< Tx reg 7 */

#define MM_RX0			0x0a0		/**< Rx reg 0 */
#define MM_RX1			0x0c0		/**< Tx reg 1 */
#define MM_RX2			0x0e0		/**< Tx reg 2 */
#define MM_RX3			0x100		/**< Tx reg 3 */
#define MM_RX4			0x120		/**< Tx reg 4 */
#define MM_RX5			0x140		/**< Tx reg 5 */
#define MM_RX6			0x160		/**< Tx reg 6 */
#define MM_RX7			0x180		/**< Tx reg 7 */
#define MM_RX8			0x1a0		/**< Tx reg 8 */
#define MM_RX9			0x1c0		/**< Tx reg 9 */
#define MM_RX10			0x1e0		/**< Tx reg 10 */
#define MM_RX11			0x200		/**< Tx reg 11 */
#define MM_RX12			0x220		/**< Tx reg 12 */
#define MM_RX13			0x240		/**< Tx reg 13 */
#define MM_RX14			0x260		/**< Tx reg 14 */
#define MM_RX15			0x280		/**< Tx reg 15 */

/*
 * Tx message register offsets. These offsets corresponds
 * to offsets within each of the MM_TXx registers
 */
#define MM_TXCTL		0x00		/**< Tx control */
#define MM_TXCMD		0x00		/**< Tx command */
#define MM_TXID			0x04		/**< Tx ID */
#define MM_TXDATAH		0x08		/**< Tx data high */
#define MM_TXDATAL		0x0c		/**< Tx data low */

/*
 * Rx message register offsets.These offsets corresponds
 * to offsets withing each of the MM_RXx registers
 */
#define MM_RXCTL		0x00		/**< Rx conrol */
#define MM_RXCMD		0x00		/**< Rx command */
#define MM_RXID			0x04		/**< Rx ID */
#define MM_RXDATAH		0x08		/**< Rx data high */
#define MM_RXDATAL		0x0c		/**< Rx data low */
#define MM_RXAMR		0x10		/**< Rx acceptance mask */
#define MM_RXACR		0x14		/**< Rx acceptance control */
#define MM_RXAMRDATA		0x18		/**< Rx AMR data */
#define MM_RXACRDATA		0x1c		/**< Rx ACR data */

/*****************************************************************************
 * Register mask and bit definitions
 *****************************************************************************/
/* Interrupt Status Bits */
#define MSK_IS_ARLOSS		(1<<BIT_IS_ARLOSS)	/**< Arbitration loss*/
#define MSK_IS_OVRLOAD		(1<<BIT_IS_OVRLOAD)	/**< Overload */
#define MSK_IS_BITERR		(1<<BIT_IS_BITERR)	/**< Bit error */
#define MSK_IS_STUFFERR		(1<<BIT_IS_STUFFERR)	/**< Stuff error */
#define MSK_IS_ACKERR		(1<<BIT_IS_ACKERR)	/**< ACK error */
#define MSK_IS_FORMERR		(1<<BIT_IS_FORMERR)	/**< Form error */
#define MSK_IS_CRCERR		(1<<BIT_IS_CRCERR)	/**< CRC error */
#define MSK_IS_BUSOFF		(1<<BIT_IS_BUSOFF)	/**< Bus off */
#define MSK_IS_RXMSGLOSS	(1<<BIT_IS_RXMSGLOSS)	/**< Rx msg loss */
#define MSK_IS_TXMSG		(1<<BIT_IS_TXMSG)	/**< Tx msg */
#define MSK_IS_RXMSG		(1<<BIT_IS_RXMSG)	/**< Rx msg */

#define BIT_IS_ARLOSS		2
#define BIT_IS_OVRLOAD		3
#define BIT_IS_BITERR		4
#define BIT_IS_STUFFERR		5
#define BIT_IS_ACKERR		6
#define BIT_IS_FORMERR		7
#define BIT_IS_CRCERR		8
#define BIT_IS_BUSOFF		9
#define BIT_IS_RXMSGLOSS	10
#define BIT_IS_TXMSG		11
#define BIT_IS_RXMSG		12

/* Interrupt enable register */
#define MSK_IE_INTENBL		(1<<BIT_IE_INTENBL)	/**< Ints Enabled */
#define MSK_IE_ARLOSS		(1<<BIT_IE_ARLOSS)	/**< Arbitration loss*/
#define MSK_IE_OVRLOAD		(1<<BIT_IE_OVRLOAD)	/**< Overload */
#define MSK_IE_BITERR		(1<<BIT_IE_BITERR)	/**< Bit error */
#define MSK_IE_STUFFERR		(1<<BIT_IE_STUFFERR)	/**< Stuff error */
#define MSK_IE_ACKERR		(1<<BIT_IE_ACKERR)	/**< ACK error */
#define MSK_IE_FORMERR		(1<<BIT_IE_FORMERR)	/**< Form error */
#define MSK_IE_CRCERR		(1<<BIT_IE_CRCERR)	/**< CRC error */
#define MSK_IE_BUSOFF		(1<<BIT_IE_BUSOFF)	/**< Bus off */
#define MSK_IE_RXMSGLOSS	(1<<BIT_IE_RXMSGLOSS)	/**< Rx msg loss */
#define MSK_IE_TXMSG		(1<<BIT_IE_TXMSG)	/**< Tx msg */
#define MSK_IE_RXMSG		(1<<BIT_IE_RXMSG)	/**< Rx msg */

#define MSK_IE_ALL		0x1ffd			/**< Enable all ints */
#define MSK_IE_ERR		0x07fc			/**< Enable all errs */

#define BIT_IE_INTENBL		0
#define BIT_IE_ARLOSS		2
#define BIT_IE_OVRLOAD		3
#define BIT_IE_BITERR		4
#define BIT_IE_STUFFERR		5
#define BIT_IE_ACKERR		6
#define BIT_IE_FORMERR		7
#define BIT_IE_CRCERR		8
#define BIT_IE_BUSOFF		9
#define BIT_IE_RXMSGLOSS	10
#define BIT_IE_TXMSG		11
#define BIT_IE_RXMSG		12

/* Buffer status indicators */
#define MSK_BS_RXMSG0		(1<<BIT_BS_RXMSG0)	/**< Rx msg 0 */
#define MSK_BS_RXMSG1		(1<<BIT_BS_RXMSG1)	/**< Rx msg 1 */
#define MSK_BS_RXMSG2		(1<<BIT_BS_RXMSG2)	/**< Rx msg 2 */
#define MSK_BS_RXMSG3		(1<<BIT_BS_RXMSG3)	/**< Rx msg 3 */
#define MSK_BS_RXMSG4		(1<<BIT_BS_RXMSG4)	/**< Rx msg 4 */
#define MSK_BS_RXMSG5		(1<<BIT_BS_RXMSG5)	/**< Rx msg 5 */
#define MSK_BS_RXMSG6		(1<<BIT_BS_RXMSG6)	/**< Rx msg 6 */
#define MSK_BS_RXMSG7		(1<<BIT_BS_RXMSG7)	/**< Rx msg 7 */
#define MSK_BS_RXMSG8		(1<<BIT_BS_RXMSG8)	/**< Rx msg 8 */
#define MSK_BS_RXMSG9		(1<<BIT_BS_RXMSG9)	/**< Rx msg 9 */
#define MSK_BS_RXMSG10		(1<<BIT_BS_RXMSG10)	/**< Rx msg 10 */
#define MSK_BS_RXMSG11		(1<<BIT_BS_RXMSG11)	/**< Rx msg 11 */
#define MSK_BS_RXMSG12		(1<<BIT_BS_RXMSG12)	/**< Rx msg 12 */
#define MSK_BS_RXMSG13		(1<<BIT_BS_RXMSG13)	/**< Rx msg 13 */
#define MSK_BS_RXMSG14		(1<<BIT_BS_RXMSG14)	/**< Rx msg 14 */
#define MSK_BS_RXMSG15		(1<<BIT_BS_RXMSG15)	/**< Rx msg 15 */

#define MSK_BS_RXALL		0x0000FFFF		/**< All Rx msg bufs */

/* Transmit msg buffers statuses */
#define MSK_BS_TXMSG0		(1<<BIT_BS_TXMSG0)	/**< Tx msg 0 */
#define MSK_BS_TXMSG1		(1<<BIT_BS_TXMSG1)	/**< Tx msg 1 */
#define MSK_BS_TXMSG2		(1<<BIT_BS_TXMSG2)	/**< Tx msg 2 */
#define MSK_BS_TXMSG3		(1<<BIT_BS_TXMSG3)	/**< Tx msg 3 */
#define MSK_BS_TXMSG4		(1<<BIT_BS_TXMSG4)	/**< Tx msg 4 */
#define MSK_BS_TXMSG5		(1<<BIT_BS_TXMSG5)	/**< Tx msg 5 */
#define MSK_BS_TXMSG6		(1<<BIT_BS_TXMSG6)	/**< Tx msg 6 */
#define MSK_BS_TXMSG7		(1<<BIT_BS_TXMSG7)	/**< Tx msg 7 */

#define MSK_BS_TXALL		0x00FF0000		/**< All Tx msg bufs */

#define BIT_BS_RXMSG0		0
#define BIT_BS_RXMSG1		1
#define BIT_BS_RXMSG2		2
#define BIT_BS_RXMSG3		3
#define BIT_BS_RXMSG4		4
#define BIT_BS_RXMSG5		5
#define BIT_BS_RXMSG6		6
#define BIT_BS_RXMSG7		7
#define BIT_BS_RXMSG8		8
#define BIT_BS_RXMSG9		9
#define BIT_BS_RXMSG10		10
#define BIT_BS_RXMSG11		11
#define BIT_BS_RXMSG12		12
#define BIT_BS_RXMSG13		13
#define BIT_BS_RXMSG14		14
#define BIT_BS_RXMSG15		15
#define BIT_BS_TXMSG0		16
#define BIT_BS_TXMSG1		17
#define BIT_BS_TXMSG2		18
#define BIT_BS_TXMSG3		19
#define BIT_BS_TXMSG4		20
#define BIT_BS_TXMSG5		21
#define BIT_BS_TXMSG6		22
#define BIT_BS_TXMSG7		23

/* Error status */
#define MSK_ES_TXERRCNT		(0xff<<BIT_ES_TXERRCNT)	/**< Tx err count */
#define MSK_ES_RXERRCNT		(0xff<<BIT_ES_RXERRCNT)	/**< Rx err count */
#define MSK_ES_ERRSTAT		(0x3<<BIT_ES_ERRSTAT)	/**< Error state */
#define MSK_ES_TXGTE96		(1<<BIT_ES_TXGTE96)	/**< Tx >= 96 */
#define MSK_ES_RXGTE96		(1<<BIT_ES_RXGTE96)	/**< Rx >= 96 */

#define BIT_ES_TXERRCNT		0
#define BIT_ES_RXERRCNT		8
#define BIT_ES_ERRSTAT		16
#define BIT_ES_TXGTE96		18
#define BIT_ES_RXGTE96		19

/* Command */
#define MSK_CMD_RUN		(1<<BIT_CMD_RUN)	/**< Run mode */
#define MSK_CMD_LISTEN		(1<<BIT_CMD_LISTEN)	/**< Listen mode */

#define BIT_CMD_RUN		0
#define BIT_CMD_LISTEN		1

/* Configuration */
#define MSK_CFG_EDGE		(1<<BIT_CFG_EDGE)	/**< Edge trig */
#define MSK_CFG_SMPL		(1<<BIT_CFG_SMPL)	/**< Sample mode */
#define MSK_CFG_SJW		(0x3<<BIT_CFG_SJW)	/**< Synch jump width*/
#define MSK_CFG_RSTRT		(1<<BIT_CFG_RSTRT)	/**< Auto restart */
#define MSK_CFG_TSEG2		(0x7<<BIT_CFG_TSEG2)	/**< Time segment 2 */
#define MSK_CFG_TSEG1		(0xf<<BIT_CFG_TSEG1)	/**< Time segment 1 */
#define MSK_CFG_ARB		(1<<BIT_CFG_ARB)	/**< Arb: RR/Fixed */
#define MSK_CFG_BITRATE		(0x3fff<<BIT_CFG_BITRATE)/**< Prescaler */

#define BIT_CFG_EDGE		0
#define BIT_CFG_SMPL		1
#define BIT_CFG_SJW		2
#define BIT_CFG_RSTRT		4
#define BIT_CFG_TSEG2		5
#define BIT_CFG_TSEG1		8
#define BIT_CFG_ARB		12
#define BIT_CFG_BITRATE		16

/* TxMessage control */
#define MSK_TXCTL_INTENBL	(1<<BIT_TXCTL_INTENBL)	/**< Tx int enable */
#define MSK_TXCTL_WPN0		(1<<BIT_TXCTL_WPN0)	/**< WPN b[2] */
#define MSK_TXCTL_DLC		(0xf<<BIT_TXCTL_DLC)	/**< Data length */
#define MSK_TXCTL_IDE		(1<<BIT_TXCTL_IDE)	/**< Ext ID? */
#define MSK_TXCTL_RTR		(1<<BIT_TXCTL_RTR)	/**< RTR? */
#define MSK_TXCTL_WPN1		(1<<BIT_TXCTL_WPN1)	/**< WPN b[21:16] */

#define BIT_TXCTL_INTENBL	2
#define BIT_TXCTL_WPN0		3
#define BIT_TXCTL_DLC		16
#define BIT_TXCTL_IDE		20
#define BIT_TXCTL_RTR		21
#define BIT_TXCTL_WPN1		23

/* TxMessage command */
#define MSK_TXCMD_REQ		(1<<BIT_TXCMD_REQ)	/**< Req tx */
#define MSK_TXCMD_ABORT		(1<<BIT_TXCMD_ABORT)	/**< Req abort */
#define MSK_TXCMD_WPN0		(1<<BIT_TXCMD_WPN0)	/**< WPN 0 */
#define MSK_TXCMD_WPN1		(1<<BIT_TXCMD_WPN1)	/**< WPN 1 */

#define BIT_TXCMD_REQ		0
#define BIT_TXCMD_ABORT		1
#define BIT_TXCMD_WPN0		3
#define BIT_TXCMD_WPN1		23

/* TxMessage ID */
#define MSK_TXMSGIDS		0xfffffff8
#define BIT_TXID			3
#define BIT_TXID_UPPER		21

/* RxMessage control */
#define MSK_RXCTL_BUFENBL	(1<<BIT_RXCTL_BUFENBL)	/**< Buf enabled */
#define MSK_RXCTL_RTRRPLY	(1<<BIT_RXCTL_RTRRPLY)	/**< RTR msg */
#define MSK_RXCTL_INTENBL	(1<<BIT_RXCTL_INTENBL)	/**< Int enabled */
#define MSK_RXCTL_LNKFLG	(1<<BIT_RXCTL_LNKFLG)	/**< Linked buffer */
#define MSK_RXCTL_WPNL		(1<<BIT_RXCTL_WPNL)	/**< WPN b[6:3] */
#define MSK_RXCTL_DLC		(0xf<<BIT_RXCTL_DLC)	/**< Data len code */
#define MSK_RXCTL_IDE		(1<<BIT_RXCTL_IDE)	/**< Ext ID? */
#define MSK_RXCTL_RTR		(1<<BIT_RXCTL_RTR)	/**< RTR? */
#define MSK_RXCTL_WPNH		(1<<BIT_RXCTL_WPNH)	/**< WPN b[21:16] */

#define BIT_RXCTL_BUFENBL	3
#define BIT_RXCTL_RTRRPLY	4
#define BIT_RXCTL_INTENBL	5
#define BIT_RXCTL_LNKFLG	6
#define BIT_RXCTL_WPNL		7
#define BIT_RXCTL_DLC		16
#define BIT_RXCTL_IDE		20
#define BIT_RXCTL_RTR		21
#define BIT_RXCTL_WPNH		23

/* RxMessage command */
#define MSK_RXCMD_MSGAV		(1<<BIT_RXCMD_MSGAV)	/**< Msg avail */
#define MSK_RXCMD_RTRRPLYPEND	(1<<BIT_RXCMD_RTRRPLYPEND)/**< RTR reply pend*/
#define MSK_RXCMD_RTRABORT	(1<<BIT_RXCMD_RTRABORT)	/**< RTR abort */
#define MSK_RXCMD_WPNL		(1<<BIT_RXCMD_WPNL)	/**< WPNL */
#define MSK_RXCMD_WPNH		(1<<BIT_RXCMD_WPNH)	/**< WPNH */

#define BIT_RXCMD_MSGAV		0
#define BIT_RXCMD_RTRRPLYPEND	1
#define BIT_RXCMD_RTRABORT	2
#define BIT_RXCMD_WPNL		7
#define BIT_RXCMD_WPNH		23

/* RxMessage ID */
#define MSK_RXID		(0x1fffffff<<BIT_RXMSGIDS)/**< Rx msg ID */
#define BIT_RXID		3
#define BIT_RXID_UPPER	21

/* RxMessage acceptance mask register */
#define MSK_RXAMR_RTR		(1<<BIT_RXAMR_RTR)	/**< Rx AMR RTR */
#define MSK_RXAMR_IDE		(1<<BIT_RXAMR_IDE)	/**< Rx AMR IDE? */
#define MSK_RXAMR_ID		(0x1fffffff<<BIT_RXAMR_ID) /**< Rx AMR ID */
#define MSK_RXAMR_ID_UPPER	(0x7ff<<BIT_RXAMR_ID_UPPER) /**< Rx AMR ID UPPER*/

#define BIT_RXAMR_RTR		1
#define BIT_RXAMR_IDE		2
#define BIT_RXAMR_ID		3
#define BIT_RXAMR_ID_UPPER	21

/* RxMessage acceptance code register */
#define MSK_RXACR_RTR		(1<<BIT_RXACR_RTR)	/**< Rx ACR RTR */
#define MSK_RXACR_IDE		(1<<BIT_RXACR_IDE)	/**< Rx ACR IDE? */
#define MSK_RXACR_ID		(0x1fffffff<<BIT_RXACR_ID)/**< Rx ACR ID */
#define MSK_RXACR_ID_UPPER	(0x7ff<<BIT_RXACR_ID_UPPER) /**< Rx ACR ID UPPER*/

#define BIT_RXACR_RTR		1
#define BIT_RXACR_IDE		2
#define BIT_RXACR_ID		3
#define BIT_RXACR_ID_UPPER	21

/* RxMessage acceptance mask register - data */
#define MSK_RXAMRDATA		0x0000ffff		/**< Rx AMR data */
#define BIT_RXAMRDATA		0

/* RxMessage acceptance code register - data */
#define MSK_RXACRDATA		0x0000ffff		/**< Rx ACR data */
#define BIT_RXACRDATA		0

#endif /* __ICP_CAN_REGS_H__ */
