/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/

/**
 *****************************************************************************
 * @file icp_can_types.h
 *
 * @defgroup icp_CanAcc_Types Public types for CAN I/O Unit.
 * 
 * @description
 *         These functions specify the hardware access function used to
 *         maipulate the CAN device.
 * 
 *****************************************************************************/

#ifndef __ICP_CAN_TYPES_H__
#define __ICP_CAN_TYPES_H__

/**
 * @ingroup icp_CanAcc_Types
 * Message data length.
 * @description
 * Used to define the CAN message data length.
 */
#define ICP_CAN_MSG_DATA_LEN		8	/**< CAN Msg data length     */

/**
 * @ingroup icp_CanAcc_Types
 * Debug output.
 * @description
 * Used to enable/disable deboug output.
 */
/*#define ICP_CAN_DEBUG 				1*/	/**< CAN debug enable/disable */

/*****************************************************************************/

/*
 ******************************************************************************
 * Structures
 ******************************************************************************
 */

/**
 * @ingroup icp_CanAcc_Types
 * ICP handle 
 * @description
 * This type uniquely identifies an ICP CAN object.
 */
typedef int icp_can_handle_t;			/**< Generic handle type     */

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Message
 * @description
 * This type defines a CAN messages.
 */
typedef struct icp_can_msg {
	unsigned short ide;			/**< Standard/extended msg   */
	unsigned int id;			/**< 11 or 29 bit msg id     */
	unsigned short dlc;			/**< Size of data            */
	unsigned char data[ICP_CAN_MSG_DATA_LEN];/**< Message pay load       */
	unsigned short rtr;			/**< RTR message             */
} icp_can_msg_t;

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Timing
 * @description
 * This type defines the parameters that define the CAN timing.
 */
typedef struct icp_can_timing {
	unsigned int bitrate;			/**< Bitrate (kbps) 	     */
	unsigned int cfg_bitrate;		/**< Bitrate 		     */
	unsigned int cfg_tseg1;			/**< Tseg1 		     */
	unsigned int cfg_tseg2;			/**< Tseg2 		     */
	unsigned int cfg_sjw;			/**< Sync jump width 	     */
	unsigned int smpl_mode;			/**< Sampling mode 	     */
	unsigned int edge_mode;			/**< Edge R / D		     */
} icp_can_timing_t;

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Error
 * @description
 * This type defines a CAN error stats.
 */
typedef struct icp_can_error {
	unsigned int rxgte96;			/**< Rx err cnt >=96   	     */
	unsigned int txgte96;			/**< Tx err cnt >=96   	     */
	unsigned int error_stat;		/**< Error state of CAN node */
						/**< 00=error active (normal)*/
						/**< 01=error passive        */
						/**< 1x=bus off              */
	unsigned int rx_err_cnt;		/**< Rx counter              */
	unsigned int tx_err_cnt;		/**< Tx counter              */
} icp_can_error_t;

/*
 ******************************************************************************
 * Enumerated Types
 ******************************************************************************
 */

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Filter
 * @description
 * This structure contains the filter information for ACR and AMR filter.
 */
typedef struct icp_can_acc_filter {
	unsigned int id;			/**< ID 					  */
	unsigned int id_ext;		/**< Standard/extended ID?    */
	unsigned int rtr;			/**< RTR message 			  */
	unsigned short data;		/**< High byte pair  	      */
} icp_can_acc_filter_t;

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Rx Filter
 * @description
 * This structures describes the ACR and AMR filter for an Rx buffer.
 */
typedef struct icp_can_rx_filter {
        unsigned int num;			/**< Filter number 	     */
	icp_can_acc_filter_t amr;		/**< Acceptance Mask Reg     */
	icp_can_acc_filter_t acr;		/**< Acceptance Control Reg  */
} icp_can_rx_filter_t;

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Active/Listen Mode
 * @description
 * Idententies the valid values for the Active/Listen mode.
 */
typedef enum {
	ICP_CAN_ACTIVE = 0,			/**< R/w to/from the CAN     */
	ICP_CAN_LISTEN				/**< Only read from the CAN  */
} icp_can_listen_mode_t;

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Run/Stop Mode
 * @description
 * Identifies the valid values for the Run/Stop mode.
 */
typedef enum {
	ICP_CAN_STOP = 0,		/**< CAN stopped 	     */
	ICP_CAN_RUN				/**< CAN running  	     */
} icp_can_run_mode_t;

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Arbitration Mode
 * @description
 * Identifies the valid values for the arbitration mode.
 */
typedef enum {
	ICP_CAN_ROUND_ROBIN = 0,		/**< Equal priority 	     */
	ICP_CAN_FIXED_PRIORITY			/**< Buffer num priority     */
} icp_can_arbiter_t;

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Sampling Mode
 * @description
 * Identifies the valid values for the sampling mode.
 */
typedef enum {
	ICP_CAN_ONE_POINT = 0,			/**< One sampling point	     */
	ICP_CAN_THREE_POINTS			/**< Three sampling points   */
} icp_can_sampling_mode_t;

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Sync Mode
 * @description
 * Identifies the valid values for the sync mode.
 */
typedef enum {
	ICP_CAN_EDGE_R_TO_D = 0,	/**< Recessive to dominant   */
	ICP_CAN_BOTH_EDGES			/**< Trigger on both edges   */
} icp_can_sync_t;

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Auto Restart Mode
 * @description
 * Identifies the valid values for the auto-restart mode.
 */
typedef enum {
	ICP_CAN_MANUAL = 0,			/**< Manual restart 	     */
	ICP_CAN_AUTO				/**< Automatic restart       */
} icp_can_auto_restart_t;


/**
 * @ingroup icp_CanAcc_Types
 *	ICP CAN Common baud rates
 * @description
 *	Identifies common baudrates.
 */
typedef enum {
	ICP_CAN_BAUD_10 = 0,		/**< 10   kbps 	             */
	ICP_CAN_BAUD_20,			/**< 20   kbps 	             */
	ICP_CAN_BAUD_50,			/**< 50   kbps 	             */
	ICP_CAN_BAUD_125,			/**< 125  kbps 	             */
	ICP_CAN_BAUD_250,			/**< 250  kbps 	             */
	ICP_CAN_BAUD_500,			/**< 500  kbps 	             */
	ICP_CAN_BAUD_800,			/**< 800  kbps 	             */
	ICP_CAN_BAUD_1000			/**< 1000 kbps 	             */
} icp_can_baud_t;

/**
 * @ingroup icp_CanAcc_Types
 * ICP CAN Interrupt Enables
 * @description
 * Identifies interrupt enables/disables.
 */
typedef enum {
	ICP_CAN_ENABLE,				/**< Enable bit only	     */
	ICP_CAN_DISABLE,			/**< Disable bit only	     */
	ICP_CAN_ALL,				/**< All ints		     */
	ICP_CAN_NONE				/**< No ints		     */
} icp_can_interrupt_t;


#endif /* __ICP_CAN_TYPES_H__ */
