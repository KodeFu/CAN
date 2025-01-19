/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/

/**************************************************************************
 * @ingroup ICP_CAN_GENERAL
 *
 * @file icp_can.c
 *
 * @description
 *   
 **************************************************************************/

#include "icp_can.h"
#include "icp_can_regs.h"

/*****************************************************************************
 * CAN HW context structure
 *****************************************************************************/

typedef struct can_hw {
    unsigned char *io_base;                /* Device registers  */
} can_hw_t;

/*****************************************************************************
 * CAN recommended timings
 *    See declaraion of timing_t for field definitions.
 ****************************************************************************/
/* Table entries converted to hex to resolve Parasoft complaints */
/* static icp_can_timing_t can_rec_timing[] = { */
/*    {10,  249, 12, 1, 0, 0, 0},           10 kbits/s         */
/*    {20,  124, 12, 1, 0, 0, 0},           20 kbits/s         */
/*    {50,   49, 12, 1, 0, 0, 0},           50 kbits/s         */
/*    {125,  19, 12, 1, 0, 0, 0},          125 kbits/s         */
/*    {250,   9, 12, 1, 0, 0, 0},          250 kbits/s         */
/*    {500,   4, 12, 1, 0, 0, 0},          500 kbits/s         */
/*    {800,   4,  6, 1, 0, 0, 0},          800 kbits/s         */
/*    {1000,  4,  4, 1, 0, 0, 0}          1000 kbits/s         */
/*}; */

 static icp_can_timing_t can_rec_timing[] = {
    {0xa,  0xf9, 0xc, 0x1, 0x0, 0x0, 0x0},        /*   10 kbits/s         */
    {0x14, 0x7c, 0xc, 0x1, 0x0, 0x0, 0x0},        /*   20 kbits/s         */
    {0x32, 0x31, 0xc, 0x1, 0x0, 0x0, 0x0},        /*   50 kbits/s         */
    {0x7d, 0x13, 0xc, 0x1, 0x0, 0x0, 0x0},        /*  125 kbits/s         */
    {0xfa, 0x9,  0xc, 0x1, 0x0, 0x0, 0x0},        /*  250 kbits/s         */
    {0x1f4, 0x4,  0xc, 0x1, 0x0, 0x0, 0x0},       /*  500 kbits/s         */
    {0x320, 0x4,  0x6, 0x1, 0x0, 0x0, 0x0},       /*  800 kbits/s         */
    {0x3e8, 0x4,  0x4, 0x1, 0x0, 0x0, 0x0}        /* 1000 kbits/s         */
};

/*****************************************************************************
 * Sets the value of a specified memory mapped register.
 *   can_reg_set mask's in value bits. Prior bits are preserved..
 *   can_reg_set_fast just writes the value. Prior bits are not preserved.
 *****************************************************************************/
void can_reg_set(
    icp_can_handle_t    handle, 
    unsigned int        offset, 
    unsigned int        mask,
    unsigned int        value);

void can_reg_set_fast(
    icp_can_handle_t    handle, 
    unsigned int        offset, 
    unsigned int        value);

/*****************************************************************************
 * Gets the value of a specified memory mapped register.
 *****************************************************************************/
unsigned int can_reg_get(
    icp_can_handle_t     handle, 
    unsigned int         offset);

/*****************************************************************************
 * Dumps CAN Memory Mapped Registers
 *****************************************************************************/
int DEBUG_DUMP(icp_can_handle_t handle, char *info, unsigned int level);
    
/*****************************************************************************
 * Sets the value of a specified memory mapped register.
 *****************************************************************************/
void can_reg_set(
    icp_can_handle_t    handle, 
    unsigned int        offset, 
    unsigned int        mask,
    unsigned int        value)
{
    unsigned int *reg = NULL;
    volatile unsigned int tmp;
    can_hw_t *can = (can_hw_t *) handle;    
    
    reg = (unsigned int *) (can->io_base + offset);

    tmp = CAN_REG_READ(reg);    /* get the reg */
    tmp &= ~mask;                /* clear the bits */
    tmp |= (value & mask);        /* set the bits */

    CAN_REG_WRITE(reg, tmp);    /* write the reg */
}

/*****************************************************************************
 * Sets the value of a specified memory mapped register.
 *****************************************************************************/
void can_reg_set_fast(
    icp_can_handle_t    handle, 
    unsigned int        offset, 
    unsigned int        value)
{
    unsigned int *reg = NULL;
    can_hw_t *can = (can_hw_t *) handle;    
    
    if (handle) {
        reg = (unsigned int *) (can->io_base + offset);
        CAN_REG_WRITE(reg, value);    /* write the reg */
    }  else {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
    }
}

/*****************************************************************************
 * Sets the value of a specified memory mapped register.
 *****************************************************************************/
unsigned int can_reg_get(
    icp_can_handle_t    handle, 
    unsigned int        offset)
{
    unsigned int *reg = NULL;
    can_hw_t *can = (can_hw_t *) handle;
    unsigned int value = 0;
    
    if (handle) {
        reg = (unsigned int *) (can->io_base + offset);
        value = CAN_REG_READ(reg);
    } else {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
    }

    return value;
}

/*****************************************************************************
 * Initializes the CAN device for use. Returns a handle to the CAN context.
 *****************************************************************************/
icp_can_handle_t icp_can_create(unsigned char *io_base)
{
    can_hw_t *can;
    
    DEBUG_OUT("icp_can_create");
    
    if (!io_base) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_ALLOC, "PCI remap");
        return -1;
    }
    
    can = (can_hw_t *) CAN_MEM_ALLOC(sizeof(can_hw_t));
    
    if (!can) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_ALLOC, "device handle");
        return (icp_can_handle_t) NULL;
    }
    
    can->io_base = io_base;

    return (icp_can_handle_t) can;
}

/*****************************************************************************
 * Free's CAN the CAN context. The CAN can no longer be used after this call.
 *****************************************************************************/
void icp_can_destroy(icp_can_handle_t handle)
{    
    can_hw_t *can = (can_hw_t *) handle;
    
    DEBUG_OUT("icp_can_destroy");
        
    if (handle) {
        CAN_MEM_FREE(can);
    } else {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_FREE, "handle");
    }
}

/*****************************************************************************
 * Sets the CAN to run or listen mode.
 *****************************************************************************/
int icp_can_set_run_mode(
    icp_can_handle_t    handle, 
    icp_can_run_mode_t    mode)
{
    int err = 0;
    /*unsigned int txreq_status;*/
    
    DEBUG_OUT("icp_can_set_run_mode");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    switch (mode) {
        case ICP_CAN_RUN:
            can_reg_set(handle, MM_CMD, MSK_CMD_RUN, MSK_CMD_RUN);
            break;
        case ICP_CAN_STOP:
            /*if ((txreq_status=(icp_can_get_buffer_status(handle) 
                & MSK_BS_TXALL))) {
                icp_can_set_listen_mode(handle, ICP_CAN_LISTEN);
            }*/
            can_reg_set(handle, MM_CMD, MSK_CMD_RUN, 0);
            break;
        default:
            CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "run mode");
            err = -1;
            break;
    }
    
    return err;
}

/*****************************************************************************
 * Sets the CAN to run or listen mode.
 *****************************************************************************/
 int icp_can_get_run_mode(
    icp_can_handle_t     handle,
    icp_can_run_mode_t    *mode)
{
    unsigned int cmd_reg; 
    
    DEBUG_OUT("icp_can_get_run_mode");
    
    if ((!handle) || (!mode)){
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    cmd_reg = can_reg_get(handle, MM_CMD);
    
    if (cmd_reg & 1) {
        *mode = ICP_CAN_RUN;
    } else {
        *mode = ICP_CAN_STOP;
    }
    
    return 0;
}

/*****************************************************************************
 * Sets CAN's arbiter mode.
 *****************************************************************************/
int icp_can_set_arbiter_mode(
    icp_can_handle_t    handle, 
    icp_can_arbiter_t    mode)
{
    int err = 0;
    
    DEBUG_OUT("icp_can_set_arbiter_mode");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }

    switch (mode) {
        case ICP_CAN_ROUND_ROBIN:
            can_reg_set(handle, MM_CFG, MSK_CFG_ARB, 0);
            break;
        case ICP_CAN_FIXED_PRIORITY:
            can_reg_set(handle, MM_CFG, MSK_CFG_ARB, MSK_CFG_ARB);
            break;
        default:
            CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "arbiter mode");
            err = -1;
            break;
    }
    
    return err;
}

/*****************************************************************************
 * Gets CAN's arbiter mode.
 *****************************************************************************/
int icp_can_get_arbiter_mode(
    icp_can_handle_t    handle, 
    icp_can_arbiter_t    *mode)
{
    int err = 0;
    unsigned int reg;
    
    DEBUG_OUT("icp_can_get_arbiter_mode");
    
    if ((!handle)  || (!mode)) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle or arb mode");
        return -1;
    }

    reg = can_reg_get(handle, MM_CFG);
    
    if (reg & MSK_CFG_ARB) {
        *mode = ICP_CAN_FIXED_PRIORITY;
    } else {
        *mode = ICP_CAN_ROUND_ROBIN;
    }
    
    return err;
}

/*****************************************************************************
 * Sets CAN's restart mode.
 *****************************************************************************/
int icp_can_set_restart_mode(
    icp_can_handle_t    handle, 
    icp_can_auto_restart_t    mode)
{
    int err = 0;
    
    DEBUG_OUT("icp_can_set_restart_mode");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    switch (mode) {
        case ICP_CAN_MANUAL:
            can_reg_set(handle, MM_CFG, MSK_CFG_RSTRT, 0<<BIT_CFG_RSTRT);
            break;
        case ICP_CAN_AUTO:
            can_reg_set(handle, MM_CFG, MSK_CFG_RSTRT, 1<<BIT_CFG_RSTRT);
            break;
        default:
            CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "auto-restart mode");
            err = -1;
            break;
    }
    
    return err;
}

/*****************************************************************************
 * Gets CAN's restart mode.
 *****************************************************************************/
int 
icp_can_get_restart_mode(
    icp_can_handle_t handle,
    icp_can_auto_restart_t *mode)
{
    int err = 0;
    unsigned int reg;
    
    DEBUG_OUT("icp_can_get_restart_mode");
    
    if ((!handle)  || (!mode)) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle or restart mode");
        return -1;
    }

    reg = can_reg_get(handle, MM_CFG);
    
    if (reg & (1<<BIT_CFG_RSTRT) ) {
        *mode = ICP_CAN_AUTO;
    } else {
        *mode = ICP_CAN_MANUAL;
    }
    
    return err;
}

/*****************************************************************************
 * Sets CAN's active/listen mode.
 *****************************************************************************/
int icp_can_set_listen_mode(
    icp_can_handle_t    handle, 
    icp_can_listen_mode_t    mode)
{
    int err = 0;

    DEBUG_OUT("icp_can_set_listen_mode");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    switch (mode) {
        case ICP_CAN_LISTEN:
            can_reg_set(handle, MM_CMD, MSK_CMD_LISTEN, MSK_CMD_LISTEN);
            break;
        case ICP_CAN_ACTIVE:
            can_reg_set(handle, MM_CMD, MSK_CMD_LISTEN, 0);
            break;
        default:
            CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "listen mode");
            err = -1;
            break;
    }
    
    return err;
}

/*****************************************************************************
 * Sets the CAN to run or listen mode.
 *****************************************************************************/
int icp_can_get_listen_mode(
    icp_can_handle_t         handle,
    icp_can_listen_mode_t        *mode)
{
    unsigned int reg; 
    
    DEBUG_OUT("icp_can_get_listen_mode");
    
    if ((!handle) || (!mode)) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle or listen mode");
        return -1;
    }
    
    reg = can_reg_get(handle, MM_CMD);
    
    if (reg & (1<<BIT_CMD_LISTEN)) {
        *mode = ICP_CAN_LISTEN;
    } else {
        *mode = ICP_CAN_ACTIVE;
    }
    
    return 0;
}

/*****************************************************************************
 * Enables specific interrupts.
 *****************************************************************************/
int icp_can_set_int_custom(
    icp_can_handle_t    handle, 
    unsigned int        interrupts)
{
    unsigned int *reg = NULL;
    can_hw_t *can = (can_hw_t *) handle;
    
    DEBUG_OUT("icp_can_set_int_custom");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    reg = (unsigned int *) (can->io_base + MM_INTENBL);
    CAN_REG_WRITE(reg, interrupts);

    return 0;
}

int icp_can_get_int_enables(
    icp_can_handle_t    handle, 
    unsigned int        *enables)
{
    unsigned int *reg = NULL;
    can_hw_t *can = (can_hw_t *) handle;
    
    DEBUG_OUT("icp_can_get_int_enables");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    reg = (unsigned int *) (can->io_base + MM_INTENBL);
    *enables = CAN_REG_READ(reg);
    
    return 0;
}

/*****************************************************************************
 * Enables/disables interrupts and sets "interrupt enables".
 *****************************************************************************/
int icp_can_set_int_enables(
    icp_can_handle_t    handle, 
    icp_can_interrupt_t    interrupt)
{
    unsigned int *reg = NULL;
    volatile unsigned int tmp;
    int err = 0;
    can_hw_t *can = (can_hw_t *) handle;
    
    DEBUG_OUT("icp_can_set_int_enables");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    switch (interrupt)
    {
        case ICP_CAN_ENABLE:
            reg = (unsigned int *) (can->io_base + MM_INTENBL);
            tmp = CAN_REG_READ(reg);
            tmp &= ~MSK_IE_INTENBL;
            tmp |= MSK_IE_INTENBL;
            CAN_REG_WRITE(reg, tmp);
            break;
        case ICP_CAN_DISABLE:
            reg = (unsigned int *) (can->io_base + MM_INTENBL);
            tmp = CAN_REG_READ(reg);
            tmp &= ~MSK_IE_INTENBL;
            CAN_REG_WRITE(reg, tmp);
            break;
        case ICP_CAN_ALL:
            can_reg_set(handle, MM_INTENBL, MSK_IE_ALL, MSK_IE_ALL);
            break;
        case ICP_CAN_NONE:
            can_reg_set(handle, MM_INTENBL,    MSK_IE_ALL, 0x0);
            break;
        default:
            CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "interrupt");
            err = -1;
            break;
    }

    return err;
}

int icp_can_rx_enable_all(icp_can_handle_t handle)
{
    unsigned int i;
    
    DEBUG_OUT("icp_can_rx_enable_all");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    for (i=0; i<NUM_RX_BUFFS; i++) {
        icp_can_set_rx_enable(handle, i, 1);
    }
    
    return 0;
}

int icp_can_rx_disable_all(icp_can_handle_t handle)
{
    unsigned int i;
    
    DEBUG_OUT("icp_can_rx_disable_all");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    for (i=0; i<NUM_RX_BUFFS; i++) {
        icp_can_set_rx_enable(handle, i, 0);
    }
    
    return 0;
}

int icp_can_tx_enable_all(icp_can_handle_t handle)
{
    unsigned int i;
    
    DEBUG_OUT("icp_can_tx_enable_all");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    for (i=0; i<NUM_TX_BUFFS; i++) {
        icp_can_set_tx_enable(handle, i, 1);
    }
    
    return 0;
}

int icp_can_tx_disable_all(icp_can_handle_t handle)
{
    unsigned int i;

    DEBUG_OUT("icp_can_tx_disable_all");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    for (i=0; i<NUM_TX_BUFFS; i++) {
        icp_can_set_tx_enable(handle, i, 0);
    }
    
    return 0;
}


int icp_can_rx_init_filter(
    icp_can_handle_t    handle,
    unsigned int        buff_num)
{
    int err;
    icp_can_rx_filter_t filter;
    
    DEBUG_OUT("icp_can_rx_init_filter");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    if (buff_num>=NUM_RX_BUFFS) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "buff_num");
        return -1;
    }
    
    /* Set all Rx filters to allow all msgs. */
    filter.amr.id       = ~0;
    filter.amr.id_ext   = ~0;
    filter.amr.rtr      = ~0;
    filter.amr.data     = ~0;
    filter.acr.id       = 0;
    filter.acr.id_ext   = 0;
    filter.acr.rtr      = 0;
    filter.acr.data     = 0;
    filter.num          = buff_num;
    
    err = icp_can_set_rx_filter(handle, &filter);
    
    return err;
}

int icp_can_set_rx_enable(
    icp_can_handle_t     handle, 
    unsigned int         buff_num,
    unsigned int        set)
{
    unsigned int offset;

    DEBUG_OUT("icp_can_set_rx_enable");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    if (buff_num>=NUM_RX_BUFFS) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "buff_num");
        return -1;
    }
    
    /* Enable / disable buffer */
    offset = MM_RX0 + (buff_num * (MM_RX1 - MM_RX0)) + MM_RXCTL;
    can_reg_set(handle, offset,
        MSK_RXCTL_BUFENBL | MSK_RXCTL_WPNL, 
        set ? (MSK_RXCTL_BUFENBL | MSK_RXCTL_WPNL) : 
              (0x0 | MSK_RXCTL_WPNL));
    
    /* Enable / disable interrupt */
    can_reg_set(handle, offset, 
        MSK_RXCTL_INTENBL | MSK_RXCTL_WPNL, 
        set ? (MSK_RXCTL_INTENBL  | MSK_RXCTL_WPNL) : 
              (0x0 | MSK_RXCTL_WPNL));
    
    return 0;
}

int icp_can_get_rx_enable(
    icp_can_handle_t     handle, 
    unsigned int         buff_num,
    unsigned int        *enable)
{
    unsigned int offset;

    DEBUG_OUT("icp_can_get_rx_enable");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    if (buff_num>=NUM_RX_BUFFS) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "buff_num");
        return -1;
    }
        
    /* Enable buffer */
    offset = MM_RX0 + (buff_num * (MM_RX1 - MM_RX0)) + MM_RXCTL;
    can_reg_set(handle, offset, MSK_RXCTL_BUFENBL, MSK_RXCTL_BUFENBL);
    *enable = (can_reg_get(handle, offset) & MSK_RXCTL_BUFENBL) 
                >> BIT_RXCTL_BUFENBL;
    
    return 0;
}

int icp_can_set_tx_enable(
    icp_can_handle_t     handle, 
    unsigned int         buff_num,
    unsigned int        set)
{
    unsigned int offset;
    
    DEBUG_OUT("icp_can_set_tx_enable");

    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    if (buff_num>=NUM_TX_BUFFS) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "buff_num");
        return -1;
    }
    
    /* Enable / disable interrupt */
    offset = MM_TX0 + (buff_num * (MM_TX1 - MM_TX0)) + MM_TXCTL;
    can_reg_set(handle, offset, 
        MSK_TXCTL_INTENBL | MSK_TXCTL_WPN0, 
        set ? (MSK_TXCTL_INTENBL | MSK_TXCTL_WPN0) : 
              (0x0 | MSK_TXCTL_WPN0));
    
    return 0;
}

int icp_can_get_tx_enable(
    icp_can_handle_t     handle, 
    unsigned int         buff_num,
    unsigned int        *enable)
{
    unsigned int offset;

    DEBUG_OUT("icp_can_get_tx_enable");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    if (buff_num>=NUM_TX_BUFFS) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "buff_num");
        return -1;
    }
    
    /* Get interrupt enable */
    offset = MM_TX0 + (buff_num * (MM_TX1 - MM_TX0)) + MM_TXCTL;
    *enable = (can_reg_get(handle, offset) & MSK_TXCTL_INTENBL) 
            >> BIT_TXCTL_INTENBL;
    
    return 0;
}

/*****************************************************************************
 * Returns CAN device's interrupt status; i.e. any ints pending?
 *****************************************************************************/
unsigned int icp_can_int_pending(icp_can_handle_t handle)
{    
    /*DEBUG_OUT("icp_can_int_pending");*/
    unsigned int ret_val;

    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return 0;
    }

    ret_val = can_reg_get(handle, MM_INTSTAT);
    
    return ret_val;
}

/*****************************************************************************
 * Set the baud rate of the CAN device based on recommended timings.
 *****************************************************************************/
int icp_can_set_baud_simple(
    icp_can_handle_t    handle, 
    icp_can_baud_t        baud)
{    
    unsigned int mask = 0;
    unsigned int reg = 0;
    
    DEBUG_OUT("icp_can_set_baud_simple");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    mask = MSK_CFG_BITRATE | MSK_CFG_TSEG1 | MSK_CFG_TSEG2 | MSK_CFG_SJW |
            MSK_CFG_SMPL | MSK_CFG_EDGE;
    
    reg =     (can_rec_timing[baud].cfg_bitrate << BIT_CFG_BITRATE) |
        (can_rec_timing[baud].cfg_tseg1   << BIT_CFG_TSEG1)   |
        (can_rec_timing[baud].cfg_tseg2   << BIT_CFG_TSEG2)   |
        (can_rec_timing[baud].cfg_sjw     << BIT_CFG_SJW)     |
        (can_rec_timing[baud].smpl_mode   << BIT_CFG_SMPL)    |
        (can_rec_timing[baud].edge_mode   << BIT_CFG_EDGE);
            
    can_reg_set(handle, MM_CFG, mask, reg);
    
    return 0;
}

/*****************************************************************************
 * Sets the CAN baud rate set; user specifies all timing parameters.
 *****************************************************************************/
int icp_can_set_baud_custom(
    icp_can_handle_t    handle, 
    icp_can_timing_t    *timing)
{
    unsigned int mask = 0;
    unsigned int reg = 0;
    
    DEBUG_OUT("icp_can_set_baud_custom");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    /* max is MAX_BITRATE */
    if (timing->cfg_bitrate > MAX_BITRATE) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "timing->cfg_bitrate");
        return -1;
    }
    
    /* 0 and 1 not allowed */
    if (timing->cfg_tseg1 < 0x2) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "timing->cfg_tseg1");
        return -1;
    }
    
    /* 0 is not allowed; 1 is allowed if sample_mode is direct */
    if (timing->cfg_tseg2 == 0) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "timing->cfg_tseg2");
        return -1;
    }
    
    if ((timing->cfg_tseg2 == 1) && (timing->smpl_mode != 0)) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "timing->cfg_tseg2");
        return -1;
    }
    
    /* sjw <= tseg1; sjw <= tseg2 */
    if ( (timing->cfg_sjw > timing->cfg_tseg1) || 
        (timing->cfg_sjw > timing->cfg_tseg2) ) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "timing->cfg_sjw");
        return -1;
    }
    
    /* 0 and 1 are the only valid values */
    if (timing->smpl_mode > 1) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "timing->smpl_mode");
        return -1;
    }
    
    /* 0 and 1 are the only valid values */
    if (timing->edge_mode > 1) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "timing->edge_mode");
        return -1;
    }
    
    mask = MSK_CFG_BITRATE | MSK_CFG_TSEG1 | MSK_CFG_TSEG2 | MSK_CFG_SJW |
            MSK_CFG_SMPL | MSK_CFG_EDGE;
    
    reg =     (timing->cfg_bitrate << BIT_CFG_BITRATE) |
        (timing->cfg_tseg1   << BIT_CFG_TSEG1)   |
        (timing->cfg_tseg2   << BIT_CFG_TSEG2)   |
        (timing->cfg_sjw     << BIT_CFG_SJW)     |
        (timing->smpl_mode   << BIT_CFG_SMPL)    |
        (timing->edge_mode   << BIT_CFG_EDGE);
            
    can_reg_set(handle, MM_CFG, mask, reg);
    
    return 0;    
}

/*****************************************************************************
 * Gets the CAN baud rate set.
 *****************************************************************************/
int icp_can_get_baud(
    icp_can_handle_t    handle, 
    icp_can_timing_t    *timing)
{
    unsigned int timing_reg = 0;
    
    DEBUG_OUT("icp_can_get_baud");
    
    if ((!handle) || (!timing)) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle or timing");
        return -1;
    }
    
    timing_reg = can_reg_get(handle, MM_CFG);
    
    timing->cfg_bitrate =  (timing_reg & MSK_CFG_BITRATE) >> BIT_CFG_BITRATE;
    timing->cfg_tseg1 = (timing_reg & MSK_CFG_TSEG1) >> BIT_CFG_TSEG1;
    timing->cfg_tseg2 = (timing_reg & MSK_CFG_TSEG2) >> BIT_CFG_TSEG2;
    timing->cfg_sjw = (timing_reg & MSK_CFG_SJW) >> BIT_CFG_SJW;
    timing->smpl_mode = (timing_reg & MSK_CFG_SMPL) >> BIT_CFG_SMPL;
    timing->edge_mode = (timing_reg & MSK_CFG_EDGE) >> BIT_CFG_EDGE;
    
    return 0;    
}


/*****************************************************************************
 * Sets the Rx filter for a receive buffer.
 *****************************************************************************/
int icp_can_set_rx_filter(
    icp_can_handle_t    handle, 
    icp_can_rx_filter_t    *filter)
{
    unsigned int rx_offset;
    unsigned int offset;
    unsigned int mask;
    unsigned int reg;
    
    DEBUG_OUT("icp_can_set_rx_filter");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }

    /* Find the n'th Rx buffer. */
    rx_offset = MM_RX0 + (filter->num * (MM_RX1 - MM_RX0));
            
    /* AMR */
    offset = rx_offset + MM_RXAMR;
    mask = MSK_RXAMR_RTR | MSK_RXAMR_IDE | MSK_RXAMR_ID;
    reg = ((filter->amr.rtr) ? MSK_RXAMR_RTR : 0) | 
          ((filter->amr.id_ext) ? MSK_RXAMR_IDE : 0);
    
    if (filter->amr.id_ext) {
        reg |= ((filter->amr.id<<BIT_RXAMR_ID) & MSK_RXAMR_ID);
    } else {
        reg |= ((filter->amr.id<<BIT_RXAMR_ID_UPPER) & (MSK_RXAMR_ID_UPPER));
        reg |= 0x3ffff<<BIT_RXAMR_ID; /* 17:0 ext id bits set to don't care */
    }
    can_reg_set(handle, offset, mask, reg);
    
    /* ACR */
    offset = rx_offset + MM_RXACR;
    mask = MSK_RXACR_RTR | MSK_RXACR_IDE | MSK_RXACR_ID;
    reg = ((filter->acr.rtr) ? MSK_RXACR_RTR : 0) | 
          ((filter->acr.id_ext) ? MSK_RXACR_IDE : 0);

    if (filter->acr.id_ext) {
        reg |= ((filter->acr.id<<BIT_RXACR_ID) & MSK_RXACR_ID);
    } else {
        reg |= ((filter->acr.id<<BIT_RXACR_ID_UPPER) & (MSK_RXACR_ID_UPPER));
        reg |= 0x3ffff<<BIT_RXACR_ID; /* 17:0 ext id bits set to don't care */
    }
    can_reg_set(handle, offset, mask, reg);
    
    /* AMR data */
    offset = rx_offset + MM_RXAMRDATA;
    can_reg_set(handle, offset, MSK_RXAMRDATA, 
        filter->amr.data<<BIT_RXAMRDATA);
    
    /* ACR data */
    offset = rx_offset + MM_RXACRDATA;
    can_reg_set(handle, offset, MSK_RXACRDATA, 
        filter->acr.data<<BIT_RXACRDATA);
    
    return 0;
}


int icp_can_set_rx_buffer_link(
    icp_can_handle_t    handle,
    unsigned int        buffer_num,
    unsigned int         set)
{
    unsigned int rx_offset;
    unsigned int offset;
    unsigned int reg;
    
    DEBUG_OUT("icp_can_set_rx_buffer_link");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    /* Find the n'th Rx buffer. */
    rx_offset = MM_RX0 + (buffer_num * (MM_RX1 - MM_RX0));
    
    /* Message Control */
    offset = rx_offset + MM_RXCTL;
    reg = can_reg_get(handle, offset);
    
    can_reg_set(handle, offset, 
        MSK_RXCTL_WPNL | MSK_RXCTL_LNKFLG, 
        (set) ? (MSK_RXCTL_WPNL | MSK_RXCTL_LNKFLG) : (MSK_RXCTL_WPNL));
    
    return 0;
}

int icp_can_get_rx_buffer_link(
    icp_can_handle_t    handle,
    unsigned int        buffer_num,
    unsigned int         *link)
{
    unsigned int rx_offset;
    unsigned int offset;
    
    DEBUG_OUT("icp_can_get_rx_buffer_link");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    /* Find the n'th Rx buffer. */
    rx_offset = MM_RX0 + (buffer_num * (MM_RX1 - MM_RX0));
    
    /* Message Control */
    offset = rx_offset + MM_RXCTL;
    
    *link = (can_reg_get(handle, offset) & MSK_RXCTL_LNKFLG) 
            >> BIT_RXCTL_LNKFLG;
    
    return 0;
}

/*****************************************************************************
 * Gets the Rx filter for a receive buffer.
 *****************************************************************************/
int icp_can_get_rx_filter(
    icp_can_handle_t    handle, 
    icp_can_rx_filter_t    *filter)
{
    unsigned int rx_offset;
    unsigned int offset;
    unsigned int reg;
    
    DEBUG_OUT("icp_can_get_rx_filter");
    
    if ((!handle) || (!filter)) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle or filter");
        return -1;
    }
    
    /* Find the n'th Rx buffer. */
    rx_offset = MM_RX0 + (filter->num * (MM_RX1 - MM_RX0));
            
    /* AMR */
    offset = rx_offset + MM_RXAMR;
    reg = can_reg_get(handle, offset);
    filter->amr.rtr = (reg & MSK_RXAMR_RTR) >> BIT_RXAMR_RTR;
    filter->amr.id_ext = (reg & MSK_RXAMR_IDE) >> BIT_RXAMR_IDE;
    
    if (!(filter->amr.id_ext)) {
        filter->amr.id = (reg & MSK_RXAMR_ID) >> BIT_RXAMR_ID_UPPER;
    } else {
        filter->amr.id = (reg & MSK_RXAMR_ID) >> BIT_RXAMR_ID;
    }
    
    /* ACR */
    offset = rx_offset + MM_RXACR;                
    reg = can_reg_get(handle, offset);
    filter->acr.rtr = (reg & MSK_RXACR_RTR) >> BIT_RXACR_RTR;
    filter->acr.id_ext = (reg & MSK_RXACR_IDE) >> BIT_RXACR_IDE;
    
    if (!(filter->acr.id_ext)) {
        filter->acr.id = (reg & MSK_RXACR_ID) >> BIT_RXACR_ID_UPPER;
    } else {
        filter->acr.id = (reg & MSK_RXACR_ID) >> BIT_RXACR_ID;
    }
    
    /* AMR data */
    offset = rx_offset + MM_RXAMRDATA;
    reg = can_reg_get(handle, offset);
    filter->amr.data = (reg & MSK_RXAMRDATA) >> BIT_RXAMRDATA;
    
    /* ACR data */
    offset = rx_offset + MM_RXACRDATA;
    reg = can_reg_get(handle, offset);
    filter->acr.data = (reg & MSK_RXACRDATA) >> BIT_RXACRDATA;
    
    return 0;
}


/*****************************************************************************
 * Open the CAN device for use. Reset the CAN, enable interrupts and start.
 *****************************************************************************/
int icp_can_open(
    icp_can_handle_t    handle, 
    icp_can_listen_mode_t    listen,
    icp_can_arbiter_t      arbiter)
{
    int err;
    int i;
    
    DEBUG_OUT("icp_can_open");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    err = icp_can_clear_buffers(handle);
    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "clear buffers");
        return err;
    }
    
    for (i=0; i<NUM_RX_BUFFS; i++) {
                err = icp_can_rx_init_filter(handle, i);
                if (err) {
                        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, 
                            "init rx filters");
                        return err;
                }
        }
    
    err = icp_can_set_listen_mode(handle, ICP_CAN_ACTIVE);
    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "active mode");
        return err;
    }
    
    err = icp_can_set_arbiter_mode(handle, arbiter);
    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "set arbiter");
        return err;
    }

    err = icp_can_rx_enable_all(handle);
    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "set rx enable all");
        return err;
    }
    
    err = icp_can_tx_enable_all(handle);
    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "set tx enable all");
        return err;
    }
    
    err = icp_can_set_int_enables(handle, ICP_CAN_ALL);
    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "set int enables");
        return err;
    }
    
    err = icp_can_set_int_enables(handle, ICP_CAN_ENABLE);

    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "enable ints");
        return err;
    }

    return 0;
}

/*****************************************************************************
 * Close the CAN device. Opposite of Open.
 *****************************************************************************/
int icp_can_release(
    icp_can_handle_t    handle)
{
    int err;
    
    DEBUG_OUT("icp_can_release");

    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }

    err = icp_can_set_run_mode(handle, ICP_CAN_STOP);
    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "stop mode");
    }
    
    err = icp_can_set_int_enables(handle, ICP_CAN_DISABLE);
    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "disable ints");
    }
    
    err = icp_can_rx_disable_all(handle);
    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "set rx enable all");
        return err;
    }
    
    err = icp_can_tx_disable_all(handle);
    if (err) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_OPERATION, "set tx enable all");
        return err;
    }

    return 0;
}

/*****************************************************************************
 * Clears an interrupt.
 *****************************************************************************/
void icp_can_int_clr(
    icp_can_handle_t    handle, 
    unsigned int        mask)
{
    DEBUG_OUT("icp_can_int_clr");
    
    /* Interrupts are cleared by setting a one 
       to the interrupt status bits. */

    if (handle) {
        can_reg_set(handle, MM_INTSTAT, mask, mask);
    } else {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
    }
}

/*****************************************************************************
 * Transmit a CAN message.
 *****************************************************************************/
int icp_can_msg_tx(
    icp_can_handle_t    handle, 
    icp_can_msg_t        *msg)
{
    unsigned int offset;
    unsigned int txreq;
    unsigned int data;
    unsigned int buffer_status;
    unsigned int tx_buffer_avail;
    icp_can_run_mode_t    run_mode;
    
    DEBUG_OUT("icp_can_msg_tx");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    icp_can_get_run_mode(handle, &run_mode);

    if (run_mode!=ICP_CAN_RUN) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_DEVICE,"CAN stopped on transmit attempt");
        return -1;
    }

    buffer_status = icp_can_get_buffer_status(handle);
    buffer_status = (buffer_status & MSK_BS_TXALL) >> BIT_BS_TXMSG0;
    
    tx_buffer_avail = 0;
    while (buffer_status & 0x1) {
        buffer_status = buffer_status >> 0x1;
        tx_buffer_avail++;
    }
    
    if (tx_buffer_avail>=NUM_TX_BUFFS) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_DEVICE, "transmit buffer full");
        return -1;
    };
    
    offset = MM_TX0 + (tx_buffer_avail * (MM_TX1 - MM_TX0));
    txreq = can_reg_get(handle, offset + MM_TXCMD) & MSK_TXCMD_REQ;
    if (!txreq) {
        /* RTR, IDE, DLC */
        can_reg_set(handle, 
            offset + MM_TXCTL, 
            MSK_TXCTL_RTR | 
            MSK_TXCTL_IDE | 
            MSK_TXCTL_DLC | 
            MSK_TXCTL_WPN1, 
            ((msg->rtr) ? MSK_TXCTL_RTR : 0) |
            ((msg->ide) ? MSK_TXCTL_IDE : 0) |
            ((msg->dlc)<<BIT_TXCTL_DLC) |
            MSK_TXCTL_WPN1);

        /* ID */
        if (msg->ide) {
            data = msg->id << BIT_TXID;
        } else {
            data = msg->id << BIT_TXID_UPPER;
        }
        can_reg_set(handle, offset + MM_TXID, 0xfffffff8, data);
        
        /* Data */
        data = (msg->data[0x0]<<0x18) | (msg->data[1]<<0x10) |
            (msg->data[0x2]<<0x8) | (msg->data[0x3]);
        can_reg_set_fast(handle, offset + MM_TXDATAH, data);
        
        data = (msg->data[0x4]<<0x18) | (msg->data[0x5]<<0x10) |
            (msg->data[0x6]<<0x8) | (msg->data[0x7]);
        can_reg_set_fast(handle, offset + MM_TXDATAL, data);
                
        /* Transmit the message */
        can_reg_set(handle, offset + MM_TXCMD, MSK_TXCMD_REQ, 
                MSK_TXCMD_REQ);
    
        return 0;
    }
    
    return -1;
}

/*****************************************************************************
 * Resets the CAN Rx/Tx buffers.
 *****************************************************************************/
int icp_can_clear_buffers(icp_can_handle_t handle)
{    
    unsigned int base;
    unsigned int offset;    
    unsigned int buff_num;
    
    DEBUG_OUT("icp_can_clear_buffers");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }
    
    /* Zero out Tx Buffers */
    for (buff_num=0; buff_num<NUM_TX_BUFFS; buff_num++) {
        base = MM_TX0 + (buff_num * (MM_TX1 - MM_TX0));
        
        /* Control Register */
        offset = base;
        can_reg_set_fast(handle, offset, 0x00800008);
        
        /* Message Id */
        offset = base + MM_TXID;
        can_reg_set_fast(handle, offset, 0x00000000);
        
        /* Data Hi */
        offset = base + MM_TXDATAH;
        can_reg_set_fast(handle, offset, 0x00000000);
        
        /* Data Lo */
        offset = base + MM_TXDATAL;
        can_reg_set_fast(handle, offset, 0x00000000);
    }
    
    /* Zero out Rx Buffers */
    for (buff_num=0; buff_num<NUM_RX_BUFFS; buff_num++) {
        base = MM_RX0 + (buff_num * (MM_RX1 - MM_RX0));
        
        /* Control Register */
        offset = base;
        /* can_reg_set_fast(handle, offset, 0x00800080); */
        if (buff_num<(NUM_RX_BUFFS-1)) {
            /* enable link on everyone else */
            can_reg_set_fast(handle, offset, 0x008000c0);
        } else {
            /* don't enable link on last buffer */
            can_reg_set_fast(handle, offset, 0x00800080);
        }
        
        /* Message Id */
        offset = base + MM_RXID;
        can_reg_set_fast(handle, offset, 0x00000000);
        
        /* Data Hi */
        offset = base + MM_RXDATAH;
        can_reg_set_fast(handle, offset, 0x00000000);
        
        /* Data Lo */
        offset = base + MM_RXDATAL;
        can_reg_set_fast(handle, offset, 0x00000000);
        
        /* AMR */
        offset = base + MM_RXAMR;
        can_reg_set_fast(handle, offset, 0x00000000);
        
        /* ACR */
        offset = base + MM_RXACR;
        can_reg_set_fast(handle, offset, 0x00000000);
        
        /* AMR Data */
        offset = base + MM_RXAMRDATA;
        can_reg_set_fast(handle, offset, 0x00000000);
        
        /* ACR Data */
        offset = base + MM_RXACRDATA;
        can_reg_set_fast(handle, offset, 0x00000000);
    }
        
    return 0;
}

/*****************************************************************************
 * Get the Rx and Tx buffer status.
 *****************************************************************************/
unsigned int icp_can_get_buffer_status(
    icp_can_handle_t    handle)
{
    DEBUG_OUT("icp_can_get_buffer_status");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return 0;
    }

    return can_reg_get(handle, MM_BUFFSTAT);
}

/*****************************************************************************
 * Get a pending Rx message.
 *****************************************************************************/
int icp_can_rx_dequeue(
    icp_can_handle_t    handle,
    icp_can_msg_t        *msg,
    unsigned int        buff_num
    )
{
    unsigned int i=0;
    unsigned int offset;
    unsigned int data;
    unsigned int tmp;
    char *data_tmp = msg->data;

    DEBUG_OUT("icp_can_rx_dequeue");
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }

    /* Message base */
    offset = MM_RX0 + (buff_num * (MM_RX1 - MM_RX0));    

    /* DLC, RTR, IDE */
    tmp = can_reg_get(handle, offset + MM_RXCTL);
    msg->dlc = (tmp & MSK_RXCTL_DLC) >> BIT_RXCTL_DLC;
    msg->rtr = (tmp & MSK_RXCTL_RTR) ? 1 : 0;
    msg->ide = (tmp & MSK_RXCTL_IDE) ? 1 : 0;

    /* ID */    
    if (msg->ide) {
        msg->id = can_reg_get(handle, offset + MM_RXID) 
                    >> BIT_RXID;
    } else {
        msg->id = can_reg_get(handle, offset + MM_RXID)
                    >> BIT_RXID_UPPER;
    }

    /* Data */
    switch (msg->dlc) {
        case 0x0:
            break;
        case 0x1:
            /* Data high */
            data = can_reg_get(handle, offset + MM_RXDATAH);
            data_tmp[0x0] = (data & 0xff000000) >> 0x18;
            break;
        case 0x2:
            /* Data high */
            data = can_reg_get(handle, offset + MM_RXDATAH);
            data_tmp[0x0] = (data & 0xff000000) >> 0x18;
            data_tmp[0x1] = (data & 0x00ff0000) >> 0x10;
            break;
        case 0x3:
            /* Data high */
            data = can_reg_get(handle, offset + MM_RXDATAH);
            data_tmp[0x0] = (data & 0xff000000) >> 0x18;
            data_tmp[0x1] = (data & 0x00ff0000) >> 0x10;
            data_tmp[0x2] = (data & 0x0000ff00) >> 0x8;
            break;
        case 0x4:
            /* Data high */
            data = can_reg_get(handle, offset + MM_RXDATAH);
            data_tmp[0x0] = (data & 0xff000000) >> 0x18;
            data_tmp[0x1] = (data & 0x00ff0000) >> 0x10;
            data_tmp[0x2] = (data & 0x0000ff00) >> 0x8;
            data_tmp[0x3] = (data & 0x000000ff);
            break;
        case 0x5:
            /* Data high */
            data = can_reg_get(handle, offset + MM_RXDATAH);
            data_tmp[0x0] = (data & 0xff000000) >> 0x18;
            data_tmp[0x1] = (data & 0x00ff0000) >> 0x10;
            data_tmp[0x2] = (data & 0x0000ff00) >> 0x8;
            data_tmp[0x3] = (data & 0x000000ff);
    
            /* Data low */
            data = can_reg_get(handle, offset + MM_RXDATAL);
            data_tmp[0x4] = (data & 0xff000000) >> 0x18; 
            break;
        case 0x6:
            /* Data high */
            data = can_reg_get(handle, offset + MM_RXDATAH);
            data_tmp[0x0] = (data & 0xff000000) >> 0x18;
            data_tmp[0x1] = (data & 0x00ff0000) >> 0x10;
            data_tmp[0x2] = (data & 0x0000ff00) >> 0x8;
            data_tmp[0x3] = (data & 0x000000ff);
    
            /* Data low */
            data = can_reg_get(handle, offset + MM_RXDATAL);
            data_tmp[0x4] = (data & 0xff000000) >> 0x18;
            data_tmp[0x5] = (data & 0x00ff0000) >> 0x10;
            break;
        case 0x7:
            /* Data high */
            data = can_reg_get(handle, offset + MM_RXDATAH);
            data_tmp[0x0] = (data & 0xff000000) >> 0x18;
            data_tmp[0x1] = (data & 0x00ff0000) >> 0x10;
            data_tmp[0x2] = (data & 0x0000ff00) >> 0x8;
            data_tmp[0x3] = (data & 0x000000ff);
    
            /* Data low */
            data = can_reg_get(handle, offset + MM_RXDATAL);
            data_tmp[0x4] = (data & 0xff000000) >> 0x18;
            data_tmp[0x5] = (data & 0x00ff0000) >> 0x10;
            data_tmp[0x6] = (data & 0x0000ff00) >> 0x8;
            break;
        case 0x8:
        default:
            /* Data high */
            data = can_reg_get(handle, offset + MM_RXDATAH);
            data_tmp[0x0] = (data & 0xff000000) >> 0x18;
            data_tmp[0x1] = (data & 0x00ff0000) >> 0x10;
            data_tmp[0x2] = (data & 0x0000ff00) >> 0x8;
            data_tmp[0x3] = (data & 0x000000ff);
    
            /* Data low */
            data = can_reg_get(handle, offset + MM_RXDATAL);
            data_tmp[0x4] = (data & 0xff000000) >> 0x18;
            data_tmp[0x5] = (data & 0x00ff0000) >> 0x10;
            data_tmp[0x6] = (data & 0x0000ff00) >> 0x8;
            data_tmp[0x7] = (data & 0x000000ff);
            break;
    }

    /* Clear unused Data bytes */
    for (i=msg->dlc; i<ICP_CAN_MSG_DATA_LEN; i++) {
        msg->data[i] = 0x00;
    }

    /* Clear MsgAv since we got the message */
    can_reg_set(handle, offset + MM_RXCMD, MSK_RXCMD_MSGAV, MSK_RXCMD_MSGAV);

    return 0;
}

/*****************************************************************************
 * Log stat/error messages.
 *****************************************************************************/ 
void icp_can_log_message(unsigned int status)
{
    DEBUG_OUT("icp_can_log_message");
    
    if (status & MSK_IS_OVRLOAD) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_DEVICE, "overload");
    } else if (status & MSK_IS_BITERR) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_DEVICE, "bit error");
    } else if (status & MSK_IS_STUFFERR) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_DEVICE, "stuff error");
    } else if (status & MSK_IS_ACKERR) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_DEVICE, "ack error");
    } else if (status & MSK_IS_FORMERR) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_DEVICE, "form error");
    } else if (status & MSK_IS_CRCERR) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_DEVICE, "CRC error");
    } else if (status & MSK_IS_BUSOFF) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_DEVICE, "bus off");
    } else if (status & MSK_IS_RXMSGLOSS) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_DEVICE, "recv msg loss");
    }
}

/*****************************************************************************
 * Get stat/error messages.
 *****************************************************************************/
int icp_can_get_error_stats(
    icp_can_handle_t     handle,
    icp_can_error_t        *error)
{
    unsigned int reg;
    
    DEBUG_OUT("icp_can_get_error_stats");
    
    if ((!handle) || (!error)) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle or timing");
        return -1;
    }

    reg = can_reg_get(handle, MM_ERRSTAT);
    
    error->rxgte96 = reg & MSK_ES_RXGTE96 >> BIT_ES_RXGTE96;
    error->txgte96 = reg & MSK_ES_TXGTE96 >> BIT_ES_TXGTE96;
    error->error_stat = reg & MSK_ES_ERRSTAT >> BIT_ES_ERRSTAT;
    error->rx_err_cnt = reg & MSK_ES_RXERRCNT >> BIT_ES_RXERRCNT;
    error->tx_err_cnt = reg & MSK_ES_TXERRCNT >> BIT_ES_TXERRCNT;
    
    return 0;
}

/*****************************************************************************
 * Output CAN device registers.
 *****************************************************************************/
int DEBUG_DUMP(icp_can_handle_t handle, char *info, unsigned int level)
{
    unsigned int base;
    unsigned int buff_num;

    DEBUG_OUT(info);
    
    if (!handle) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_PARAM, "handle");
        return -1;
    }

    if (level&0x01) {
        /* Dump the Control, Config and Status registers only. */
        CAN_PRINT_DEBUG("CANDBG: IS=0x%08x IE=0x%08x BS=0x%08x ES=0x%08x\n",
            can_reg_get(handle, MM_INTSTAT), can_reg_get(handle, MM_INTENBL), 
            can_reg_get(handle, MM_BUFFSTAT), can_reg_get(handle, MM_ERRSTAT));

        CAN_PRINT_DEBUG("CANDBG: CM=0x%08x CF=0x%08x\n", 
            can_reg_get(handle, MM_CMD), can_reg_get(handle, MM_CFG));
    }

    if (level&0x02) {
        /* Dump the Transmit registers only. */
        for (buff_num=0; buff_num<NUM_TX_BUFFS; buff_num++) {    
            base = MM_TX0 + (buff_num * (MM_TX1 - MM_TX0));
            CAN_PRINT_DEBUG(
                "CANDBG: Tx%02d CM=0x%08x ID=0x%08x DH=0x%08x DL=0x%08x\n", 
                buff_num, can_reg_get(handle, base), 
                can_reg_get(handle, base + MM_TXID),
                can_reg_get(handle, base + MM_TXDATAH), 
                can_reg_get(handle, base + MM_TXDATAL));
        }

    }

    if (level&0x04) {
        for (buff_num=0; buff_num<NUM_RX_BUFFS; buff_num++) {
            base = MM_RX0 + (buff_num * (MM_RX1 - MM_RX0));
            CAN_PRINT_DEBUG(
                "CANDBG: Rx%02d CM=0x%08x ID=0x%08x DH=0x%08x DL=0x%08x\n", 
                buff_num, can_reg_get(handle, base + MM_RXCMD), 
                can_reg_get(handle, base + MM_RXID), 
                can_reg_get(handle, base + MM_RXDATAH), 
                can_reg_get(handle, base + MM_RXDATAL));

            CAN_PRINT_DEBUG("CANDBG:      AM=0x%08x AC=0x%08x AH=0x%08x \
                AL=0x%08x\n", 
                can_reg_get(handle, base + MM_RXAMR), 
                can_reg_get(handle, base + MM_RXACR), 
                can_reg_get(handle, base + MM_RXAMRDATA), 
                can_reg_get(handle, base + MM_RXACRDATA));
        }
    }
    
    return 0;
}

