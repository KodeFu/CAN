/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/
/**************************************************************************
 * @ingroup CAN_FIFO_GENERAL
 *
 * @file can_fifo.c
 *
 * @description
 *   
 **************************************************************************/

#include "icp_can.h"
#include "can_fifo.h"

/*****************************************************************************
 * A FIFO element is a CAN msg.
 *****************************************************************************/
typedef struct can_fifo_item {
    icp_can_msg_t        msg;
    struct can_fifo_item    *next;
} can_fifo_item_t;

/*****************************************************************************
 * The FIFO.
 *****************************************************************************/
typedef struct can_fifo {
    can_fifo_item_t *head;
    can_fifo_item_t *tail;
    unsigned int size;
} can_fifo_t;

/*****************************************************************************
 * Check if FIFO is empty.
 *****************************************************************************/
int can_fifo_empty(icp_can_handle_t handle)
{
    int ret_val;
    can_fifo_t *f = (can_fifo_t *) handle;

    ret_val = (f->head==f->tail);

    return ret_val;
}

/*****************************************************************************
 * Check if FIFO is full.
 *****************************************************************************/
int can_fifo_full(icp_can_handle_t handle)
{
    int ret_val;
    can_fifo_t *f = (can_fifo_t *) handle;

    ret_val = (f->head->next==f->tail);

    return ret_val;
}

/*****************************************************************************
 * Create a CAN messsage FIFO of size specifified.
 *****************************************************************************/
icp_can_handle_t can_fifo_create(unsigned int num_nodes)
{
    unsigned int i;
    can_fifo_item_t  *curr;
    can_fifo_t *f;
    
    f = (can_fifo_t *) CAN_MEM_ALLOC(sizeof(can_fifo_t));
    
    if (!f) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_ALLOC, "msg queue");
        return (icp_can_handle_t) 0;
    }
    
    f->head = (can_fifo_item_t *) CAN_MEM_ALLOC(sizeof(can_fifo_item_t));

    f->tail = f->head;
    
    if (!(f->head)) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_ALLOC, "msg queue head");
        CAN_MEM_FREE(f);
        return (icp_can_handle_t) 0;
    }
    curr = f->head;
    
    for (i=1; i<num_nodes; i++) {
        curr->next = (can_fifo_item_t *) CAN_MEM_ALLOC(sizeof(can_fifo_item_t));

        if (!(curr->next)) {
            CAN_PRINT_DEBUG(ICP_CAN_ERR_ALLOC, "msg queue node");
            f->size = i-1; 
            CAN_MEM_FREE(f);
            return (icp_can_handle_t) 0;
        }
        
        curr = curr->next;
    }
    
    curr->next = f->head;
    f->size = num_nodes;
    
    return (icp_can_handle_t) f;
}

/*****************************************************************************
 * Delete the FIFO.
 *****************************************************************************/
void can_fifo_destroy(icp_can_handle_t handle)
{
    unsigned int i;
    can_fifo_item_t *curr;
    can_fifo_item_t *next;
    can_fifo_t *f = (can_fifo_t *) handle;
        
    if (handle) {
        curr = f->head;
        next = curr->next;
        
        for (i=0; i<f->size; i++) {
            if (!curr) {
                CAN_PRINT_DEBUG(ICP_CAN_ERR_FREE, 
                    "msg queue node");
            }
            CAN_MEM_FREE(curr);
            curr = next;
            next = (can_fifo_item_t *) curr->next;
        }
        
        CAN_MEM_FREE(f);
    }
}

/*****************************************************************************
 * Get the first element of the FIFO.
 *****************************************************************************/
int can_fifo_get(
    icp_can_handle_t    handle, 
    icp_can_msg_t        *msg)
{
    int i;
    can_fifo_t *f = (can_fifo_t *) handle;
    icp_can_msg_t msg_tmp = f->tail->msg;
    
    if ((!handle) || (!msg)) {
        return -1;
    }
    
    if (f->head==f->tail) {
        /*CAN_PRINT_DEBUG(ICP_CAN_ERR_QUEUE_EMPTY);*/
        return -1;
    }
        
    msg->ide = msg_tmp.ide;
    msg->id  = msg_tmp.id;
    msg->dlc = msg_tmp.dlc;
    msg->rtr = msg_tmp.rtr;
    
    for (i=0; i<ICP_CAN_MSG_DATA_LEN; i++) {
        msg->data[i] = msg_tmp.data[i];
    }
    
    f->tail = f->tail->next;
    return 0;
}

/*****************************************************************************
 * Put a message into the FIFO.
 *****************************************************************************/
int can_fifo_put(
    icp_can_handle_t    handle, 
    icp_can_msg_t        *msg)
{
    int i;
    can_fifo_t *f = (can_fifo_t *) handle;
    icp_can_msg_t *msg_tmp = &(f->head->msg);
    
    if ((!handle) || (!msg)) {
        return -1;
    }
    
    if (f->head->next==f->tail) {
        CAN_PRINT_DEBUG(ICP_CAN_ERR_QUEUE_FULL);
        return -1;
    }
    
    msg_tmp->ide = msg->ide;
    msg_tmp->rtr = msg->rtr;
    msg_tmp->id  = msg->id;
    msg_tmp->dlc = msg->dlc;
    
    for (i=0; i<ICP_CAN_MSG_DATA_LEN; i++) {
        msg_tmp->data[i] = msg->data[i];
    }
    
    f->head = f->head->next;
    return 0;
}
