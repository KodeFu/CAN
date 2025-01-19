/*****************************************************************************
 * %LICENSE_DUAL%
 * <COPYRIGHT_TAG>
 *****************************************************************************/

/*****************************************************************************
 * can_port.h
 *
 * Common Driver OS Portability Enabling Module
 *
 * Description
 *   This is an internal API that intended as a lightweight common 
 *   portability layer that allows driver versions to reduce their 
 *   OS dependent code. 
 *
 *****************************************************************************/

#ifndef __CAN_PORT_H__
#define __CAN_PORT_H__

#include <linux/slab.h>
#include <linux/spinlock.h>
#include <asm/io.h>

#define CAN_REG_WRITE(addr, val)    writel((val), (addr));
#define CAN_REG_READ(addr)      	readl((addr));

#define CAN_PRINT_DEBUG				printk

#ifdef ICP_CAN_DEBUG
#define DEBUG_OUT(S)				printk("CAN_DEBUG: %s\n", S)
#else
#define DEBUG_OUT(S)
#endif


#define CAN_MEM_ALLOC(size)			kmalloc((size), GFP_KERNEL);
#define CAN_MEM_FREE(ptr)			kfree((ptr));

#endif /* __CAN_PORT_H__ */
