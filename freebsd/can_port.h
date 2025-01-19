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

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/param.h>
#include <sys/malloc.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <vm/vm.h>
#include <vm/vm_kern.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>

#define CAN_REG_WRITE(addr, val)	writel((addr), (val));
#define CAN_REG_READ(addr)		readl((addr));

#define CAN_PRINT_DEBUG			uprintf

#ifdef ICP_CAN_DEBUG
#define DEBUG_OUT(S)                    uprintf("CAN_DEBUG: %s\n", S)
#else
#define DEBUG_OUT(S)
#endif


#define CAN_MEM_ALLOC(size)		malloc((size), M_TEMP, (M_WAITOK | M_ZERO));
#define CAN_MEM_FREE(ptr)		free((ptr), M_TEMP);

#endif /* __CAN_PORT_H__ */
