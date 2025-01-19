/*******************************************************************************
 * %LICENSE_PROPRIETARY%
 * <COPYRIGHT_TAG>
 *******************************************************************************/

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

#include <ntddk.h>

#define CAN_REG_WRITE(addr, val)	WRITE_REGISTER_ULONG((addr), (val));
#define CAN_REG_READ(addr)		READ_REGISTER_ULONG((addr));

#define CAN_POOL_TAG           ' NAC'

#define CAN_PRINT_DEBUG			DbgPrint

#ifdef ICP_CAN_DEBUG
#define DEBUG_OUT(S)                    DbgPrint("CAN_DEBUG: %s\n", S)
#else
#define DEBUG_OUT(S)
#endif


#define CAN_MEM_ALLOC(size)		ExAllocatePoolWithTag(NonPagedPool, (size), CAN_POOL_TAG);
#define CAN_MEM_FREE(ptr)		ExFreePool((ptr));


#endif /* __CAN_PORT_H__ */
