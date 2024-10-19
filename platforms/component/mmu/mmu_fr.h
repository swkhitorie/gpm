#ifndef __MMU_FR_H_
#define __MMU_FR_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define	portBYTE_ALIGNMENT			(8)
#define	portBYTE_ALIGNMENT_MASK		(0x0007)	
#ifndef pdFALSE
    #define pdFALSE (0)
#endif
#ifndef pdTRUE
    #define pdTRUE 	(1)
#endif

#ifndef NULL
	#define NULL (0)
#endif

#ifndef portPOINTER_SIZE_TYPE
    #define portPOINTER_SIZE_TYPE uint32_t
#endif

typedef long BaseType_t;	
typedef unsigned int size_t;

extern void *rtos_mem1_malloc( size_t xWantedSize );
extern void rtos_mem1_heap_init( void *mem, size_t size );
extern size_t rtos_mem1_get_freeheapsize( void );
extern void rtos_mem1_free( void *pv );

extern void *rtos_mem2_malloc( size_t xWantedSize );
extern void rtos_mem2_heap_init( void *mem, size_t size );
extern size_t rtos_mem2_get_freeheapsize( void );
extern size_t rtos_mem2_get_minimumEverfreeheapSize( void );
extern void rtos_mem2_free( void *pv );

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // rtos_mem_H

