
#ifndef __MMU_RTX_H_
#define __MMU_RTX_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef NULL
	#define NULL (0)
#endif
	
// memory pool header
typedef struct {
    uint32_t size;    /*<! memory pool size */
    uint32_t used;    /*<! used memory */
} mem_head_t;

// memory block header
typedef struct mem_block_s {
    struct mem_block_s *next;     /*<! next memory Block in list */
    uint32_t            info;     /*<! block info or max used memory (in last block) */
} mem_block_t;

//  memory block info: length = <31:2>:'00', type = <1:0>
#define MB_INFO_LEN_MASK        0xFFFFFFFCU
#define MB_INFO_TYPE_MASK       0x00000003U

extern uint32_t rtx_mem_init(void *mem, uint32_t size);
extern void *rtx_mem_alloc(void *mem, uint32_t size, uint32_t type);
extern uint32_t rtx_mem_free(void *mem, void *block);

#ifdef __cplusplus
}
#endif  // __cplusplus

#endif  // rtx_mem_H
