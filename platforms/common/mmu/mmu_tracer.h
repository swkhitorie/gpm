#ifndef __MMU_TRACER_H_
#define __MMU_TRACER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MMU_TRACER_BLOCK_LEN (100)

typedef struct __mmu_tracer
{
    void *block_addr;
    uint32_t block_len;
} mmu_tracer_t;

void mmu_tracerecord(void *addr, uint32_t len);
void mmu_traceremove(void *addr);
void mmu_traceinfo(uint16_t idx, void **addr, uint32_t *len);

#ifdef __cplusplus
}
#endif

#endif
