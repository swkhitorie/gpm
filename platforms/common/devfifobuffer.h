#ifndef __DEVFIFO_BUFFER_H_
#define __DEVFIFO_BUFFER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

typedef enum __devfifo_bufalloc {
	DEVFIFO_ALLOC,
	DEVFIFO_STATIC
} devfifo_buffalloc_t;

typedef struct __devfifobuffer {
	uint8_t *buf;
	uint16_t capacity;
	uint16_t size;
	uint16_t idx_out;
	uint16_t idx_in;
	uint16_t overflow;
} devfifobuffer_t;

uint16_t devfifo_size(devfifobuffer_t *obj);
uint16_t devfifo_free(devfifobuffer_t *obj);
uint16_t devfifo_overflow(devfifobuffer_t *obj);
bool devfifo_is_empty(devfifobuffer_t *obj);
bool devfifo_is_full(devfifobuffer_t *obj);
void devfifo_reset(devfifobuffer_t *obj);
bool devfifo_init(devfifobuffer_t *obj, uint8_t *psource, uint16_t capacity, devfifo_buffalloc_t way);

uint16_t devfifo_write(devfifobuffer_t *obj, const uint8_t *p, uint16_t len);
uint16_t devfifo_read(devfifobuffer_t *obj, uint8_t *p, uint16_t len);
uint16_t devfifo_query(devfifobuffer_t *obj, uint16_t offset, uint8_t *p, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
