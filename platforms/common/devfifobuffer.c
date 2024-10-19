#include "devfifobuffer.h"

uint16_t devfifo_size(devfifobuffer_t *obj) { return obj->size; }
uint16_t devfifo_free(devfifobuffer_t *obj) { return obj->capacity - obj->size; }
uint16_t devfifo_overflow(devfifobuffer_t *obj) { return obj->overflow; }
bool devfifo_is_empty(devfifobuffer_t *obj) { return (obj->size == 0); }
bool devfifo_is_full(devfifobuffer_t *obj) { return (obj->size == obj->capacity); }
bool devfifo_init(devfifobuffer_t *obj, uint8_t *psource, uint16_t capacity, 
	devfifo_buffalloc_t way)
{
	obj->size = 0;
	obj->idx_out = 0;
	obj->idx_in = 0;
	obj->overflow = 0;

	switch (way) {
	case DEVFIFO_ALLOC:
		obj->buf = (uint8_t *)malloc(sizeof(uint8_t) * capacity);
		obj->capacity = capacity;
		break;
	case DEVFIFO_STATIC:
		obj->buf = psource;
		obj->capacity = capacity;
		break;
	}

	return (obj->buf != NULL);
}

void devfifo_reset(devfifobuffer_t *obj)
{
	obj->size = 0;
	obj->idx_out = 0;
	obj->idx_in = 0;
	obj->overflow = 0;	
}


uint16_t devfifo_write(devfifobuffer_t *obj, const uint8_t *p, uint16_t len)
{
	uint16_t i;
	uint16_t write_len = 0;
	uint16_t fifo_res_size = 0;

	fifo_res_size = obj->capacity - devfifo_size(obj);
	if (fifo_res_size >= len) {
		write_len = len;
	} else {
		write_len = fifo_res_size;
		obj->overflow = len - fifo_res_size;
	}

	for (i = 0; i < write_len; i++) {
		obj->buf[obj->idx_in] = p[i];
		obj->idx_in++;
		if (obj->idx_in >= obj->capacity) {
			obj->idx_in -= obj->capacity;
		}
	}

	obj->size += write_len;
	return write_len;
}

uint16_t devfifo_read(devfifobuffer_t *obj, uint8_t *p, uint16_t len)
{
	uint16_t i;
	uint16_t read_len = 0;

	if (devfifo_size(obj) >= len) {
		read_len = len;
	} else {
		read_len = devfifo_size(obj);
	}

	for (i = 0; i < read_len; i++) {
		p[i] = obj->buf[obj->idx_out];
		obj->idx_out++;
		if (obj->idx_out >= obj->capacity) {
			obj->idx_out -= obj->capacity;
		}
	}

	obj->size -= read_len;
	return read_len;
}

uint16_t devfifo_query(devfifobuffer_t *obj, uint16_t offset, uint8_t *p, uint16_t len)
{
	uint16_t i;
	uint16_t query_len = 0;
	uint16_t query_idx = obj->idx_out + offset;

	if (devfifo_size(obj) >= (offset + len)) {
		query_len = len;
	} else {
		query_len = devfifo_size(obj);
	}

	for (i = 0; i < query_len; i++) {
		p[i] = obj->buf[query_idx];
		query_idx++;
		if (query_idx >= obj->capacity) {
			query_idx -= obj->capacity;
		}
	}

	return query_len;
}

