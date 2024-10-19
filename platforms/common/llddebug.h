#ifndef __LLD_DEBUG_H_
#define __LLD_DEBUG_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LLD_PRINT_ENABLE (1)
#define LLD_PRINT_BUFFER (128)
void lldprint(const char *fmt, ...);
void lldlogsendbytes(const uint8_t *pdata, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
