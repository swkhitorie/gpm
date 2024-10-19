#include "llddebug.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#ifdef __GNUC__
    __attribute__((weak)) void lldlogsendbytes(const uint8_t *pdata, uint16_t len) {}
#else 
    __weak void lldlogsendbytes(const uint8_t *pdata, uint16_t len) {}
#endif


char buffer[LLD_PRINT_BUFFER];
void lldprint(const char *fmt, ...)
{
#if (LLD_PRINT_ENABLE == 1)
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, LLD_PRINT_BUFFER, fmt, args);
    lldlogsendbytes((uint8_t *)buffer, strlen(buffer));
    va_end(args);
#endif
}


