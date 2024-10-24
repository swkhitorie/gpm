
#include "lld_include.h"
#include "ttscheduler.h"
#include "llddebug.h"

#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

lld_uart_t com1;

void debug(void *p)
{
    lldprint("[debug] edit h7 arm dma cppfile cpu time: %f \r\n", lld_kernel_get_time(0) / 1e6);
}

int main()
{
    lld_kernel_init(0);
    lld_uart_init(&com1,1,115200,1,1,true,true,UART_PARITY_NONE,UART_WORDLENGTH_8B,UART_STOPBITS_1);
    
    ttscheduler_init();
    ttscheduler_add_task("debug", debug, NULL, 100, 0);

    while (1) {
        ttscheduler_dispatch_tasks();
    }
}

void lldlogsendbytes(const uint8_t *pdata, uint16_t len) 
{
    lld_uart_send_bytes(&com1, pdata, len, RWIT); 
}
