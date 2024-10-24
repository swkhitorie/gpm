
#include "lld_include.h"
#include "ttscheduler.h"
#include "llddebug.h"

#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "test_cpp.h"

lld_uart_t com3;

void debug(void *p)
{
    lldprint("[debug] edit armcc cpu time: %f \r\n", lld_kernel_get_time(0) / 1e6);
    test_obj a;
    a.test_1();
}

int main()
{
    lld_kernel_init(0);
    lld_uart_init(&com3,3,115200,true,true,true,USART_Parity_No,USART_WordLength_8b,USART_StopBits_1);

    ttscheduler_init();
    ttscheduler_add_task("debug", debug, NULL, 100, 0);
    
    while (1) {
        ttscheduler_dispatch_tasks();
    }
}

void lldlogsendbytes(const uint8_t *pdata, uint16_t len) 
{
    lld_uart_send_bytes(&com3, pdata, len, RWIT); 
}
