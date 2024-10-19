#include "main.h"
#include "usb_config.h"
#include "usbd_core.h"
#include "usbd_cdc.h"

#include "bl_kernel.h"
#include "bl_board.h"
#include "bl_flash.h"

static char compiler_time[] = "2024Äê6ÔÂ13ÈÕ10:46:58";
void LoadFirmware(void){}
void NMI_Handler(void) {}
void MemManage_Handler(void) {}
void BusFault_Handler(void) {}
void UsageFault_Handler(void) {}
void HardFault_Handler(void) {}	
    
int main()
{
    prevent_warning();
    start_up();
    HAL_Delay(1000);
    board_usart_init();
    bt_init();
    
    extern void cdc_acm_init(uint8_t busid, uint32_t reg_base);
    cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);

    uint32_t state_allow_erase = (0x1 | 0x2);
    uint32_t state_allow_reboot = (0x1 | 0x2 | 0x8 | 0x10);
    uint32_t core_tick = get_tick();
    
	while (1) {
        
        bootloader();
        
        /* no any firmware upgrade host communication */
        if (
            ((get_tick() - core_tick) > 3 * 1e3) && 
            ((bl_get_state() & state_allow_erase) != state_allow_erase)
            ) {
            jump_to_app();
        }
        
        /* when upgrade process end success */
        if ((bl_get_state() & state_allow_reboot) == state_allow_reboot) {
            jump_to_app();
        }
	}
}

