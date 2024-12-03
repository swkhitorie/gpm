#include "app_main.h"
#include "FreeRTOS.h"
// #define __PX4_POSIX
// #include <visibility.h>
// #include <px4_platform_common/atomic.h>
// #include <px4_platform_common/atomic_bitset.h>
// #include <px4_platform_common/log.h>
// #include <px4_platform_common/defines.h>

// #include <containers/Array.hpp>
// #include <containers/Bitset.hpp>
// #include <containers/BlockingList.hpp>
// #include <containers/BlockingQueue.hpp>
// #include <containers/IntrusiveQueue.hpp>
// #include <containers/IntrusiveSortedList.hpp>
// #include <containers/List.hpp>
// #include <containers/LockGuard.hpp>

// #include <uORB/uORB.h>

int main(void)
{
	board_app_vector_init();
	board_io_array_init();
	board_app_init();

    // px4::atomic<int> a;
    // a.fetch_sub(3);
    // px4::AtomicBitset<10> b;
    // b.set(0, true);

    HAL_Init();

    board_system_rcc_config();

    HAL_Delay(1000);
    cdc_acm_init(0, USB_OTG_FS_PERIPH_BASE);
    HAL_Delay(800);

    uint32_t tick = HAL_GetTick();

    while (1) {
        if (HAL_GetTick() - tick > 500) {
            tick = HAL_GetTick();
            cdc_acm_data_send_with_dtr_test(0);
            board_blue_led_toggle();
        }
    }
}
