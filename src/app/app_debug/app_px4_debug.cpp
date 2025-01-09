#include "app_px4_debug.h"

static bool g_exit1 = false;

static char debug_2[512];
int px4_app1_main(int argc, char *argv[])
{
	int ret = 0;

    for(;;) {
		ret = px4_sleep(1000);
        PX4_INFO("px4 app1 running %.5f", hrt_absolute_time()/1e6f);

        if (g_exit1) {
            PX4_INFO("now app1 exit self ");
            px4_task_exit(ret);
        }
    }

	PX4_INFO("px4 app1 exit");
	return ret;
}

int px4_app1_detect_main(int argc, char *argv[])
{
	int ret = 0;

    for (;;) {
        ret = px4_sleep(1000);
        px4_show_tasks();
        debug_led_toggle();
        if (hrt_absolute_time() / 1e6f >= 10.0f) {
            PX4_INFO("now let app1 exit self ");
            g_exit1 = true;
        }
        vTaskList(&debug_2[0]);
        fprintf(stdout, "%s \r\n", debug_2);
    }
	return ret;
}

void app_px4_debug_init()
{
    px4_task_t taskid1 = px4_task_spawn_cmd( "px4_app1", 
                                            SCHED_DEFAULT, SCHED_PRIORITY_MIN + 3, 
                                            2048, px4_app1_main, (char *const *)NULL);
    px4_task_t taskid2 = px4_task_spawn_cmd( "app1_detect", 
                                            SCHED_DEFAULT, SCHED_PRIORITY_MIN + 4, 
                                            1024, px4_app1_detect_main, (char *const *)NULL);

}
