#include "./app_main.h"
#include <drivers/drv_hrt.h>
#include <stdio.h>
#include <string.h>

#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <fcntl.h>
#include <mqueue.h>
#include <sched.h>
#include <sdqueue.h>
#include <semaphore.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>
#include <utils.h>
#include <pthread.h>

/**
 * test item:
 * pthread: pthread_attr
 *          pthread_muttx
 */


void debug_led_toggle()
{
#if   BOARD_SELECT == BOARD_FMUV2
    board_led_toggle();
#elif BOARD_SELECT == BOARD_FANKEH7
    board_blue_led_toggle();
#elif BOARD_SELECT == BOARD_FMUV6C
    board_blue_led_toggle();
    board_red_led_toggle();
#endif
}

void fr_heart(void *p)
{
    static char fr_debug_str[512];
    for (;;) {
        vTaskList(&fr_debug_str[0]);
        fprintf(stdout, "%s \r\n", fr_debug_str);
        debug_led_toggle();
        vTaskDelay(1000);
    }
}

pthread_mutex_t m1;
int critical_val = 2;


typedef struct {
	float value;
} p1_data_t;
pthread_attr_t t1_attr;
pthread_t t1;
p1_data_t t1_params;
void* t1_process(void *p)
{
    float *value = (float *)p;
    char name[16] = {'\0'};
    int i = 0;

    pthread_getname_np(pthread_self(), &name[0], 16);
    fprintf(stdout, "[%s] %.6f t1_process start\r\n", name, hrt_absolute_time()/1e6f);

    memset(&name[0], 0, 16);
    strcpy(&name[0], "t1_edited");
    pthread_setname_np(pthread_self(), &name[0]);

    for (; i < 3; i++) {
        fprintf(stdout, "[%s] %.6f t1_process tag, val: %.5f, %d, %d\r\n", name, hrt_absolute_time()/1e6f,
                                    *value, i, critical_val);
        sleep(1000); // sleep 2s
    }

    fprintf(stdout, "[%s] %.6f t1_process end\r\n", name, hrt_absolute_time()/1e6f);
    return NULL;
}


typedef struct {
	float value;
} p2_data_t;
pthread_attr_t t2_attr;
pthread_t t2;
p2_data_t t2_params;
void* t2_process(void *p)
{
    float *value = (float *)p;
    char name[16] = {'\0'};
    int i = 0;

    // while(hrt_absolute_time()/1e6f < 10.0f);

    pthread_getname_np(pthread_self(), &name[0], 16);
    fprintf(stdout, "[%s] %.6f t2_process start\r\n", name, hrt_absolute_time()/1e6f);

    memset(&name[0], 0, 16);
    strcpy(&name[0], "t2_edited");
    pthread_setname_np(pthread_self(), &name[0]);

    for (; i < 60; i++) {
        fprintf(stdout, "[%s] %.6f t2_process tag, val: %.5f, %d, %d\r\n", name, hrt_absolute_time()/1e6f,
                                    *value, i, critical_val);
        sleep(500);
        debug_led_toggle();
    }

    fprintf(stdout, "[%s] %.6f t2_process end\r\n", name, hrt_absolute_time()/1e6f);
    return NULL;
}

int main(void)
{
    board_init();
    hrt_init();

    {
        int rv;
        struct sched_param p1_param = { .sched_priority = 4, };
        t1_params.value = 3.512f;
        pthread_attr_init(&t1_attr);
        // PTHREAD_CREATE_DETACHED PTHREAD_CREATE_JOINABLE
        pthread_attr_setdetachstate(&t1_attr, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setschedparam(&t1_attr, &p1_param);
        pthread_attr_setstacksize(&t1_attr, 1024 * sizeof(StackType_t));
        // rv = pthread_create(&t1, &t1_attr, &t1_process, &t1_params);
        // if (rv != 0) {
        //     fprintf(stdout, "[t1] %.6f create pthread failed\r\n",hrt_absolute_time()/1e6f);
        // }
    }

    {
        int rv;
        struct sched_param p2_param = { .sched_priority = 6, };
        t2_params.value = 0.123f;
        pthread_attr_init(&t2_attr);
        // PTHREAD_CREATE_DETACHED PTHREAD_CREATE_JOINABLE
        pthread_attr_setdetachstate(&t2_attr, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setschedparam(&t2_attr, &p2_param);
        pthread_attr_setstacksize(&t2_attr, 1024 * sizeof(StackType_t));
        rv = pthread_create(&t2, &t2_attr, &t2_process, &t2_params);
        if (rv != 0) {
            fprintf(stdout, "[t2] %.6f create pthread failed\r\n",hrt_absolute_time()/1e6f);
        }
    }

    //xTaskCreate(fr_heart, "ht_debug1", 1024, NULL, 3, NULL);

    sched_start();
    for (;;);
}
