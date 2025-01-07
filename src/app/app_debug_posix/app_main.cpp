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

typedef struct __pthread_test{
    pthread_attr_t attr;
    pthread_t id;
    float arg;
    struct sched_param param;
} pthread_test_t;

typedef struct __euler {
    float pitch;
    float roll;
    float yaw;
} euler_t;

pthread_test_t p1;
pthread_test_t p2;
pthread_test_t p3;
pthread_test_t p4;
pthread_test_t p5;
mqd_t msg_1;

/**
 * 1. test simple pthread
 * 2. test time clock delay, like Delay and Delay Until
 * 3. soft timer triggle
 * 4. msg queue debug
 */
#define POSIX_TEST_ITEM      (4)

// 1:Delay 2:Delay Until
#define POSIX_DELAY_METHOD   (2)

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

void utils_fr_posix_debug()
{
    int res = 0;
    int pos = 0;
    char d1[20];  /* can not initialized with char d1[20] = "str"; */
    struct timespec x1;
    struct timespec y1;
    struct timespec z1;
    TickType_t a1;
    x1.tv_sec = 5;
    x1.tv_nsec = 30 * 1000 * 1000;
    y1.tv_sec = 4;
    y1.tv_nsec = 700 * 1000;
    z1.tv_sec = 2;
    z1.tv_nsec = 4000 * 1000 * 1000;
    sprintf(d1, "hello world\r\n");
    fprintf(stdout, "strlen : %d\r\n", utils_strlen(d1, 30));
    fprintf(stdout, "x1 valid:%d, y1 valid:%d, z1 valid:%d\r\n",
                        utils_validtimespec(&x1), utils_validtimespec(&y1), utils_validtimespec(&z1));
    fprintf(stdout, "x1>y1?:%d, y1>x1?:%d \r\n", utils_timespec_compare(&x1, &y1), utils_timespec_compare(&y1, &x1));
    res = utils_timespec_add(&x1, &y1, &z1);
    fprintf(stdout, "x1+y1:%d,%d | %d\r\n", z1.tv_sec, z1.tv_nsec, res);
    res = utils_timespec_subtract(&x1, &y1, &z1);
    fprintf(stdout, "x1-y1:%d,%d | %d\r\n", z1.tv_sec, z1.tv_nsec, res);
    res = utils_timespec_addnanoseconds(&x1, 350*1000, &z1);
    fprintf(stdout, "x1+30*1000*1000:%d,%d | %d\r\n", z1.tv_sec, z1.tv_nsec, res);
    res = utils_timespec_toticks(&x1, &a1);
    fprintf(stdout, "x1 to ticks:%d | %d\r\n", a1, res);
    utils_nanoseconds_totimespec(1 * 1000 * 1000 * 1000 + 3 * 1000, &z1);
    fprintf(stdout, "1000000000+3000:%d,%d | %d\r\n", z1.tv_sec, z1.tv_nsec, res);
    res = utils_timespec_todeltaticks(&x1, &y1, &a1);
    fprintf(stdout, "delta ticks :%d | %d\r\n", a1, res);
}

void time_clock_fr_debug()
{
    struct timespec x1;
    clock_gettime(0, &x1);
    fprintf(stdout, "clock gettime: %d %d\r\n", x1.tv_sec, x1.tv_nsec);
}




void* p1_entry(void *p)
{
    float *value = (float *)p;
    char name[16] = {'\0'};
    int i = 0;
    pthread_getname_np(pthread_self(), &name[0], 16);
    fprintf(stdout, "[%s] %.6f p1 start\r\n", name, hrt_absolute_time()/1e6f);
    memset(&name[0], 0, 16);
    strcpy(&name[0], "p1_edited");
    pthread_setname_np(pthread_self(), &name[0]);
    for (; i < 3; i++) {
        fprintf(stdout, "[%s] %.6f p1 tag, val: %.5f, %d\r\n", name, hrt_absolute_time()/1e6f,
                                    *value, i);
        sleep(1000); // sleep 2s
    }
    fprintf(stdout, "[%s] %.6f p1 end\r\n", name, hrt_absolute_time()/1e6f);
    return NULL;
}

void* p2_entry(void *p)
{
    float *value = (float *)p;
    char name[16] = {'\0'};
    int i = 0;
    pthread_getname_np(pthread_self(), &name[0], 16);
    fprintf(stdout, "[%s] %.6f p2 start\r\n", name, hrt_absolute_time()/1e6f);
    memset(&name[0], 0, 16);
    strcpy(&name[0], "p2_edited");
    pthread_setname_np(pthread_self(), &name[0]);
    for (; i < 15; i++) {
        fprintf(stdout, "[%s] %.6f p2 tag, val: %.5f, %d\r\n", name, hrt_absolute_time()/1e6f,
                                    *value, i);
        sleep(1000);
        debug_led_toggle();
    }
    fprintf(stdout, "[%s] %.6f p2 end\r\n", name, hrt_absolute_time()/1e6f);
    return NULL;
}

static char debug_str1[1024];
void* p3_entry(void *p)
{
    struct timespec rqtp;
    struct timespec x1;
#if POSIX_DELAY_METHOD == 1
    rqtp.tv_sec = 1;
    rqtp.tv_nsec = 0;
#endif
    for (;;) {
        debug_led_toggle();
        vTaskList(&debug_str1[0]);
        printf("%s\r\n", debug_str1);
        //utils_fr_posix_debug();
        clock_gettime(0, &x1);
        fprintf(stdout, "clock gettime: %d %d\r\n", x1.tv_sec, x1.tv_nsec);
#if POSIX_DELAY_METHOD == 2
        clock_gettime(0, &rqtp);
        rqtp.tv_sec += 1;
        int re = clock_nanosleep(0, TIMER_ABSTIME, &rqtp, NULL);
#endif
#if POSIX_DELAY_METHOD == 1
        nanosleep(&rqtp, NULL);
#endif
    }
}

void* p4_entry(void *p)
{
    float *value = (float *)p;
    char name[16] = {'\0'};
    int i = 0;
    strcpy(&name[0], "p4_edited");
    pthread_setname_np(pthread_self(), &name[0]);

    msg_1 = mq_open("/node0", O_RDWR | O_CREAT | O_NONBLOCK, 0, NULL);
    fprintf(stdout, "[%s] msg_1 : %d\r\n", &name[0], msg_1);

    euler_t tmp = {.pitch = 1.0f, .roll = 2.0f, .yaw = 3.14f,};
    for (;;) {
        tmp.pitch += 0.1f;
        tmp.roll += 0.2f;
        tmp.yaw -= 0.5f;

        mq_send(msg_1, (const char*)&tmp, sizeof(euler_t), 0);
        fprintf(stdout, "[%s] send data: %.3f, %.3f, %.3f\r\n", &name[0], tmp.pitch, tmp.roll, tmp.yaw);
        sleep(500);
    }

    return NULL;
}

void* p5_entry(void *p)
{
    float *value = (float *)p;
    char name[16] = {'\0'};
    int i = 0;
    strcpy(&name[0], "p5_edited");
    pthread_setname_np(pthread_self(), &name[0]);

    // mqd_t rcv = mq_open("/d1", 0x00, 0, NULL);
    // if (rcv == NULL) {
    //     fprintf(stdout, "can not find msg /d1\r\n");
    // }

    euler_t tmp_rcv;
    char rcv_array[64];
    for (;;) {
        mq_receive(msg_1, &rcv_array[0], sizeof(euler_t), NULL);
        memcpy((char *)&tmp_rcv, &rcv_array[0], sizeof(euler_t));
        fprintf(stdout, "[%s] rcv data: %.3f, %.3f, %.3f\r\n", &name[0], tmp_rcv.pitch, tmp_rcv.roll, tmp_rcv.yaw);
        sleep(1000);
        debug_led_toggle();
    }

    return NULL;
}

void tr1_entry(union sigval value)
{
    fprintf(stdout, "[tr1] %.6f sign event \r\n",hrt_absolute_time()/1e6f);
}

int main(void)
{
    board_init();
    hrt_init();

#if POSIX_TEST_ITEM == 1
    {
        int rv;
        p1.param.sched_priority = 4;
        p1.arg = 3.425f;
        pthread_attr_init(&p1.attr);
        pthread_attr_setdetachstate(&p1.attr, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setschedparam(&p1.attr, &p1.param);
        pthread_attr_setstacksize(&p1.attr, 512*sizeof(StackType_t));
        rv = pthread_create(&p1.id, &p1.attr, &p1_entry, &p1.arg);
        if (rv != 0) {
            fprintf(stdout, "[p1] %.6f create pthread failed\r\n",hrt_absolute_time()/1e6f);
        }
    }

    {
        int rv;
        p2.param.sched_priority = 6;
        p2.arg = 0.123f;
        pthread_attr_init(&p2.attr);
        // PTHREAD_CREATE_DETACHED PTHREAD_CREATE_JOINABLE
        pthread_attr_setdetachstate(&p2.attr, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setschedparam(&p2.attr, &p2.param);
        pthread_attr_setstacksize(&p2.attr, 512*sizeof(StackType_t));
        rv = pthread_create(&p2.id, &p2.attr, &p2_entry, &p2.arg);
        if (rv != 0) {
            fprintf(stdout, "[p2] %.6f create pthread failed\r\n",hrt_absolute_time()/1e6f);
        }
    }
#endif

#if POSIX_TEST_ITEM == 2
    {
        int rv;
        p3.param.sched_priority = 6;
        p3.arg = 0.123f;
        pthread_attr_init(&p3.attr);
        // PTHREAD_CREATE_DETACHED PTHREAD_CREATE_JOINABLE
        pthread_attr_setdetachstate(&p3.attr, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setschedparam(&p3.attr, &p3.param);
        pthread_attr_setstacksize(&p3.attr, 512*sizeof(StackType_t));
        rv = pthread_create(&p3.id, &p3.attr, &p3_entry, &p3.arg);
        if (rv != 0) {
            fprintf(stdout, "[p3] %.6f create pthread failed\r\n",hrt_absolute_time()/1e6f);
        }
    }
#endif

#if POSIX_TEST_ITEM == 3
    timer_t tr1;
    struct sigevent event;
    pthread_attr_t event_attr;
    struct sched_param event_param = {.sched_priority = 5,};
    pthread_attr_init(&event_attr);
    pthread_attr_setdetachstate(&event_attr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setschedparam(&event_attr, &event_param);
    pthread_attr_setstacksize(&event_attr, 256*sizeof(StackType_t));
    event.sigev_notify = SIGEV_THREAD;
    event.sigev_value.sival_int = 4;
    event.sigev_notify_function = tr1_entry;
    event.sigev_notify_attributes = NULL;
    int res = timer_create(NULL, &event, &tr1);
    fprintf(stdout, "[tr1] timer create: %d\r\n", res);

    struct itimerspec val;
    timer_gettime(tr1, &val);
    fprintf(stdout, "[tr1] get interval: %d %d | %d %d\r\n", 
        val.it_value.tv_sec, val.it_value.tv_nsec,
        val.it_interval.tv_sec, val.it_interval.tv_nsec);

    struct itimerspec new_val;
    clock_gettime(NULL, &new_val.it_value);
    new_val.it_interval.tv_sec = 0;
    new_val.it_interval.tv_nsec = 0;
    new_val.it_value.tv_sec += 3;
    int re2 = timer_settime(tr1, TIMER_ABSTIME, &new_val, &val);
    fprintf(stdout, "[tr1] set interval: %d\r\n", re2);
#endif


#if POSIX_TEST_ITEM == 4
    {
        int rv;
        p4.param.sched_priority = 4;
        p4.arg = -41.0f;
        pthread_attr_init(&p4.attr);
        pthread_attr_setdetachstate(&p4.attr, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setschedparam(&p4.attr, &p4.param);
        pthread_attr_setstacksize(&p4.attr, 512*sizeof(StackType_t));
        rv = pthread_create(&p4.id, &p4.attr, &p4_entry, &p4.arg);
        if (rv != 0) {
            fprintf(stdout, "[p4] %.6f create pthread failed\r\n",hrt_absolute_time()/1e6f);
        }
    }

    {
        int rv;
        p5.param.sched_priority = 6;
        p5.arg = 0.123f;
        pthread_attr_init(&p5.attr);
        // PTHREAD_CREATE_DETACHED PTHREAD_CREATE_JOINABLE
        pthread_attr_setdetachstate(&p5.attr, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setschedparam(&p5.attr, &p5.param);
        pthread_attr_setstacksize(&p5.attr, 512*sizeof(StackType_t));
        rv = pthread_create(&p5.id, &p5.attr, &p5_entry, &p5.arg);
        if (rv != 0) {
            fprintf(stdout, "[p5] %.6f create pthread failed\r\n",hrt_absolute_time()/1e6f);
        }
    }
#endif

    sched_start();
    for (;;);
}
