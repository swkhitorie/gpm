#include "board_config.h"

static RTC_HandleTypeDef hrtc;
static RTC_TimeTypeDef vtime;
static RTC_DateTypeDef vdate;
static struct tm now_time;

void board_rtc_init()
{
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    __HAL_RCC_RTC_ENABLE();

	hrtc.Instance = RTC;					
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;

    HAL_RTC_Init(&hrtc);
}

struct tm *board_rtc_timeget()
{
    HAL_RTC_GetTime(&hrtc, &vtime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &vdate, RTC_FORMAT_BIN);

    /* 
    Year[actual - 2000], tm_mon[1,12] tm_mday[1,31] 
    tm_hour[0,23] tm_min[0,59] tm_sec[0,59]
    */   

    /* 
    tm_year[actual - 1900], tm_mon[0,11] tm_mday[1,31] 
    tm_hour[0,23] tm_min[0,59] tm_sec[0,59]
    */

    now_time.tm_year = (vdate.Year + 2000) - 1900;
    now_time.tm_mon = vdate.Month - 1;
    now_time.tm_mday = vdate.Date;
    now_time.tm_hour = vtime.Hours;
    now_time.tm_min = vtime.Minutes;
    now_time.tm_sec = vtime.Seconds;
    return &now_time;
}

void board_rtc_timeset(struct tm *val)
{
    vdate.Year = (val->tm_year + 1900 - 2000);
    vdate.Month = val->tm_mon + 1;
    vdate.Date = val->tm_mday;
    vtime.Hours = val->tm_hour;
    vtime.Minutes = val->tm_min;
    vtime.Seconds = val->tm_sec;

    HAL_RTC_SetTime(&hrtc, &vtime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(&hrtc, &vdate, RTC_FORMAT_BIN);
}