#include "log.hpp"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
using namespace ESAF;

__EWEAK void LogSendBytes(uint8_t *pdata, uint16_t len) {}

Log::Log(Mutex *m) : vaild(true), enable_status(true), enable_debug(true), enable_error(true)
{
    if (m)
    {
        mutex = m;
    }
    else
    {
        mutex = new MutexDefault();
    }
	vaild = true;
}

Log::~Log()
{
    delete mutex;
}

Log &Log::title(int level, const char *prefix, const char *func, int line)
{
    if (level)
    {
        vaild            = true;
        const char str[] = "\n%s/%d @ %s, L%d: ";
        print(str, prefix, level, func, line);
    }
    else
    {
        vaild = false;
    }
    return *this;
}

Log &Log::title(int level, const char *prefix)
{
    if (level)
    {
        vaild            = true;
        const char str[] = "\n%s/%d ";
        print(str, prefix, level);
    }
    else
    {
        vaild = false;
    }
    return *this;
}

void Log::setMutex(Mutex *m)
{
    if (m)
    {
        mutex = m;
    }
    else
    {
        mutex = new MutexDefault();
    }
}

Log &Log::print()
{
    return *this;
}

Log &Log::print(const char *fmt, ...)
{
    char buffer[DEBUG_PRINT_BUFFER];
    uint16_t len;

    if (vaild)
    {
        va_list args;
        va_start(args, fmt);
        vsnprintf(buffer, DEBUG_PRINT_BUFFER, fmt, args);
        len = strlen(buffer);
        mutex->lock();
        LogSendBytes((uint8_t *)buffer, len);
        mutex->unlock();
        va_end(args);
    }
    return *this;
}

Log &Log::operator<<(bool val)
{
    if (val)
    {
        print("True");
    }
    else
    {
        print("False");
    }
    return *this;
}

Log &Log::operator<<(short val)
{
    print("%d", val);
    return *this;
}

Log &Log::operator<<(uint16_t val)
{
    print("%u", val);
    return *this;
}

Log &Log::operator<<(int val)
{
    print("%d", val);
    return *this;
}

Log &Log::operator<<(uint32_t val)
{
    print("%u", val);
    return *this;
}

Log &Log::operator<<(long val)
{
    print("%ld", val);
    return *this;
}

Log &Log::operator<<(unsigned long val)
{
    print("%lu", val);
    return *this;
}

Log &Log::operator<<(long long val)
{
    print("%lld", val);
    return *this;
}

Log &Log::operator<<(unsigned long long val)
{
    print("%llu", val);
    return *this;
}

Log &Log::operator<<(float val)
{
    print("%f", val);
    return *this;
}
Log &Log::operator<<(double val)
{
    print("%lf", val);
    return *this;
}

Log &Log::operator<<(long double val)
{
    print("%Lf", val);
    return *this;
}

Log &Log::operator<<(void *val)
{
    print("ptr:0x%X", val);
    return *this;
}

Log &Log::operator<<(char c)
{
    print("%c", c);
    return *this;
}

Log &Log::operator<<(uint8_t c)
{
    print("0x%.2X", c);
    return *this;
}

Log &Log::operator<<(int8_t c)
{
    print("%c", c);
    return *this;
}

Log &Log::operator<<(const char *str)
{
    print("%s", str);
    return *this;
}

void Log::setStatusLog(bool enable)
{
    enable_status = enable;
}

void Log::setDebugLog(bool enable)
{
    enable_debug = enable;
}

void Log::setErrorLog(bool enable)
{
    enable_error = enable;
}

bool Log::getStatusLogState()
{
    return enable_status;
}

bool Log::getDebugLogState()
{
    return enable_debug;
}

bool Log::getErrorLogState()
{
    return enable_error;
}
