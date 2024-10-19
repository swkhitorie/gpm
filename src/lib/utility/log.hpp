
#ifndef __LOG_H_
#define __LOG_H_

#include <stdint.h>
#include "signleton.hpp"
#include "thread_manager.hpp"

#ifdef __GNUC__
#    define EPACKED(__Declaration__) __Declaration__ __attribute__((packed))
#    define __EWEAK __attribute__((weak))
#else
#    define __EWEAK __attribute__((weak))
#    define EPACKED(__Declaration__) __packed __Declaration__
#endif

#define __STM32__
//#define __WIN__

#ifdef __STM32__
#    define __LOG_SOLUTION__
#endif

#ifdef __WIN__
#    define __LOG_SOLUTION__ virtual
#endif

#define DEBUG_PRINT_BUFFER (1024)

#define DLOG_MUTEX(_mutex_) ESAF::Log::instance().setMutex(_mutex_)

#define DLOG(_title_) ESAF::Log::instance().title((_title_), #_title_, __FUNCTION__, __LINE__).print

#define DLOG_PRIVATE(_title_) ESAF::Log::instance().title((_title_), #_title_).print

#define STATUS ESAF::Log::instance().getStatusLogState()
#define ERRORLOG ESAF::Log::instance().getErrorLogState()
#define DEBUG ESAF::Log::instance().getDebugLogState()

/*! @brief Global Logging macro for status messages
 *  @details Users can use methods in the ESAF::Log class to
 *  enable/disable this logging channel
 */
#define DDSTATUS DLOG(STATUS)
#define DDSTATUS_PRIVATE DLOG_PRIVATE(STATUS)

/*! @brief Global Logging macro for error messages
 *  @details Users can use methods in the ESAF::Log class to
 *  enable/disable this logging channel
 */
#define DERROR DLOG(ERRORLOG)
#define DERROR_PRIVATE DLOG_PRIVATE(ERRORLOG)

/*! @brief Global Logging macro for debug messages
 *  @details Users can use methods in the ESAF::Log class to
 *  enable/disable this logging channel
 */
#define DDEBUG DLOG(DEBUG)
#define DDEBUG_PRIVATE DLOG_PRIVATE(DEBUG)

#define ELOG ESAF::Log::instance()
#define EDEBUG ELOG.print

extern "C" {
/*! @brief weak function for final log output, default to do nothing
 */
void LogSendBytes(uint8_t *pdata, uint16_t len);
}

namespace ESAF
{

/*! @brief Logger for ESAF supporting different logging channels
 *
 * @details The Log class is a singleton and contains some pre-defined logging
 * levels. The class provides methods to turn on or off these predefined
 * logging levels. Users can also create their own logging channels using the
 * DLOG macro.
 */
class Log : public Singleton<Log>
{
  public:
    ~Log();
    Log(Mutex *m = 0);

    void setMutex(Mutex *m);

    /*!
     * @brief if title level is 0, this log would not be print at all
     * this feature is used for dynamical/statical optional log output
     */
    Log &title(int level, const char *prefix, const char *func, int line);

    /*!
     * @brief Used for closed source logging,
     * where we don't want to print function name and line number
     */
    Log &title(int level, const char *prefix);

    Log &print();

    /*!
     * @brief set enable logging of status messages
     */
    void setStatusLog(bool enable);

    /*!
     * @brief set debug logging of status messages
     */
    void setDebugLog(bool enable);

    /*!
     * @brief set error logging of status messages
     */
    void setErrorLog(bool enable);

    // Retrieve logging switches - used for global macros
    bool getStatusLogState();
    bool getDebugLogState();
    bool getErrorLogState();

    __LOG_SOLUTION__ Log &print(const char *fmt, ...);

    Log &operator<<(bool val);
    Log &operator<<(short val);
    Log &operator<<(uint16_t val);
    Log &operator<<(int val);
    Log &operator<<(uint32_t val);
    Log &operator<<(long val);
    Log &operator<<(unsigned long val);
    Log &operator<<(long long val);
    Log &operator<<(unsigned long long val);
    Log &operator<<(float val);
    Log &operator<<(double val);
    Log &operator<<(long double val);
    Log &operator<<(void *val);
    Log &operator<<(char c);
    Log &operator<<(uint8_t c);
    Log &operator<<(int8_t c);
    Log &operator<<(const char *str);

  public:
    bool vaild;
    bool enable_status;
    bool enable_debug;
    bool enable_error;

    Mutex *mutex;
};

}  // namespace ESAF

#endif  // log_H
