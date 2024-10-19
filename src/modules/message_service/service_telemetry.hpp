
#ifndef __SERVICE_TELEMETRY_H_
#define __SERVICE_TELEMETRY_H_

namespace ESAF
{
namespace Telemetry
{

/*!
 * @brief enum ServiceName is the interface for user to create service and access
 * @note Please see telemetry_doc.hpp for detailed documentation.
 */
typedef enum
{
    SRV_ENABLEMOTOR,
    TOTAL_SERVICE,
} ServiceName;

/*!
 * @brief enum SRV_ENABLEMOTOR - enable all motor
 */
struct srvEnableMotor
{
    struct request
    {
        int enable_num;
    };
    struct response
    {
        int enable_res;
    };
};

/*! @brief template struct maps a service input output to the corresponding interface
 * type
 */
template <ServiceName T>
struct srvTypeMap
{
    typedef void request_type;
    typedef void response_type;
};

// clang-format off
template <> struct srvTypeMap<SRV_ENABLEMOTOR          > {  typedef srvEnableMotor::request request_type;
                                                            typedef srvEnableMotor::response response_type; };




}  // namespace Telemetry
}  // namespace ESAF


#endif  // service_telemetry_H
