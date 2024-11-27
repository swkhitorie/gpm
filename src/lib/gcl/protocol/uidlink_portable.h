
#ifndef _UIDLINK_PORTABLE_H_
#define _UIDLINK_PORTABLE_H_
#include <cstdint>
#include <cstdlib>
#include <cstring>

extern "C" {
/*! @brief weak function for final uidlink send, default to do nothing
 */
void uidlink_lowlevel_send_uart(const char *buf, uint16_t len);
}

namespace ESAF
{

/*! @brief low level interface for uidlink
 *
 */
class uidlink_lowlevel_layer
{
  public:
    uidlink_lowlevel_layer() {}

  protected:
    /*! @brief lowlevel data send fuction,
     *  @param buf array pointer
     *  @param len data array len
     */
    void _lowlevel_send_uart(const char *buf, uint16_t len)
    {
        uidlink_lowlevel_send_uart(buf, len);
    }

    /*! @brief lowlevel data start send fuction,
     *  @param len data array len
     */
    void _lowlevel_start_uart_send(uint16_t len) {}

    /*! @brief lowlevel data end send fuction,
     *  @param len data array len
     */
    void _lowlevel_end_uart_send(uint16_t len) {}
};

}  // namespace ESAF

#endif  // uidlink_portable_H
