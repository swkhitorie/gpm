
#ifndef __PULSE_COUNTER_H_
#define __PULSE_COUNTER_H_

#include <cstdint>

namespace ESAF
{

/*!
 * @brief interface which count the inpu digital pulse
 */
class drv_pulse_counter
{
  public:
    virtual ~drv_pulse_counter() = default;

    /**
     * @brief (virtual function) get input pulse count
     * @param selec channel of pulse counter
     * @return int64_t pulse count
     */
    virtual int64_t get_pulse_count(int selec) { return 0; };
};

}  // namespace ESAF

#endif  // drv_pulse_counter_H
