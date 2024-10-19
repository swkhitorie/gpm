
#ifndef __ENCODER_INCREMENTAL_H_
#define __ENCODER_INCREMENTAL_H_

#include <cstdint>

namespace ESAF
{

/*!
 * @brief incremental encoder interface
 */
class drv_encoder_incremental
{
  public:
    virtual ~drv_encoder_incremental() = default;

    /**
     * @brief (virtual function) get incremental encoder pulse count
     * @return int64_t pulse count
     */
    virtual int64_t get_encoder_count(void) { return 0; };
};

}  // namespace ESAF

#endif  // drv_encoder_incremental_H
