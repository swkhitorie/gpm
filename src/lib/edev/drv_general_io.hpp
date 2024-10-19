
#ifndef __GENERAL_IO_H_
#define __GENERAL_IO_H_

namespace ESAF
{

/*!
 * @brief General-purpose input/output interface
 */
class drv_general_io
{
  public:
    virtual ~drv_general_io() = default;

    /**
     * @brief set pin
     * @param level pin electrical level, true - high level, false - low level
     */
    virtual void set(bool level) = 0;

    /**
     * @brief get pin level
     * @return unsigned char 1 - high level, 0 - low level
     */
    virtual unsigned char get(void) = 0;

    drv_general_io &operator=(bool level)
    {
        set(level);
        return (*this);
    }
};

}  // namespace ESAF

#endif  // drv_general_io_H
