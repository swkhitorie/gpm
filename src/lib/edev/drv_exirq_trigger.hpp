
#ifndef __EXIRQ_TRIGGER_H_
#define __EXIRQ_TRIGGER_H_

#include <cstdint>

namespace ESAF
{

/*!
 * @brief external interrupt trigger
 */
class drv_exirq_trigger
{
  public:
    typedef void (*trigger_func)();

    drv_exirq_trigger()
    {
        _rising_irq  = nullptr;
        _falling_irq = nullptr;
    }

    ~drv_exirq_trigger()
    {
        _rising_irq  = nullptr;
        _falling_irq = nullptr;
    }

    enum trigger_mode
    {
        FALLING,
        RISING
    };

    /**
     * @brief connect trigger function with trigger mode
     * @param mode trigger mode
     * @param _trig trigger function
     */
    void connect(enum trigger_mode mode, trigger_func _trig)
    {
        if (_trig != nullptr)
        {
            switch (mode)
            {
                case FALLING:
						_falling_irq = _trig;
                    break;
                case RISING:
						_rising_irq = _trig;
                    break;
            }
        }
    }

  protected:
    trigger_func _rising_irq;
    trigger_func _falling_irq;
};

}  // namespace ESAF

#endif  // drv_exirq_trigger_H
