
#ifndef __PWM_GENERATOR_H_
#define __PWM_GENERATOR_H_

namespace ESAF
{

/*!
 * @brief interface which can generate pwm
 */
class drv_pwm_generator
{
  public:
    virtual ~drv_pwm_generator() = default;

    /**
     * @brief (virtual function) generate pwm
     * @param selec channel of pulse generator
     * @param percent the duty of pwm
     */
    virtual void generate_pulse(int selec, float percent) { percent = 0.0f; };
};

}  // namespace ESAF

#endif  // drv_pwm_generator_H
