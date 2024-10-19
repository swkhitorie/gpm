
#ifndef __MOTOR_H_
#define __MOTOR_H_

#define MOTOR_SET_BIT(REG, BIT)			((REG) |= (BIT))
#define MOTOR_CLEAR_BIT(REG, BIT)		((REG) &= ~(BIT))

class Motor
{
public:
    Motor() {}
    ~Motor() {}
    enum MOTORMODE 
    {
        MOTOR_MODE_CUR,
        MOTOR_MODE_VEL,
        MOTOR_MODE_POS
    };

    unsigned int get_cstatus(void) { return cstatus; }
    unsigned int get_cerrorcode(void) { return cerrorcode; }
    
    float get_cvoltage(void) { return cvoltage; }
    float get_ccurrent(void) { return ccurrent; }
    
	float get_throttle(void) { return fb_throttle; }
	float get_torque(void) { return fb_torque; }
	float get_current(void) { return fb_current; }
	float get_velocity(void) { return fb_velocity; }
	float get_position(void) { return fb_position; }

	void set_throttle(float _thro) { ctrl_throttle = _thro; }
	void set_torque(float _torq) { ctrl_torque = _torq; }
	void set_current(float _current) { ctrl_current = _current; }
	void set_velocity(float _velocity) { ctrl_velocity = _velocity; }
	void set_position(float _position) { ctrl_position = _position; }

protected:
    unsigned int cstatus;
    unsigned int cerrorcode;

    float cvoltage;
    float ccurrent;

	float fb_throttle;				    /** present throttle of motor */
	float fb_torque;				    /** present torque of motor */
	float fb_current;				    /** present current of motor */
	float fb_velocity;				    /** present velocity of motor */
	float fb_position;				    /** present position of motor */

	float ctrl_throttle;				/** control throttle of motor */
	float ctrl_torque;				    /** control torque of motor */
	float ctrl_current;				    /** control current of motor */
	float ctrl_velocity;				/** control velocity of motor */
	float ctrl_position;				/** control position of motor */
};


#endif

