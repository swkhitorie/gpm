#pragma once

class JoyStick
{
public:
	JoyStick() = default;
	~JoyStick() = default;
	
	enum JoyType {
		JMID = 0x00,
		JNOTMID,
		ROCKER
	};
	
	static int8_t value(int8_t pctvalue, bool reverse, uint8_t deadzone, enum JoyType joytype)
	{
		int8_t val;
		if (reverse) {
			val = -pctvalue + 100;
		} else {
			val = pctvalue;
		}
		
		switch (joytype) {
		case ROCKER:
			val = val / 50;
			break;
		case JNOTMID:
			if (val <= deadzone)
				val = 0;
			else val -= deadzone;
			break;
		case JMID:
			if (val < (50 - deadzone))
				val = -(50 - deadzone) + val;
			else if (val > (50 + deadzone))
				val = val - (50 + deadzone);
			else val = 0;
			break;
		}
	
		return val;
	}
};
