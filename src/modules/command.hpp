#ifndef _COMMAND_H_
#define _COMMAND_H_

#define COMMAND_MAX_CHANNELS    (15)

class Command
{
public:
	Command()
    {
        for (int i = 0; i < COMMAND_MAX_CHANNELS; i++)
            odrc[i] = 0;

        for (int i = 0; i < COMMAND_MAX_CHANNELS; i++) {
            odsw[i] = 0;
        }
    }
	~Command() {}

	int getod_rc(unsigned int _index) { return odrc[_index]; }
	unsigned char getod_sw(unsigned int _index) { return odsw[_index]; }

	void setsw(unsigned int _index, int value) { odsw[_index] = value / 50; }
	void setrc(unsigned int _index, int value, unsigned char deadzone) { odrc[_index] = decode_joy(value, deadzone); }
    void setrc_pure(unsigned int _index, int value, unsigned char deadzone) { odrc[_index] = decode_joy_pure(value, deadzone); }

public:
	int decode_joy(int _data, unsigned char deadzone)
    {
        if (_data > (50 - deadzone) && _data < (50 + deadzone))
            return 0;
        else if (_data <= (50 - deadzone))
            return ((50 - deadzone) - _data);
        else if (_data >= (50 + deadzone))
            return ((50 + deadzone) - _data);
        return 0; 
    }
    
    int decode_joy_pure(int _data, unsigned char deadzone)
    {
        if (_data < deadzone)
            return 0;
        else return _data - deadzone;
    }

public:
	int odrc[COMMAND_MAX_CHANNELS];
	unsigned char odsw[COMMAND_MAX_CHANNELS];
};









#endif
