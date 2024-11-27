#pragma once

#include <math.h>
#include "platform_defines.h"

class Lpf2ndButterworth
{
public:
	Lpf2ndButterworth() = default;
	~Lpf2ndButterworth() = default;
	
	void setcutoff_freq(float _sample_freq, float _cutoff_freq)
	{
		float fr = 0;
		float ohm = 0;
		float c = 0;

		sample_Freq = _sample_freq;
		cutoff_Freq = _cutoff_freq;

		fr = sample_Freq / cutoff_Freq;
		ohm = tanf(M_PI_F / fr);
		c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;

		if (cutoff_Freq > 0.0f) {
			_b01 = ohm * ohm / c;
			_b11 = 2.0f * _b01;
			_b21 = _b01;
			_a11 = 2.0f * (ohm * ohm - 1.0f) / c;
			_a21 = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
		}
	}
	
	
	float apply(float sample)
	{
		float delay_element_0 = 0 , output = 0;
		if (cutoff_Freq <= 0.0f) {
			/* no filtering */
			return sample;
		}else {
			delay_element_0 = sample - _delay_element_11 * _a11 - _delay_element_21 * _a21;
			/* do the filtering */
			if (isnan(delay_element_0) || isinf(delay_element_0)) {
				/* don't allow bad values to propogate via the filter */
				delay_element_0 = sample;
			}
			output = delay_element_0 * _b01 + _delay_element_11 * _b11 + _delay_element_21 * _b21;
			
			_delay_element_21 = _delay_element_11;
			_delay_element_11 = delay_element_0;

			return output;
		}
	}
	
protected:
	float sample_Freq;
	float cutoff_Freq;
	float _a11;
	float _a21;
	float _b01;
	float _b11;
	float _b21;
	float _delay_element_11;
	float _delay_element_21;
};


