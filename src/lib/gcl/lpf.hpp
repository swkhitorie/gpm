#pragma once

#include <math.h>
#include "platform_defines.h"

class Lpf
{
public:
	Lpf() = default;
	~Lpf() = default;
	
	/**
	 * @brief 	Init the params of Filter
	 * @param _sample_freq the frequence of signal sample
	 * @param _cutoff_freq the cutoff frequence of signal sample
	*/
	void setcutoff_freq(float _sample_freq, float _cutoff_freq)
	{
		sample_Freq	= _sample_freq;
		cutoff_Freq = _cutoff_freq;
		dt = 1 / sample_Freq;
	}
	
	
	/**
	 * @brief 	put data into Filter, and output the lpf filter result
	 * @param sample the data will be input to lpf filter
	 * @return the result of lpf filter 
	*/
	float apply(float sample)
	{
		float result;
		float factor = ((1.0f / sample_Freq) * 2 * M_PI_F * cutoff_Freq);
		result = factor * sample + (1 - factor) * old_data;
		old_data = result;
		return result;
	}
	
protected:
	float cutoff_Freq;
	float sample_Freq;
	float old_data;
	float dt;
};

