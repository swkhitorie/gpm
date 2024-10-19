#pragma once

#include <math.h>

template<typename T, uint16_t size>
class MoveAverageFilter
{
public:
	MoveAverageFilter() = default;
	~MoveAverageFilter() = default;
	
	T get(T _input)
	{
		T sum = 0;
		sample[index] = _input;
		index++;
		if (index == size)
			index = 0;
		for (int i = 0; i < size; i++) {
			sum += sample[i];
		}
		return (sum / size);
	}
	
	T get_antipulse(T _input)
	{
		T sum = 0;
		T max = 0;
		T min = 0;
		if (number == 0) {
			min = _input;
			max = _input;
		}
		sample[index] = _input;
		index++;
		if (index == size)
			index = 0;
		for (int i = 0; i < size; i++) {
			if (sample[i] > max) max = sample[i];
			else if(sample[i] < min) min = sample[i];
			sum += sample[i];
		}
		return (sum - max - min) / (size - 2);
	}
	
	
protected:
	T input;
	T sample[size];
	
	uint16_t number;
	uint16_t index;
};

