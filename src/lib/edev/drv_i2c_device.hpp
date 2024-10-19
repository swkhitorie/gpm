#pragma once

#include <cstdint>

class i2cDevice
{
public:
	i2cDevice() : _healthy(false),  _updated(false), _update_us(0), _interval_us(0)
	{
	}
	uint64_t interval() { return _interval_us; }
	bool ishealthy() { return _healthy; }
	void updated(uint64_t time_us)
	{
		_updated = true;
		uint64_t current_us = time_us;
		_interval_us = current_us - _update_us;
		_update_us = current_us;
	}
protected:
	bool _healthy;
	bool _updated;
	uint64_t _update_us;
	uint64_t _interval_us;
};

