#pragma once

#include "parameters_types.hpp"
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

template<typename T, UParams::parnodetype p>
class Param {};

template<UParams::parnodetype p>
class Param<float, p>
{
public:
	static_assert(__parnodepodtype[(int)p] == UParams::PARAM_TYPE_FLOAT, "parameter type must be float");
	
	Param() 
    {
        update();
    }
	~Param() = default;

    bool commit() const { return UParams::param_set(handle(), &_val) == 0; }
    
    bool commit_no_notification() const { return UParams::param_set_no_notification(handle(), &_val) == 0; }

	bool commit_no_notification(float val)
	{
		if (fabsf(val - _val) > FLT_EPSILON) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}
    
	const float &reference() const { return _val; }
	
	float get() const { return _val; }
	
	void set(float val) { _val = val; }
    
	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}
    
    bool update() { return UParams::param_get(handle(), &_val) == 0; }
    
    UParams::param_t handle() const { return UParams::param_handle(p); }
private:
	float _val;
};

template<UParams::parnodetype p>
class Param<float &, p>
{
public:
    static_assert(__parnodepodtype[(int)p] == UParams::PARAM_TYPE_FLOAT, "parameter type must be float");
	
	Param(float &external_val)
		: _val(external_val)
    {
        update();
    }
	~Param() = default;

    bool commit() const { return UParams::param_set(handle(), &_val) == 0; }
    
    bool commit_no_notification() const { return UParams::param_set_no_notification(handle(), &_val) == 0; }

	bool commit_no_notification(float val)
	{
		if (fabsf(val - _val) > FLT_EPSILON) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}
    
	const float &reference() const { return _val; }
	
	float get() const { return _val; }
	
	void set(float val) { _val = val; }
    
	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}
    
    bool update() { return UParams::param_get(handle(), &_val) == 0; }
    
    UParams::param_t handle() const { return UParams::param_handle(p); }
private:
	float &_val;
};

template<UParams::parnodetype p>
class Param<int32_t, p>
{
public:
    static_assert(__parnodepodtype[(int)p] == UParams::PARAM_TYPE_INT32, "parameter type must be int32_t");

	Param() 
    {
        update();
    }
	~Param() = default;

    bool commit() const { return UParams::param_set(handle(), &_val) == 0; }
    
    bool commit_no_notification() const { return UParams::param_set_no_notification(handle(), &_val) == 0; }

	bool commit_no_notification(int32_t val)
	{
		if (val != _val) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}
    
	const int32_t &reference() const { return _val; }
	
	int32_t get() const { return _val; }
	
	void set(int32_t val) { _val = val; }
    
	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}
    
    bool update() { return UParams::param_get(handle(), &_val) == 0; }
    
    UParams::param_t handle() const { return UParams::param_handle(p); }
private:
	int32_t _val;
};

template<UParams::parnodetype p>
class Param<int32_t &, p>
{
public:
	static_assert(__parnodepodtype[(int)p] == UParams::PARAM_TYPE_INT32, "parameter type must be int32_t");
	
	Param(int32_t &external_val)
		: _val(external_val)
    {
        update();
    }
	~Param() = default;

    bool commit() const { return UParams::param_set(handle(), &_val) == 0; }
    
    bool commit_no_notification() const { return UParams::param_set_no_notification(handle(), &_val) == 0; }

	bool commit_no_notification(int32_t val)
	{
		if (val != _val) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}
    
	const int32_t &reference() const { return _val; }
	
	int32_t get() const { return _val; }
	
	void set(int32_t val) { _val = val; }
    
	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}
    
    bool update() { return UParams::param_get(handle(), &_val) == 0; }
    
    UParams::param_t handle() const { return UParams::param_handle(p); }
private:
	int32_t &_val;
};

template<UParams::parnodetype p>
class Param<bool, p>
{
public:
	static_assert(__parnodepodtype[(int)p] == UParams::PARAM_TYPE_INT32, "parameter type must be int32_t");
	
	Param() 
    {
        update();
    }
	~Param() = default;

    bool commit() const { return UParams::param_set(handle(), &_val) == 0; }
    
    bool commit_no_notification() const { return UParams::param_set_no_notification(handle(), &_val) == 0; }

	bool commit_no_notification(bool val)
	{
		if (val != _val) {
			set(val);
			commit_no_notification();
			return true;
		}

		return false;
	}
    
	const bool &reference() const { return _val; }
	
	bool get() const { return _val; }
	
	void set(bool val) { _val = val; }
    
	void reset()
	{
		param_reset_no_notification(handle());
		update();
	}
    
    bool update() 
    { 
        int32_t value_int;
        int ret = UParams::param_get(handle(), &value_int);
        
        if (ret == 0) {
            _val = value_int != 0;
            return true;
        }
        
        return false;
    }
    
    UParams::param_t handle() const { return UParams::param_handle(p); }
	
private:
	bool _val;
};

template <UParams::parnodetype p>
using ParamFloat = Param<float, p>;

template <UParams::parnodetype p>
using ParamInt = Param<int32_t, p>;

template <UParams::parnodetype p>
using ParamExtFloat = Param<float &, p>;

template <UParams::parnodetype p>
using ParamExtInt = Param<int32_t &, p>;

template <UParams::parnodetype p>
using ParamBool = Param<bool, p>;

/* 
    ##### declare #####
    ParamFloat<UParams::INS_ACCEL_FILTER> _param_imu_accel_filter_cutoff;

    ##### update/commit #####
    if (_param_imu_accel_filter_cutoff.update()) {} else {}
    if (_param_imu_accel_filter_cutoff.commit()) {} else {}

    ##### get/set #####
    float tmp = _param_imu_accel_filter_cutoff.get();
    _param_imu_accel_filter_cutoff.set(3.1f);
*/

