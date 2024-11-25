/**
 * @file Sensor.hpp
 * Abstract class for sensors
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */

#pragma once

#include "common.h"

namespace estimator
{
namespace sensor
{

class Sensor
{
public:
	virtual ~Sensor() {};

	/*
	 * run sanity checks on the current data
	 * this has to be called immediately after
	 * setting new data
	 */
	virtual void runChecks(){};

	/*
	 * return true if the sensor is healthy
	 */
	virtual bool isHealthy() const = 0;

	/*
	 * return true if the delayed sample is healthy
	 * and can be fused in the estimator
	 */
	virtual bool isDataHealthy() const = 0;
};

} // namespace sensor
} // namespace estimator
