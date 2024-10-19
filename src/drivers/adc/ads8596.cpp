#include "ads8596.h"

ads8596::ads8596(ESAF::ebus_spimaster *spi, 
	ESAF::drv_general_io *cs, 
	ESAF::drv_general_io *convst,
	ESAF::drv_general_io *reset,
	ESAF::drv_general_io *range,
	ESAF::drv_general_io *standby) :
	_spi(spi),
	_cs(cs),
	_convst(convst),
	_reset(reset),
	_range(range),
	_standby(standby) 
{
	for (uint8_t i = 0; i < 6; i++)
		channel_val[i] = 0;
	
	_convst->set(true);
	
	_reset->set(true);
	
	for (uint32_t i = 0; i < 10000; i++);
	
	_reset->set(false);
}
ads8596::~ads8596() {}

void ads8596::update()
{
	uint8_t recv[3];
	_range->set(true);
	range_factor = (2 * 10.0f / 65535);
	_reset->set(false);
	_standby->set(true);
	
	_convst->set(false);
	for (uint32_t i = 0; i < 500; i++);
	_convst->set(true);
	for (uint32_t i = 0; i < 1000; i++);

	_cs->set(false);
	
	_spi->read((uint8_t *)&channel_val[0], 6);
	for (int i = 0; i < 6; i++)
		adc_v[i] = (float)(channel_val[i] * range_factor);
	
	_cs->set(true);
}


