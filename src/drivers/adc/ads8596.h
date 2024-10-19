#ifndef __ADS8596_H_
#define __ADS8596_H_

#include "ebus_spimaster.hpp"
#include "spiMasterCommand.hpp"
#include "drv_general_io.hpp"

class ads8596
{
public:
    ads8596() = delete;
    ads8596(ESAF::ebus_spimaster *spi, 
		ESAF::drv_general_io *cs, 
		ESAF::drv_general_io *convst,
		ESAF::drv_general_io *reset,
		ESAF::drv_general_io *range,
		ESAF::drv_general_io *standby);
    ~ads8596();
	
	void update();
	
public:
	float range_factor;
	float adc_v[6];

	int16_t channel_val[6];
	
	ESAF::ebus_spimaster *_spi;
	ESAF::drv_general_io *_cs;

	ESAF::drv_general_io *_convst;
	ESAF::drv_general_io *_reset;
	ESAF::drv_general_io *_range;
	ESAF::drv_general_io *_standby;
};

#endif
