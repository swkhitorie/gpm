#include "ads1220l.h"

ads1220l::ads1220l(ESAF::ebus_spimaster *spi, ESAF::drv_general_io *cs) :
    _spi(spi), _cs(cs)
{
    _config = 0;
    _idx = 0; 
	channel_val[0] = 1e-7f;
    channel_val[1] = 1e-7f;
    channel_val[2] = 1e-7f;
    channel_val[3] = 1e-7f;
    _rbuf[0] = 0;
    _rbuf[1] = 0;
}

void ads1220l::init(uint16_t _ch, uint16_t pga, uint16_t sps)
{
    uint8_t txlist[2];
    uint16_t reg =  ADS1118_SS_START | _ch | pga | ADS1118_Continuous_MODE | sps | ADS1118_ADC_MODE | ADS1118_PUUP_EN | ADS1118_NOP_UPDATA;
    txlist[0]= reg >> 8;
    txlist[1] = reg;

    _cs->set(false);
    _spi->write(&txlist[0], 2);
    _cs->set(true);
}

float ads1220l::get(uint16_t _ch, uint16_t pga, uint16_t sps)
{
    int16_t _adc_value = 0;
    float _tdata = 0;

    uint8_t txlist[2];
    uint16_t reg =  ADS1118_SS_START | _ch | pga | ADS1118_Continuous_MODE | sps | ADS1118_ADC_MODE | ADS1118_PUUP_EN | ADS1118_NOP_UPDATA;
    txlist[0]= reg >> 8;
    txlist[1] = reg;

    _cs->set(false);
    _spi->handle_inout(&txlist[0], &_rbuf[0], 2);
    _cs->set(true);
    
    _adc_value = (_rbuf[0] << 8) | _rbuf[1];
    
    if (_adc_value & 0X8000) {
        _adc_value = (~_adc_value)+1;
    }
    
    if (pga == ADS1118_PGA_6144)     _tdata = _adc_value * 0.1875f;
    else if(pga == ADS1118_PGA_4096) _tdata = _adc_value * 0.125f;
    else if(pga == ADS1118_PGA_2048) _tdata = _adc_value * 0.0625f;
    else if(pga == ADS1118_PGA_1024) _tdata = _adc_value * 0.03125f;
    else if(pga == ADS1118_PGA_0512) _tdata = _adc_value * 0.015625f;
    else                             _tdata = _adc_value*0.0078125;
    return _tdata / 1000.0f;
}

void ads1220l::update()
{
    if (++_idx > 4)
        _idx = 0;
    
    switch (_idx) {
    case 0: channel_val[0] = get(ADS1118_MUX_AIN0, ADS1118_PGA_6144, ADS1118_DR_128SPS); break;
    case 1: channel_val[2] = get(ADS1118_MUX_AIN1, ADS1118_PGA_6144, ADS1118_DR_128SPS); break;
    case 2: channel_val[3] = get(ADS1118_MUX_AIN2, ADS1118_PGA_6144, ADS1118_DR_128SPS); break;
    case 3: channel_val[1] = get(ADS1118_MUX_AIN3, ADS1118_PGA_6144, ADS1118_DR_128SPS); break;            
    } 
}
