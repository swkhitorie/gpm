#include "at24cxx.h"

at24cxx::at24cxx(ESAF::ebus_i2cmaster *i2c, uint8_t etype):_i2c(i2c), _type(etype), _addr_limit((1 << etype) - 1) {}

bool at24cxx::rwtest()
{
    uint32_t r_data = 0x00;
    uint32_t test_data = AT24CXX_TEST_DATA2;
    
    writes(AT24CXX_ADDR, reinterpret_cast<uint8_t *>(&test_data), AT24CXX_TEST_ADDR_LEN);
    reads(AT24CXX_ADDR, reinterpret_cast<uint8_t *>(&r_data), AT24CXX_TEST_ADDR_LEN);
    if (r_data != test_data)
        return false;
    
    test_data = AT24CXX_TEST_DATA;
    
    writes(AT24CXX_ADDR, reinterpret_cast<uint8_t *>(&test_data), AT24CXX_TEST_ADDR_LEN);
    reads(AT24CXX_ADDR, reinterpret_cast<uint8_t *>(&r_data), AT24CXX_TEST_ADDR_LEN);
    if (r_data != test_data)
        return false; 
    
    return true;
}

void at24cxx::wait()
{
    for (int i = 0; i < 5000; i++);
}

void at24cxx::reads(uint16_t addr, uint8_t *pbuff, uint16_t len)
{
    //_i2c->set_regtype(I2C_MEMADD_SIZE_16BIT);
    while (len) {
        *pbuff++ = read(addr++);
        len--;
    }
    //_i2c->set_regtype(I2C_MEMADD_SIZE_8BIT);
}


void at24cxx::writes(uint16_t addr, const uint8_t *pbuff, uint16_t len)
{
    //_i2c->set_regtype(I2C_MEMADD_SIZE_16BIT);
    while (len) {
        write(addr, *pbuff);
        pbuff++;
        addr++;
        len--;
        wait();
    }
    //_i2c->set_regtype(I2C_MEMADD_SIZE_8BIT);
}

void at24cxx::write(uint16_t addr, uint8_t data)
{
    ESAF::i2cMCmd command;
    
    if (_type > AT24C02) {
        _write_array[0] = (uint8_t)((addr & 0xFF00) >> 8);
        _write_array[1] = (uint8_t)(addr & 0x00FF);
        _write_array[2] = data;
        _write_len = 3;
    } else {
        _write_array[0] = (uint8_t)(addr & 0x00FF);
        _write_array[1] = data;
        _write_len = 2;
    }

    command.set(AT24CXX_ADDR, &_write_array[0], _write_len, 0, 0, true, this, ESAF::i2cMCmd::MEMADD_SIZE_16BIT);
    _i2c->write_through(&command, 1, ESAF::EDev::BLOCK);
}

uint8_t at24cxx::read(uint16_t addr, uint8_t check_data, uint8_t times)
{
    do {
        
        ESAF::i2cMCmd command;
        
        if (_type > AT24C02) {
            _write_array[0] = (uint8_t)((addr & 0xFF00) >> 8);
            _write_array[1] = (uint8_t)(addr & 0x00FF);
            _write_len = 2;  
        } else {
            _write_array[0] = (uint8_t)(addr & 0x00FF);
            _write_len = 1;
        }

        command.set(AT24CXX_ADDR, &_write_array[0], _write_len, &_read, 1, true, this, ESAF::i2cMCmd::MEMADD_SIZE_16BIT);
        _i2c->read_through(&command, 1, ESAF::EDev::BLOCK);
        
        times--;
        
    } while(times && _read != check_data);
    
    return _read;  
}
