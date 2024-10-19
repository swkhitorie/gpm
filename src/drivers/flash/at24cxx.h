#ifndef AT24CXX_H_
#define AT24CXX_H_

#include "ebus_i2cmaster.hpp"
#include "i2cMCmd.hpp"

class at24cxx : public i2cDevice
{
public:
    enum Type {
        AT24C01 = 0,
        AT24C02 = 1,
        AT24C04 = 2,
        AT24C08 = 3,
        AT24C16 = 4,
        AT24C32 = 5,
        AT24C64 = 6,
        AT24C128 = 7,
        AT24C256 = 8,
        AT24C512 = 9
    };
    at24cxx() = delete;
    at24cxx(ESAF::ebus_i2cmaster *i2c, uint8_t etype = AT24C256);
    ~at24cxx() = default;
    
    static constexpr uint16_t AT24CXX_ADDR = 0xA0;
    static constexpr uint16_t AT24CXX_TEST_ADDR_START = 0x00;
    static constexpr uint16_t AT24CXX_TEST_ADDR_LEN = sizeof(uint32_t);
    static constexpr uint32_t AT24CXX_TEST_DATA = ((4 << 24) | (3 << 16) | (2 << 8) | 1);
    static constexpr uint32_t AT24CXX_TEST_DATA2 = ((6 << 24) | (5 << 16) | (4 << 8) | 3);
    static constexpr uint16_t AT24CXX_USER_ADDR = AT24CXX_ADDR + AT24CXX_TEST_ADDR_LEN;
    
    bool rwtest();
    
    void wait();
    void reads(uint16_t addr, uint8_t *pbuff, uint16_t len);
    void writes(uint16_t addr, const uint8_t *pbuff, uint16_t len);
 
protected:

    void write(uint16_t addr, uint8_t data);
    uint8_t read(uint16_t addr, uint8_t check_data = 0x00, uint8_t times = 1);

    ESAF::ebus_i2cmaster *_i2c;
    uint8_t _type;
    uint16_t _addr_limit;

    uint8_t _write_array[5];
    uint8_t _write_len;
    uint8_t _read;
};











#endif
