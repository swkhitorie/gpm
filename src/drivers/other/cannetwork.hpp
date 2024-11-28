
#ifndef __CANNETWORK_H_
#define __CANNETWORK_H_

#include "candriver.hpp"

#define MAX_CAN_NODE_NUM    (10)

class canNetwork
{
public:
    typedef struct device_node
    {
        canDriver *node;
        uint8_t prescaler;
        uint8_t cnt;
    } devnode_t;

    canNetwork() = delete;
    canNetwork(ESAF::ebus_can *bus, uint8_t buspre) : 
        _devnum(0),
        _busprescaler(buspre),
        _busprecnt(0),
        _bus(bus)
    {
        for (int i = 0; i < MAX_CAN_NODE_NUM; i++) {
            _dev[i].node = nullptr;
            _dev[i].prescaler = 0;
            _dev[i].cnt = 0;
        }
    }
    ~canNetwork() {}

    void enable()
    {
        for (int i = 0; i < _devnum; i++)
            _dev[i].node->enable();
    }
    void disable()
    {
        for (int i = 0; i < _devnum; i++)
            _dev[i].node->disable();
    }

    uint8_t getdevnum() { return _devnum; }
    void setbuspre(uint8_t pre) { _busprescaler = pre; }
    void setnodepre(uint8_t pre) 
    {    
        for (int i = 0; i < _devnum; i++)
            _dev[i].prescaler = pre;
    }
    void setnodepre(uint8_t pre, uint8_t index) { _dev[index].prescaler = pre; }

    bool devadd(canDriver *node)
    {
        if (_devnum == MAX_CAN_NODE_NUM)
            return false;

        _dev[_devnum].node = node;
        _devnum++;
        return true;
    }

    bool stepbus()
    {
        if (++_busprecnt > _busprescaler) {
            _busprecnt = 1;
            return true;
        } else {
            return false;
        }
    }

    void stepnode_exec(uint8_t index)
    {
        if (++_dev[index].cnt > _dev[index].prescaler) {
            _dev[index].cnt = 1;
            _dev[index].node->poll_operation();
        }
    }

    void poll()
    {
        if (stepbus()) {
            ESAF::CANFrame rxMsg;
            while (_bus->read(&rxMsg, 1) == ESAF::EDev::EOK) {
                for (int i = 0; i < _devnum; i++) {
                    _dev[i].node->poll_recvparser(&rxMsg);
                }
            }

            if (_bus->esize(ESAF::EDev::STREAMOUT) > 0) {
                _bus->update();
            } else {
                for (int i = 0; i < _devnum; i++)
                    stepnode_exec(i);
            }
        }
    }

public:
    uint8_t _devnum;
    uint8_t _busprescaler;
    uint8_t _busprecnt;

    ESAF::ebus_can *_bus;
    device_node _dev[MAX_CAN_NODE_NUM];
};

#endif
