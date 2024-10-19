
#ifndef __SERVICELIST_H_
#define __SERVICELIST_H_

#include <string.h>
#include "stdint.h"

namespace ESAF
{

/*! @brief service list framework
 * it use for module develop, responsible for the action interaction between different modules
 */
class serviceList
{
  public:
    /**
     * @brief service callback function defination
     * two param, the first is request
     * the last is response
     */
    typedef void (*srvCallback)(void *, void *);

    /*! @brief service node
     */
    typedef struct srv_node
    {
        unsigned char mnemonic[16]; /*!< service name string */
        srvCallback pcall;          /*!< service callback pointer */
    } srv_node;

    serviceList(int _capacity, srvCallback nullcallback) : capacity(_capacity), size(0)
    {
        node_nullcallback = nullcallback;
        node_list = new srv_node[_capacity];
        for (int i = 0; i < capacity; i++)
        {
            node_list[i].pcall = node_nullcallback;
            for (uint8_t j = 0; j < 16; j++)
                node_list[i].mnemonic[j] = '\0';
        }
    }

    ~serviceList() { delete[] node_list; }

    /**
     * @brief regist a service to service list
     * @param srv_address service callback
     * @param pstr service name
     * @return bool true - success, false - fail
     */
    bool regist(srvCallback srv_address, const char *pstr)
    {
        if (size >= capacity)
        {
            return false;
        }
        node_list[size].pcall = srv_address;
        for (unsigned char i = 0; i < 15; i++)
            node_list[size].mnemonic[i] = pstr[i];
        node_list[size].mnemonic[14] = '\0';
        size++;
        return true;
    }

    /**
     * @brief get service callback pointer by service name
     * @param _to_bind service callback will be bind
     * @param pstr service name
     * @return bool true - success, false - fail
     */
    bool bind(const char *pstr, srvCallback &_to_bind)
    {
        unsigned char i = 0;
        for (; i < size; i++)
        {
            if (node_list[i].pcall == node_nullcallback)
                continue;
            else
            {
                if (!strcmp((const char *)node_list[i].mnemonic, pstr))
                {
                    _to_bind = node_list[i].pcall;
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief get service callback index by service name
     * @param index service callback index in service list
     * @param pstr service name
     * @return bool true - success, false - fail
     */
    bool getHandle(const char *pstr, int &index)
    {
        unsigned char i = 0;
        for (; i < size; i++)
        {
            if (node_list[i].pcall == node_nullcallback)
                continue;
            else
            {
                if (!strcmp((const char *)node_list[i].mnemonic, pstr))
                {
                    index = i;
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief call service by list index
     * @param index service callback index in service list
     * @param request request of service
     * @param response response of service
     */
    void call(unsigned short index, void *request, void *response)
    {
        if (node_list[index].pcall != node_nullcallback)
            (*node_list[index].pcall)(request, response);
    }

  private:
    int capacity;
    int size;
    srv_node *node_list;
    srvCallback node_nullcallback;
};

}  // namespace ESAF

#endif  // serviceList_H
