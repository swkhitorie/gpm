
#ifndef _MESSAGE_POOL_H_
#define _MESSAGE_POOL_H_

#include <string.h>
#include "message_telemetry.hpp"

namespace ESAF
{

/*! @brief message pool framework
 * it use for module develop, responsible for the data interaction between different modules
 */
class messagePool
{
  public:
    /**
     * @brief message pool node
     */
    typedef struct messagePool_node
    {
        unsigned char mnemonic[15]; /*!< message pool node name string */
        unsigned char type_size;    /*!< message pool node data type size */
        const void *pdata;          /*!< message pool node data pointer */
    } node_type;

    messagePool(int _capacity) : capacity(_capacity), size(0)
    {
        node_pool = new node_type[_capacity];
        for (int i = 0; i < capacity; i++)
        {
            node_pool[i].pdata     = NULL;
            node_pool[i].type_size = 4;
            for (uint8_t j = 0; j < 15; j++)
                node_pool[i].mnemonic[j] = '\0';
        }
    }

    ~messagePool() { delete[] node_pool; }

    int get_capacity(void) { return capacity; }
    int get_size(void) { return size; }

    /**
     * @brief regist a message node to message pool
     * @param data_address data address
     * @param type_size data type size
     * @param pstr message node name string
     * @return bool true - success, false - fail
     */
    bool regist(void *data_address, unsigned char type_size, const char *pstr)
    {
        if (size >= capacity || node_pool[size].pdata != NULL)
            return false;

        node_pool[size].type_size = type_size;
        node_pool[size].pdata     = data_address;
        for (unsigned char i = 0; i < 15; i++)
            node_pool[size].mnemonic[i] = pstr[i];
        node_pool[size].mnemonic[14] = '\0';
        size++;
        return true;
    }

    /**
     * @brief bind a data pointer into pool data
     * @param target target message pool name string
     * @param __pdata address of user data pointer
     * @return bool true - success, false - fail
     */
    bool bind(const char *target, const void **__pdata)
    {
        unsigned char i = 0;
        for (; i < size; i++)
        {
            if (node_pool[i].pdata == NULL)
                continue;
            else
            {
                if (!strcmp((const char *)node_pool[i].mnemonic, target))
                {
                    *__pdata = node_pool[i].pdata;
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief get index of message pool by name string
     * @param pstr target message pool name string
     * @param index output index of target message pool node
     * @return bool true - success, false - fail
     */
    bool getHandle(const char *pstr, int &index)
    {
        unsigned char i = 0;
        for (; i < size; i++)
        {
            if (node_pool[i].pdata == NULL)
                continue;
            else
            {
                if (!strcmp((const char *)node_pool[i].mnemonic, pstr))
                {
                    index = i;
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief read data from message pool by name string
     * @param target target message pool name string
     * @param __pdata output data
     * @return bool true - success, false - fail
     */
    bool read(const char *target, void *__pdata)
    {
        unsigned char i = 0, j = 0;
        const unsigned char *src = NULL;
        unsigned char *dst       = static_cast<unsigned char *>(__pdata);
        for (; i < size; i++)
        {
            if (node_pool[i].pdata == NULL)
                continue;
            else
            {
                if (!strcmp((const char *)node_pool[i].mnemonic, target))
                {
                    src = static_cast<const unsigned char *>(node_pool[i].pdata);
                    for (j = 0; j < node_pool[i].type_size; j++)
                        *dst++ = *src++;
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief read data from message pool by list index
     * @param handle target message pool index
     * @param __pdata output data
     * @return bool true - success, false - fail
     */
    bool read(unsigned short handle, void *__pdata)
    {
        unsigned char i          = 0;
        const unsigned char *src = static_cast<const unsigned char *>(node_pool[handle].pdata);
        unsigned char *dst       = static_cast<unsigned char *>(__pdata);

        if (node_pool[handle].pdata == NULL)
            return false;

        for (; i < node_pool[handle].type_size; i++)
            *dst++ = *src++;
        return true;
    }

  private:
    int capacity;
    int size;
    node_type *node_pool;
};

}  // namespace ESAF

#endif  // messagePool_H
