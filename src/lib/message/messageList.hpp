
#ifndef __MESSAGELIST_H_
#define __MESSAGELIST_H_

#include <string.h>
#include "stdint.h"

namespace ESAF
{

/*! @brief message list framework
 * it use for module develop, responsible for the data and action interaction between different
 * modules
 */
class messageList
{
  public:
    /**
     * @brief message callback function defination
     * param -> data pointer
     */
    typedef void (*msgCallback)(const void *);

    /*! @brief message list node
     */
    typedef struct messageList_node
    {
        unsigned char mnemonic[16]; /*!< message list node name string */
        msgCallback pCall;          /*!< message list callback */
    } node_listType;

    messageList(int _capacity, msgCallback nullcallback) : capacity(_capacity), size(0)
    {
        node_nullcallback = nullcallback;
        node_list = new node_listType[_capacity];
        for (int i = 0; i < capacity; i++)
        {
            node_list[i].pCall = node_nullcallback;
            for (uint8_t j = 0; j < 15; j++)
                node_list[i].mnemonic[j] = '\0';
        }
    }

    ~messageList() { delete[] node_list; }

    int get_capacity(void) { return capacity; }
    int get_size(void) { return size; }

    /**
     * @brief advertise a message
     * @param pstr message name string
     * @param handle output message list node index
     * @return bool true - success, false - fail
     */
    bool advertise(const char *pstr, int &handle)
    {
        if (size >= capacity)
        {
            return false;
        }
        for (unsigned char i = 0; i < 16; i++)
            node_list[size].mnemonic[i] = pstr[i];
        node_list[size].mnemonic[15] = '\0';
        handle                       = size;
        size++;
        return true;
    }

    /**
     * @brief subscribe a message
     * @param pstr target message name string
     * @param msg_address message callback
     * @return bool true - success, false - fail
     */
    bool subscribe(const char *pstr, msgCallback msg_address)
    {
        int i = 0;
        for (; i < size; i++)
        {
            if (node_list[i].pCall != node_nullcallback)
                continue;
            else
            {
                if (!strcmp((const char *)node_list[i].mnemonic, pstr))
                {
                    node_list[i].pCall = msg_address;
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * @brief publish a message
     * @param handle target message list node index
     * @param pdata published data
     * @return bool true - success, false - fail
     */
    void publish(int handle, void *pdata) { (*node_list[handle].pCall)(pdata); }

  private:
    int capacity;
    int size;
    node_listType *node_list;
    msgCallback node_nullcallback;
};

}  // namespace ESAF

#endif  // messageList_H
