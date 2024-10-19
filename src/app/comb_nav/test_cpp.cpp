#include "test_cpp.h"

#include "llddebug.h"
test_obj::test_obj()
{
    value = 1;
    lldprint("call test_obj constructor %d\r\n", value);
}

test_obj::~test_obj()
{
    lldprint("call test_obj disconstructor %d\r\n", value);
}

void test_obj::test_1()
{
    value = 3;
    lldprint("call test_obj func %d\r\n", value);
}

