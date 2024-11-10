#include "test_cpp.h"
#include "platforms_common.h"

test_obj::test_obj()
{
    value = 1;
    print("call test_obj constructor %d\r\n", value);
}

test_obj::~test_obj()
{
    print("call test_obj disconstructor %d\r\n", value);
}

void test_obj::test_1()
{
    value = 3;
    print("call test_obj func %d\r\n", value);
}

