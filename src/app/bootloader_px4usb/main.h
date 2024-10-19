#ifndef _MAIN_H_
#define _MAIN_H_

// #include <memory.h>

#if defined ( __clang__ )
	__attribute__((section (".RAM_D1"))) short tmp_value_1;
	void prevent_warning()
	{
		tmp_value_1 = 0;
	}
#endif

#ifdef __cplusplus
extern "C" {
#endif
    
    extern void LoadFirmware(void);
    
#ifdef __cplusplus
}
#endif

#endif
