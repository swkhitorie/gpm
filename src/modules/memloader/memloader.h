
#ifndef __MEMLOADER_H_
#define __MEMLOADER_H_
#ifdef __cplusplus
extern "C" {
#endif




#include "stdint.h"

	
void load_mem(void);
#ifdef MODULE_INFO_DEBUG
	void load_mem_debug(uint32_t *ZI_Base, uint32_t *ZI_len, uint32_t *RW_Base, uint32_t *RW_Len);
#endif






#ifdef __cplusplus
}
#endif
#endif
