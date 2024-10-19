#ifdef __cplusplus
extern "C" {
#endif

#include "memloader.h"

void load_mem()
{
	extern uint32_t Image$$RW_IRAM1$$ZI$$Base;
	extern uint32_t Image$$RW_IRAM1$$ZI$$Length;
	
	extern uint32_t Image$$RW_IRAM1$$Base;
	extern uint32_t Load$$RW_IRAM1$$Base;
	extern uint32_t Image$$RW_IRAM1$$Length;
	
	uint32_t *drc;
	uint32_t *src;
	uint32_t length;
	
	/* init .data segment */
	drc = (uint32_t *)&Image$$RW_IRAM1$$Base;
	src = (uint32_t *)&Load$$RW_IRAM1$$Base;
	length = (uint32_t)&Image$$RW_IRAM1$$Length;	
	
	while (length--) {
		*drc++ = *src++;
	}
	
	/* clear .bss segment */
	drc = (uint32_t *)&Image$$RW_IRAM1$$ZI$$Base;
	length = (uint32_t)&Image$$RW_IRAM1$$ZI$$Length;	
	
	while (length--) { 
		*drc++ = 0;
	}
}

#ifdef MODULE_INFO_DEBUG
	void load_mem_debug(uint32_t *ZI_Base, uint32_t *ZI_len, uint32_t *RW_Base, uint32_t *RW_Len)
	{
		extern uint32_t Image$$RW_IRAM1$$ZI$$Base;
		extern uint32_t Image$$RW_IRAM1$$ZI$$Length;
		
		extern uint32_t Image$$RW_IRAM1$$Base;
		extern uint32_t Load$$RW_IRAM1$$Base;
		extern uint32_t Image$$RW_IRAM1$$Length;
		
		*ZI_Base = (uint32_t)&Image$$RW_IRAM1$$ZI$$Base;
		*ZI_len = (uint32_t)&Image$$RW_IRAM1$$ZI$$Length;
		
		*RW_Base = (uint32_t)&Image$$RW_IRAM1$$Base;
		*RW_Len = (uint32_t)&Image$$RW_IRAM1$$Length;
	}
#endif





#ifdef __cplusplus
}
#endif

