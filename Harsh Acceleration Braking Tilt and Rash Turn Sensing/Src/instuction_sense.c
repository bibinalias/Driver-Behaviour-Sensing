#include "instuction_sense.h"
#include "stdint.h"
void USER_Instruction_Sense(void)
{
	extern volatile uint8_t uartrx_req,i;
	extern volatile char instruction_raw[25],c;
	if(uartrx_req==0&&i<25)
	{
		uartrx_req=1;
		strncpy(instruction_raw+i,&c,1);
		// instruction_raw[i]=c;
		i++;

	}
}
