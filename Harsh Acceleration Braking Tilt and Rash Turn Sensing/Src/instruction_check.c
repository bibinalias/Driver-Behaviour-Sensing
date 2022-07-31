#include "instruction_check.h"

void USER_Instruction_CHECK(void)
{
	extern volatile char instruction_final[25];
	if(!strncmp(instruction_final,"SET ",4))
	{
		USER_SET_Command();
	}
	if(!strncmp(instruction_final,"GET ",4))
	{
		USER_GET_Command();
	}

}
