/*
 * sim800_cmd_transmiter.c
 *
 *  Created on: 19/07/2018
 *      Author: Korman Yuri
 */

#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "sim800_cmd_transmiter.h"

void InitSendCommandFlags(send_command_flags *flags)
{
	assert(NULL != flags);

	flags->INIT = 0U;
	flags->AT_CMGR = 0U;
}

void InitSuccessfulCommandFlags(successful_command_flags *flags)
{
	assert(NULL != flags);

	flags->AT_CMGR = 0U;
	flags->INIT = 0U;
}

void InitATCommands(at_commands *commands)
{
	assert(NULL != commands);

	strcpy(commands->init, "ATV0E0+CMGD=1,4;+CMGF=1;+IPR=9600;+CGSN\r\n");
	strcpy(commands->AT_CMGR, "AT+CMGR=1\r\n");
	strcpy(commands->AT_CMGD, "AT+CMGD=1\r\n");
}
