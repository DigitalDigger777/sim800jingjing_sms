/*
 * sim800_cmd_transmiter.h
 *
 *  Created on: 19/07/2018
 *      Author: Korman Yuri
 */

#ifndef SIM800_CMD_TRANSMITER_H_
#define SIM800_CMD_TRANSMITER_H_


#include <stdint.h>

typedef struct _send_command_flags{
	uint8_t AT_CMGR;
	uint8_t AT_CMGD;
	uint8_t CMTI;
	uint8_t INIT;
}send_command_flags;

typedef struct _mqtt_send_command_flags{
	uint8_t CONNECT;
	uint8_t SUBSCRIBE;
}mqtt_send_command_flags;

typedef struct _successful_command_flags{
	uint8_t AT_CMGR;
	uint8_t AT_CMGD;
	uint8_t CMTI;
	uint8_t INIT;
}successful_command_flags;


typedef struct _at_commands{
	uint8_t init[42];
	uint8_t AT_CMGD[12];
	uint8_t AT_CMGR[12];
}at_commands;


void InitSendCommandFlags(send_command_flags *flags);
void InitSuccessfulCommandFlags(successful_command_flags *flags);
void InitATCommands(at_commands *commands);

#endif /* SIM800_CMD_TRANSMITER_H_ */
