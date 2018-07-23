/*
 * sim800_cmd_receiver.h
 *
 *  Created on: 19/07/2018
 *      Author: Korman Yuri
 */

#ifndef SIM800_CMD_RECEIVER_H_
#define SIM800_CMD_RECEIVER_H_
#include <stdint.h>
#include "sim800_cmd_transmiter.h"

void sim800_INIT(uint8_t data, send_command_flags *s_flags, successful_command_flags *sf_flags);
void sim800_WiatSMS(uint8_t data, send_command_flags *s_flags, successful_command_flags *sf_flags, uint8_t *buffer);
void sim800_AT_CMGR(uint8_t data, send_command_flags *s_flags, successful_command_flags *sf_flags, uint8_t *buffer, uint8_t *receiveDelay, uint8_t *receiveIMEI);
void sim800_AT_CMGD(uint8_t data, send_command_flags *s_flags, successful_command_flags *sf_flags);
void sim800_AT_CGSN(uint8_t data, send_command_flags *s_flags, successful_command_flags *sf_flags, uint8_t *imei, uint8_t *buffer);



#endif /* SIM800_CMD_RECEIVER_H_ */
