/*
 * sim800_cmd_receiver.c
 *
 *  Created on: 19/07/2018
 *      Author: Korman Yuri
 */

#include "sim800_cmd_receiver.h"
#include "sim800_cmd_transmiter.h"

uint8_t waitOK = 0; //wait ok
uint8_t waitError = 0; //wait error
uint8_t waitNL = 0; //wait new line
uint8_t waitStatus = 0;
uint8_t waitConnectionStatus = 0;


void sim800_INIT(uint8_t data, send_command_flags *s_flags, successful_command_flags *sf_flags)
{
    if(0 == waitOK && 48 == data){
    	waitOK++;
    } else if(1 == waitOK && 13 == data) {
    	waitOK++;
    } else if(2 == waitOK && 10 == data) {
    	waitOK++;
    } else {
    	waitOK = 0;
    }

    //end receive
    if (waitOK == 3){

    	//reset flag for send command
    	s_flags->INIT = 0U;
    	sf_flags->INIT = 1U;

    	waitOK = 0;
    }


    //Error
    if(0 == waitError && 52 == data){
    	waitError++;
    } else if(1 == waitError && 13 == data) {
    	waitError++;
    } else if(2 == waitError && 10 == data) {
    	waitError++;
    } else {
    	waitError = 0;
    }

    //end receive
    if (waitError == 3){

    	//reset flag for send command
    	s_flags->INIT = 0U;
    	sf_flags->INIT = 1U;

    	waitError = 0;
    }
}

void sim800_AT_CMGR(uint8_t data, send_command_flags *s_flags, successful_command_flags *sf_flags, uint8_t *buffer, uint8_t *receiveDelay, uint8_t *receiveIMEI)
{
    if(0 == waitOK && 48 == data){
    	waitOK++;
    } else if(1 == waitOK && 13 == data) {
    	waitOK++;
    } else if(2 == waitOK && 10 == data) {
    	waitOK++;
    } else {
    	waitOK = 0;
    }

    //end receive
    if (waitOK == 3){
    	s_flags->AT_CMGR = 0U;
    	sf_flags->AT_CMGR = 1U;

    	int i = 100;
    	uint8_t startReadTimeUnits = 0U;
    	uint8_t startReadTimeIMEI = 0U;
    	uint8_t timeUnits = 109;
    	uint8_t timeValue = 0;
    	uint8_t imei[15];
    	uint8_t imeiCounter = 14;
    	uint8_t digid = 0;

    	while(i >= 0) {
    		if (startReadTimeUnits) {

    			if (buffer[i] == 124) {
    				startReadTimeUnits = 0U;
    				startReadTimeIMEI = 1U;
    				continue;
    			}

    			if (digid == 0 && buffer[i] >= 48 && buffer[i] <= 57) {

    				timeValue = buffer[i] - 48;
    				digid = 10;

    			} else if (buffer[i] >= 48 && buffer[i] <= 57) {

    				timeValue = (digid * (buffer[i] - 48)) + timeValue;
    				digid = digid * digid;
    			}

    		}

    		if (startReadTimeIMEI) {
    			if (buffer[i] >= 48 && buffer[i] <= 57) {

    				imei[imeiCounter] = buffer[i];

    				if (imeiCounter == 0) {
    					startReadTimeIMEI = 0U;
    				}
    				imeiCounter--;
    			}
    		}

    		if (buffer[i] == 109 || buffer[i] == 104) {
    			startReadTimeUnits = 1U;
    			timeUnits = buffer[i];
    		}

    		i--;
    	}

    	(*receiveDelay) = timeValue;
    	memcpy(receiveIMEI, imei, 15);
    	waitOK = 0;
    }

    //Error
    if(0 == waitError && 52 == data){
    	waitError++;
    } else if(1 == waitError && 13 == data) {
    	waitError++;
    } else if(2 == waitError && 10 == data) {
    	waitError++;
    } else {
    	waitError = 0;
    }

    //end receive
    if (waitError == 3){

    	//reset flag for send command
    	s_flags->AT_CMGR = 0U;
    	sf_flags->AT_CMGR = 1U;

    	waitError = 0;
    }
}

void sim800_WiatSMS(uint8_t data, send_command_flags *s_flags, successful_command_flags *sf_flags, uint8_t *buffer)
{
	//+  C  M  T  I
	//43 67 77 84 73 58 32 34 83 77 34 44
    uint8_t pattern[5] = {43, 67, 77, 84, 73};

	if(0 == waitOK && 13 == data){
    	waitOK++;
    } else if(1 == waitOK && 10 == data) {
    	waitOK++;
    } else {
    	waitOK = 0;
    }

    //end receive
    if (waitOK == 2){

    	int n = memcmp(pattern, buffer, sizeof(pattern));
    	if (n == 0) {
        	s_flags->CMTI = 0U;
        	sf_flags->CMTI = 1U;
    		waitOK = 0;
    	}

    }

}

void sim800_AT_CMGD(uint8_t data, send_command_flags *s_flags, successful_command_flags *sf_flags)
{
    if(0 == waitOK && 48 == data){
    	waitOK++;
    } else if(1 == waitOK && 13 == data) {
    	waitOK++;
    } else if(2 == waitOK && 10 == data) {
    	waitOK++;
    } else {
    	waitOK = 0;
    }

    //end receive
    if (waitOK == 3){
    	s_flags->AT_CMGD = 0U;
    	sf_flags->AT_CMGD = 1U;

    	waitOK = 0;
    }

    //Error
    if(0 == waitError && 52 == data){
    	waitError++;
    } else if(1 == waitError && 13 == data) {
    	waitError++;
    } else if(2 == waitError && 10 == data) {
    	waitError++;
    } else {
    	waitError = 0;
    }

    //end receive
    if (waitError == 3){

    	//reset flag for send command
    	s_flags->AT_CMGD = 0U;
    	sf_flags->AT_CMGD = 1U;

    	waitError = 0;
    }
}

void sim800_AT_CGSN(uint8_t data, send_command_flags *s_flags, successful_command_flags *sf_flags, uint8_t *imei, uint8_t *buffer)
{
    if(0 == waitOK && 48 == data){
    	waitOK++;
    } else if(1 == waitOK && 13 == data) {
    	waitOK++;
    } else if(2 == waitOK && 10 == data) {
    	waitOK++;
    } else {
    	waitOK = 0;
    }

    //end receive
    if (waitOK == 3){
    	int i = 0;

    	while(i < 15){
    		imei[i] = buffer[i];
    		i++;
    	}

    	s_flags->AT_CGSN = 0U;
    	sf_flags->AT_CGSN = 1U;

    	waitOK = 0;
    }

    //Error
    if(0 == waitError && 52 == data){
    	waitError++;
    } else if(1 == waitError && 13 == data) {
    	waitError++;
    } else if(2 == waitError && 10 == data) {
    	waitError++;
    } else {
    	waitError = 0;
    }

    //end receive
    if (waitError == 3){

    	//reset flag for send command
    	s_flags->AT_CGSN = 0U;
    	sf_flags->AT_CGSN = 1U;

    	waitError = 0;
    }
}
