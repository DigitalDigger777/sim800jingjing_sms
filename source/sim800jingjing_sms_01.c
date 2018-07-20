/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    sim800jingjing_sms_01.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include <math.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "LPC824.h"
#include "fsl_debug_console.h"
#include "fsl_usart.h"

/* TODO: insert other include files here. */
#include "sim800_cmd_receiver.h"
#include "sim800_cmd_transmiter.h"


/* TODO: insert other definitions and declarations here. */
#define SIM800_USART USART1
#define SIM800_USART_CLK_SRC kCLOCK_MainClk
#define SIM800_USART_CLK_FREQ CLOCK_GetFreq(SIM800_USART_CLK_SRC)
#define SIM800_USART_IRQn USART1_IRQn

#define RX_BUFFER_SIZE 100

static void SIM800_USARTInit(void);
static void treatmenATCommand(uint8_t data);
static void sendATCommand();

uint8_t rxBuffer[RX_BUFFER_SIZE];
uint8_t moduleIMEI[15];
uint8_t receiveIMEI[15];

uint8_t delayForRelay;
uint8_t enableRelayFlag = 0U;

volatile uint8_t rxDataCounter = 0U;
volatile uint32_t g_systickCounter;

send_command_flags sendCommandFlags;
successful_command_flags successfulCommandFlags;
at_commands atCommands;

void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while(g_systickCounter != 0U)
    {
    }
}

static void treatmenATCommand(uint8_t data)
{

	//treatment init
	if(sendCommandFlags.INIT) {
		sim800_INIT(data, &sendCommandFlags, &successfulCommandFlags, &moduleIMEI, &rxBuffer);
	}

	if(sendCommandFlags.AT_CMGR) {
		sim800_AT_CMGR(data, &sendCommandFlags, &successfulCommandFlags, &rxBuffer, &delayForRelay, &receiveIMEI);
	}

	if(sendCommandFlags.AT_CMGD) {
		sim800_AT_CMGD(data, &sendCommandFlags, &successfulCommandFlags);
	}

	if(sendCommandFlags.CMTI) {
		sim800_WiatSMS(data, &sendCommandFlags, &successfulCommandFlags, &rxBuffer);
	}
}

static void sendATCommand()
{

	//if init successful
	if (successfulCommandFlags.INIT){
		//reset buffer
		memset(rxBuffer, 0, sizeof(rxBuffer));
		rxDataCounter = 0;

		//reset successful flag for init
		successfulCommandFlags.INIT = 0U;

		//module is ready light RDY diode
		GPIO_WritePinOutput(GPIO, 0U, BOARD_INITPINS_READY_LED_ID_PIN, 0U);

	    //next treatment command
	    sendCommandFlags.CMTI = 1U;
	}

	//if receive sms successful
	if (successfulCommandFlags.CMTI){
		//reset buffer
		memset(rxBuffer, 0, sizeof(rxBuffer));
		rxDataCounter = 0;

		//reset successful flag for init
		successfulCommandFlags.CMTI = 0U;
	    USART_WriteBlocking(SIM800_USART, atCommands.AT_CMGR, sizeof(atCommands.AT_CMGR));

	    //receive sms light DAT diode
	    GPIO_WritePinOutput(GPIO, 0U, BOARD_INITPINS_DAT_LED_ID_PIN, 0U);

	    //next treatment command
	    sendCommandFlags.AT_CMGR = 1U;
	}

	//if read sms successful
	if (successfulCommandFlags.AT_CMGR){
		//reset buffer
		memset(rxBuffer, 0, sizeof(rxBuffer));
		rxDataCounter = 0;

		//reset successful flag for init
		successfulCommandFlags.AT_CMGR = 0U;
	    USART_WriteBlocking(SIM800_USART, atCommands.AT_CMGD, sizeof(atCommands.AT_CMGD));

	    sendCommandFlags.AT_CMGD = 1U;
	}

	//if delete sms successful
	if (successfulCommandFlags.AT_CMGD){

		//reset buffer
		memset(rxBuffer, 0, sizeof(rxBuffer));
		rxDataCounter = 0;

		//reset successful flag for init
		successfulCommandFlags.AT_CMGD = 0U;

		//if imei equivalent open relay and wait next sms
		if (memcmp(moduleIMEI, receiveIMEI, sizeof(moduleIMEI)) == 0) {

			enableRelayFlag = 1U;
			GPIO_WritePinOutput(GPIO, 0U, BOARD_INITPINS_RELAY_ID_PIN, 1U);

			//off ERR diode
			GPIO_WritePinOutput(GPIO, 0U, BOARD_INITPINS_ERROR_LED_ID_PIN, 1U);
		} else {
		    //not equivalent imei of module and receive imei in sms light ERR diode
		    GPIO_WritePinOutput(GPIO, 0U, BOARD_INITPINS_ERROR_LED_ID_PIN, 0U);

		    //off DAT diode
		    GPIO_WritePinOutput(GPIO, 0U, BOARD_INITPINS_DAT_LED_ID_PIN, 1U);
		}

	    //next treatment command
	    sendCommandFlags.CMTI = 1U;
	}
}

void USART1_IRQHandler(void)
{
	uint8_t data;

    if (kUSART_RxReady & USART_GetStatusFlags(SIM800_USART))
    {
    	data = USART_ReadByte(SIM800_USART);
    	rxBuffer[rxDataCounter++] = data;
    	//PRINTF(data);

        while (!(kUSART_TxReady & USART_GetStatusFlags(SIM800_USART))){}

    	treatmenATCommand(data);
    	sendATCommand();
    }
}

/*
 * @brief   Application entry point.
 */
int main(void) {
    /* Enable clock of uart1. */
    CLOCK_EnableClock(kCLOCK_Uart1);

    /* Ser DIV of uart0. ??? */
    CLOCK_SetClkDivider(kCLOCK_DivUsartClk,1U);

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	if(SysTick_Config(SystemCoreClock / 1000U))
    {
        while(1)
        {
        }
    }

	/* init sim800 */

	InitSendCommandFlags(&sendCommandFlags);
	InitSuccessfulCommandFlags(&successfulCommandFlags);
	InitATCommands(&atCommands);

	SIM800_USARTInit();

    while(1) {

		if (enableRelayFlag) {
			PRINTF("Open relay...\r\n");

			SysTick_DelayTicks(delayForRelay * 60 * 1000);
			//reset flags
			enableRelayFlag = 0U;
			delayForRelay = 0U;

			GPIO_WritePinOutput(GPIO, 0U, BOARD_INITPINS_RELAY_ID_PIN, 0U);
			PRINTF("Close relay...\r\n");
			//off DAT diode
			GPIO_WritePinOutput(GPIO, 0U, BOARD_INITPINS_DAT_LED_ID_PIN, 1U);
		}


    }
    return 0 ;
}

static void SIM800_USARTInit(void)
{
    usart_config_t config;
    /* Default config by using USART_GetDefaultConfig():
     * config.baudRate_Bps = 9600U;
     * config.parityMode = kUSART_ParityDisabled;
     * config.stopBitCount = kUSART_OneStopBit;
     * config.bitCountPerChar = kUSART_8BitsPerChar;
     * config.loopback = false;
     * config.enableRx = false;
     * config.enableTx = false;
     * config.syncMode = kUSART_SyncModeDisabled;
     */
    USART_GetDefaultConfig(&config);
    config.enableRx = true;
    config.enableTx = true;
    config.baudRate_Bps = BOARD_DEBUG_USART_BAUDRATE;

    /* Initialize the USART with configuration. */
    USART_Init(SIM800_USART, &config, SIM800_USART_CLK_FREQ);

    /* Send init module command. */
    //USART_WriteBlocking(SIM800_USART, sim800ATCommands.AT, (sizeof(sim800ATCommands.AT) / sizeof(sim800ATCommands.AT[0])) - 1);
    //sendCommandFlags.AT = 1U;
    USART_WriteBlocking(SIM800_USART, atCommands.init, (sizeof(atCommands.init) / sizeof(atCommands.init[0])) - 1);
    sendCommandFlags.INIT = 1U;

    /* Enable USART RX ready interrupt. */
    USART_EnableInterrupts(SIM800_USART, kUSART_RxReadyInterruptEnable);
    //USART_EnableInterrupts(SIM800_USART, kUSART_TxReadyInterruptEnable);
    EnableIRQ(SIM800_USART_IRQn);
}
