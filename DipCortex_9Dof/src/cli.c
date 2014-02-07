/*
  ____        _     _           ____        _           _		 _          _
 / ___|  ___ | | __| | ___ _ __/ ___| _ __ | | __ _ ___| |__	| |    __ _| |__  ___
 \___ \ / _ \| |/ _` |/ _ \ '__\___ \| '_ \| |/ _` / __| '_ \	| |   / _` | '_ \/ __|
  ___) | (_) | | (_| |  __/ |   ___) | |_) | | (_| \__ \ | | |	| |__| (_| | |_) \__ \
 |____/ \___/|_|\__,_|\___|_|  |____/| .__/|_|\__,_|___/_| |_|	|_____\__,_|_.__/|___/
                                     |_|
 (C)SolderSplash Labs 2013 - www.soldersplash.co.uk - C. Matthews - R. Steel


	@file     cli.c
	@author   Carl Matthews (soldersplash.co.uk)
	@date     01 May 2013

    @section LICENSE

	Software License Agreement (BSD License)

    Copyright (c) 2013, C. Matthews - R. Steel (soldersplash.co.uk)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    @section DESCRIPTION

    Command line interface, each function is added to a list in the header file, when the user enter a command that matches on in the list
    that function is sent the parameters

*/

#include <string.h>
#include <stdio.h>
#include <time.h>

#include "SolderSplashLpc.h"
#include "console.h"

#define _CLI_
#include "cli.h"
// ------------------------------------------------------------------------------------------------------------
/*!
    @brief CLI_Init -
*/
// ------------------------------------------------------------------------------------------------------------
void CLI_Init ( void )
{
	// Give the console our command list
	Console_Init( ( CONSOLE_CMDS_STRUCT *)&ConsoleCommands );
}

// ------------------------------------------------------------------------------------------------------------
/*!
    @brief CLI_Help
*/
// ------------------------------------------------------------------------------------------------------------
int CLI_Help (int argc, char **argv)
{
uint8_t i = 0;

	ConsolePrintf("\r\n");

	i = 0;
	while (ConsoleCommands[i].Command)
	{
		ConsolePrintf("%15s \t: %s\r\n", ConsoleCommands[i].Command, ConsoleCommands[i].help);
		i++;
	}

	return(1);
}


// ------------------------------------------------------------------------------------------------------------
/*!
    @brief CLI_9DofInit
*/
// ------------------------------------------------------------------------------------------------------------
int CLI_9DofInit (int argc, char **argv)
{
	LSM330_Init();
	ConfigureMAG3110();
	return(1);
}
// ------------------------------------------------------------------------------------------------------------
/*!
    @brief CLI_Time
*/
// ------------------------------------------------------------------------------------------------------------
int CLI_Gpio (int argc, char **argv)
{
	ConsolePrintf("\r\n");
	ConsoleInsertPrintf("Port 0 : 0x%08x\r\nPort 1 : 0x%08x", LPC_GPIO->PIN[0], LPC_GPIO->PIN[1] );

	return(1);
}

// ------------------------------------------------------------------------------------------------------------
/*!
    @brief CLI_9Dof
*/
// ------------------------------------------------------------------------------------------------------------
int CLI_9Dof (int argc, char **argv)
{
uint8_t result = 1;
uint32_t i = 0;

	ConsolePrintf("\r\n");
	LSM330_ReadNow();
	MAG3110_Task();

	for (i=0; i<200; i++)
	{
		__WFI();
	}
	return(result);
}

#define DIPCORTEX_PIN_CNT	40-5

typedef struct DIPCORTEX_PINS_T
{
	uint8_t 	port;
	uint8_t		pinNo;
} DIPCORTEX_PINS_T;

const DIPCORTEX_PINS_T DIP_CORTEX_PINS[DIPCORTEX_PIN_CNT] =
{
	{ 0, 0  },		// P1  P0_0
	{ 0, 11 },		// P2  P0_11
	{ 0, 12 },		// P3  P0_12
	{ 0, 13 },		// P4  P0_13
	{ 0, 14 },		// P5  P0_14
	{ 1, 31 },		// P6  P1_31

	{ 0, 16 },		// P8  P0_16
	{ 0, 22 },		// P9  P0_22
	{ 0, 23 },		// P10 P0_23

	{ 1, 29 },		// P13 P1_29
	{ 1, 21 },		// P14 P1_21
	{ 0, 8  },		// P15 P0_8
	{ 0, 9  },		// P16 P0_9
	{ 1, 24 },		// P17 P1_24
	{ 0, 4  },		// P18 P0_4
	{ 1, 13 },		// P19 P1_13
	{ 1, 14 },		// P20 P1_14

	{ 1, 22 },		// P21 P1_22
	{ 0, 17 },		// P22 P0_17
	{ 0, 5  },		// P23 P0_5
	{ 0, 21 },		// P24 P0_21
	{ 0, 19 },		// P25 P0_19
	{ 0, 18 },		// P26 P0_18

	{ 1, 15 },		// P27 P1_15
	{ 1, 16 },		// P28 P1_16
	{ 1, 25 },		// P29 P1_25
	{ 1, 19 },		// P30 P1_19

	{ 0, 20 },		// P33 P0_20
	{ 0, 2  },		// P34 P0_2
	{ 1, 26 },		// P35 P1_26
	{ 1, 27 },		// P36 P1_27
	{ 1, 20 },		// P37 P1_20
	{ 1, 23 },		// P38 P1_23
	{ 0, 7  },		// P39 P0_7
	{ 1, 28 }		// P40 P1_28
};

void Test ( void )
{
uint32_t i = 0;

	LPC_GPIO->DIR[ 0 ] = 2;
	LPC_GPIO->SET[ 0 ] = 2;

	DelayUs(50000);

	LPC_GPIO->CLR[ 0 ] = 2;
	for ( i = 0; i< DIPCORTEX_PIN_CNT; i++ )
	{
		LPC_GPIO->DIR[ DIP_CORTEX_PINS[i].port ] |= 1<<DIP_CORTEX_PINS[i].pinNo;
		LPC_GPIO->SET[ DIP_CORTEX_PINS[i].port ] = 1<<DIP_CORTEX_PINS[i].pinNo;
		DelayUs(10000);
		LPC_GPIO->CLR[ DIP_CORTEX_PINS[i].port ] = 1<<DIP_CORTEX_PINS[i].pinNo;
		//LPC_GPIO->DIR[ DIP_CORTEX_PINS[i].port ] &= ~(1<<DIP_CORTEX_PINS[i].pinNo);
	}

}

int CLI_Test (int argc, char **argv)
{
	Test();
	Buttons_Init();
}

int CLI_Soft (int argc, char **argv)
{
	UsbCdcDisconnect();
}
