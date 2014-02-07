/*
  ____        _     _           ____        _           _		 _          _
 / ___|  ___ | | __| | ___ _ __/ ___| _ __ | | __ _ ___| |__	| |    __ _| |__  ___
 \___ \ / _ \| |/ _` |/ _ \ '__\___ \| '_ \| |/ _` / __| '_ \	| |   / _` | '_ \/ __|
  ___) | (_) | | (_| |  __/ |   ___) | |_) | | (_| \__ \ | | |	| |__| (_| | |_) \__ \
 |____/ \___/|_|\__,_|\___|_|  |____/| .__/|_|\__,_|___/_| |_|	|_____\__,_|_.__/|___/
                                     |_|
 (C)SolderSplash Labs 2013 - www.soldersplash.co.uk - R. Steel - C. Matthews


	@file     main.c
	@author   Rob Steel (soldersplash.co.uk)
	@date     01 June 2013

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

*/


#include "SolderSplashLpc.h"
#include "i2c.h"
#include "gpio.h"

#include "LSM330.h"

// ------------------------------------------------------------------------------------------------------------
//
// LSM330_Init
//
// ------------------------------------------------------------------------------------------------------------
void LSM330_Init ( void )
{
	// Configure the IOCON Registers
	LSM_INT1A_INIT;
	LSM_INT2A_INIT;

	// Set interrupt pins as inputs
	LPC_GPIO->DIR[LSM_INT1A_PORT] &= ~LSM_INT1A_MASK;
	LPC_GPIO->DIR[LSM_INT2A_PORT] &= ~LSM_INT2A_MASK;

	/*
	GPIOSetPinInterrupt(CHANNEL6, LSM_INT1A_PORT, LSM_INT1A_PIN, 0, 1);
	GPIOSetPinInterrupt(CHANNEL7, LSM_INT2A_PORT, LSM_INT2A_PIN, 0, 1);
	GPIOPinIntEnable(CHANNEL6, 1);
	GPIOPinIntEnable(CHANNEL7, 1);
	*/

	I2CInit(I2CSLAVE);
	ConfigureLSM330();
	LSM330_ConfInt();


}

// ------------------------------------------------------------------------------------------------------------
/*!
    @brief LSM330_ConfInt
*/
// ------------------------------------------------------------------------------------------------------------
void LSM330_ConfInt ( void )
{
uint32_t i;

	for ( i = 0; i < BUFSIZE; i++ )	 //clear buffer                                                                                                                                                                                                                                                                                                                                                                                                           for ( i = 0; i < BUFSIZE; i++ )	 clear buffer
	{
		I2CMasterBuffer[i] = 0;
	}

	I2CWriteLength = 4;
	I2CReadLength = 0;
	I2CMasterBuffer[0] = LSM330_ACC_ADDR;
	I2CMasterBuffer[1] = INT1_THS_A | AUTO_INC;
	I2CMasterBuffer[2] = 0x20;
	I2CMasterBuffer[3] = 0x0A;

	if ( I2CEngine( ) ) 
	{

		for ( i = 0; i < BUFSIZE; i++ )	 //clear buffer                                                                                                                                                                                                                                                                                                                                                                                                           for ( i = 0; i < BUFSIZE; i++ )	 clear buffer
		{
			I2CMasterBuffer[i] = 0;
		}
	
		I2CWriteLength = 3;
		I2CReadLength = 0;
		I2CMasterBuffer[0] = LSM330_ACC_ADDR;
		I2CMasterBuffer[1] = INT1_CFG_A;
		I2CMasterBuffer[2] = 0x4F;
	
		I2CEngine( );
		
		//return ( true );
	}
	else
	{
		//return ( false );
	}
}

// ------------------------------------------------------------------------------------------------------------
/*!
    @brief PIN_INT6_IRQHandler - INT1A on GPIO Interrupt 6
*/
// ------------------------------------------------------------------------------------------------------------
void PIN_INT6_IRQHandler(void)
{
uint32_t intMask = 0x1<<6;

	if ( LPC_GPIO_PIN_INT->IST & intMask )
	{
		if ( LPC_GPIO_PIN_INT->ISEL & intMask )
		{
			// Level
		}
		else
		{
			if ( ( LPC_GPIO_PIN_INT->RISE & intMask ) && ( LPC_GPIO_PIN_INT->IENR & intMask ) )
			{
				// Rising
				LPC_GPIO_PIN_INT->RISE = intMask;
			}
			if ( ( LPC_GPIO_PIN_INT->FALL & intMask ) && ( LPC_GPIO_PIN_INT->IENF & intMask ) )
			{
				// Falling
				LPC_GPIO_PIN_INT->FALL = intMask;
			}

			LPC_GPIO_PIN_INT->IST = intMask;
		}
	}

	ConsoleInsertPrintf("GPIO INT6");
}

// ------------------------------------------------------------------------------------------------------------
/*!
	@brief PIN_INT7_IRQHandler - INT1A on GPIO Interrupt 6
*/
// ------------------------------------------------------------------------------------------------------------
void PIN_INT7_IRQHandler(void)
{
uint32_t intMask = 0x1<<7;

	if ( LPC_GPIO_PIN_INT->IST & intMask )
	{
		if ( LPC_GPIO_PIN_INT->ISEL & intMask )
		{
			// Level
		}
		else
		{
			if ( ( LPC_GPIO_PIN_INT->RISE & intMask ) && ( LPC_GPIO_PIN_INT->IENR & intMask ) )
			{
				// Rising
				LPC_GPIO_PIN_INT->RISE = intMask;
			}
			if ( ( LPC_GPIO_PIN_INT->FALL & intMask ) && ( LPC_GPIO_PIN_INT->IENF & intMask ) )
			{
				// Falling
				LPC_GPIO_PIN_INT->FALL = intMask;
			}

			LPC_GPIO_PIN_INT->IST = intMask;
		}
	}

	ConsoleInsertPrintf("GPIO INT7");
}

// ------------------------------------------------------------------------------------------------------------
//
// ConfigureLSM330: Setup LSM330 device
//
// ------------------------------------------------------------------------------------------------------------
uint8_t ConfigureLSM330( void )
{
uint32_t i;
bool flag = FALSE;

	/* In order to start the I2CEngine, all the parameters must be
	set in advance, including I2CWriteLength, I2CReadLength, and
	the I2cMasterBuffer which contains the stream command/data to
	the I2c slave device. */

	//prove we can get correct device ID first
	/* Get device ID register */
	for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */
	{
		I2CMasterBuffer[i] = 0;
		I2CSlaveBuffer[i] = 0;
	}

	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = LSM330_GYR_ADDR;
	I2CMasterBuffer[1] = WHO_AM_I_G;
	I2CMasterBuffer[2] = LSM330_GYR_ADDR | RD_BIT;
	I2CEngine( );

	if(I2CSlaveBuffer[0] == LSM330_ID)
	{

		//Configure Accelerometer registers
		for ( i = 0; i < BUFSIZE; i++ )	 //clear buffer                                                                                                                                                                                                                                                                                                                                                                                                           for ( i = 0; i < BUFSIZE; i++ )	 clear buffer
		{
			I2CMasterBuffer[i] = 0;
		}

		I2CWriteLength = 8;
		I2CReadLength = 0;
		I2CMasterBuffer[0] = LSM330_ACC_ADDR;
		I2CMasterBuffer[1] = CTRL_REG1_A | AUTO_INC;
		I2CMasterBuffer[2] = A_ODR_25HZ | LP_EN | Z_EN | X_EN | Y_EN;	//CTRL_REG1_A
		I2CMasterBuffer[3] = HP_NORMAL;							//CTRL_REG2_A
		I2CMasterBuffer[4] = I1_AOI1;							//CTRL_REG3_A
		I2CMasterBuffer[5] = FS_2G;								//CTRL_REG4_A
		I2CMasterBuffer[6] = LIR_INT1 | D4D_INT1;				//CTRL_REG5_A
		I2CMasterBuffer[7] = 0;									//CTRL_REG6_A
		I2CEngine( );


		//Configure Gyro registers
		for ( i = 0; i < BUFSIZE; i++ )	 //clear buffer                                                                                                                                                                                                                                                                                                                                                                                                           for ( i = 0; i < BUFSIZE; i++ )	 clear buffer
		{
			I2CMasterBuffer[i] = 0;
		}

		I2CWriteLength = 7;
		I2CReadLength = 0;
		I2CMasterBuffer[0] = LSM330_GYR_ADDR;
		I2CMasterBuffer[1] = CTRL_REG1_G | AUTO_INC;
		I2CMasterBuffer[2] = G_ODR_95HZ_BW_12_5HZ | G_PD | G_Z_EN | G_Y_EN | G_X_EN;		//CTRL_REG1_G
		I2CMasterBuffer[3] = 0;																//CTRL_REG2_G
		I2CMasterBuffer[4] = 0;																//CTRL_REG3_G
		I2CMasterBuffer[5] = DATA_BE | FS_250DPS;											//CTRL_REG4_G
		I2CMasterBuffer[6] = 0;																//CTRL_REG5_G
		I2CEngine( );


		flag = TRUE;
	}

	else
	{
		flag = FALSE;
	}

	return flag;
}

// ------------------------------------------------------------------------------------------------------------
//
// ReadAccLSM330: Read Accelerometer data from LSM330 data
//
// ------------------------------------------------------------------------------------------------------------
uint8_t ReadAccLSM330( void )
{
	uint32_t i;
	uint8_t flag = FALSE;


	for ( i = 0; i < BUFSIZE; i++ )
	{
		I2CMasterBuffer[i] = 0;
		I2CSlaveBuffer[i] = 0;
	}

	//query status reg
	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = LSM330_ACC_ADDR;
	I2CMasterBuffer[1] = STATUS_REG_A;
	I2CMasterBuffer[2] = LSM330_ACC_ADDR | RD_BIT;
	I2CEngine( );

	//check to see if data is available
	if(I2CSlaveBuffer[0] & ZYXDA)
	{

		// read out XYZ data
		for ( i = 0; i < BUFSIZE; i++ )                                                                                                                                                                                                                                                                                                                                                                                                        for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */
		{
			I2CMasterBuffer[i] = 0;
		}

		I2CWriteLength = 2;
		I2CReadLength = 6;
		I2CMasterBuffer[0] = LSM330_ACC_ADDR;
		I2CMasterBuffer[1] = OUT_X_L_A | AUTO_INC;
		I2CMasterBuffer[2] = LSM330_ACC_ADDR | RD_BIT;
		I2CEngine( );


		flag = TRUE;
	}

	else
	{
		flag = FALSE;
	}

	return flag;
}


// ------------------------------------------------------------------------------------------------------------
//
// ReadLSM330Temperature: Read Temperature
//
// ------------------------------------------------------------------------------------------------------------
int8_t ReadLSM330Temperature( void )
{
uint32_t i;
uint8_t flag = FALSE;

	for ( i = 0; i < BUFSIZE; i++ )
	{
		I2CMasterBuffer[i] = 0;
		I2CSlaveBuffer[i] = 0;
	}

	// Query temperature reg
	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = LSM330_GYR_ADDR;
	I2CMasterBuffer[1] = OUT_TEMP_G;
	I2CMasterBuffer[2] = LSM330_GYR_ADDR | RD_BIT;
	I2CEngine();

	return ( (int8_t)I2CSlaveBuffer[0] );
}

// ------------------------------------------------------------------------------------------------------------
//
// ReadIntSrc: Read Intterupt source
//
// ------------------------------------------------------------------------------------------------------------
uint8_t ReadIntSrc( void )
{
uint32_t i;
uint8_t flag = FALSE;

	for ( i = 0; i < BUFSIZE; i++ )
	{
		I2CMasterBuffer[i] = 0;
		I2CSlaveBuffer[i] = 0;
	}

	// Query temperature reg
	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = LSM330_ACC_ADDR;
	I2CMasterBuffer[1] = INT1_SOURCE_A;
	I2CMasterBuffer[2] = LSM330_ACC_ADDR | RD_BIT;
	I2CEngine();

	return ( I2CSlaveBuffer[0] );
}

// ------------------------------------------------------------------------------------------------------------
//
// ReadGyroLSM330: Read Gyro data from LSM330 data
//
// ------------------------------------------------------------------------------------------------------------
uint8_t ReadGyroLSM330( void )
{
	uint32_t i;
	uint8_t flag = FALSE;


	for ( i = 0; i < BUFSIZE; i++ )
	{
		I2CMasterBuffer[i] = 0;
		I2CSlaveBuffer[i] = 0;
	}

	//query status reg
	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = LSM330_GYR_ADDR;
	I2CMasterBuffer[1] = STATUS_REG_G;
	I2CMasterBuffer[2] = LSM330_GYR_ADDR | RD_BIT;
	I2CEngine( );

	//check to see if data is available
	if(I2CSlaveBuffer[0] & ZYXDA)
	{

		// read out XYZ data
		for ( i = 0; i < BUFSIZE; i++ )                                                                                                                                                                                                                                                                                                                                                                                                        for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */
		{
			I2CMasterBuffer[i] = 0;
		}

		I2CWriteLength = 2;
		I2CReadLength = 6;
		I2CMasterBuffer[0] = LSM330_GYR_ADDR;
		I2CMasterBuffer[1] = OUT_X_L_G | AUTO_INC;
		I2CMasterBuffer[2] = LSM330_GYR_ADDR | RD_BIT;
		I2CEngine( );


		flag = TRUE;
	}

	else
	{
		flag = FALSE;
	}

	return flag;
}

// ------------------------------------------------------------------------------------------------------------
//
// LSM330_Task: read data
//
// ------------------------------------------------------------------------------------------------------------
uint8_t LSM330_Task( bool actionInt )
{
	uint8_t flag = FALSE;
	uint8_t temperature = 0;

	if ( LPC_GPIO->PIN[LSM_INT1A_PORT] & LSM_INT1A_MASK )
	{
		if ( actionInt )
		{
			// Movement detected
		}

		if(ReadAccLSM330())
		{
			LSM330_A_Buffer[0] = (I2CSlaveBuffer[0] << 8) + I2CSlaveBuffer[1];
			LSM330_A_Buffer[1] = (I2CSlaveBuffer[2] << 8) + I2CSlaveBuffer[3];
			LSM330_A_Buffer[2] = (I2CSlaveBuffer[4] << 8) + I2CSlaveBuffer[5];

			ConsoleInsertPrintf("Accelerometer : %d, %d, %d ", LSM330_A_Buffer[0], LSM330_A_Buffer[1], LSM330_A_Buffer[2]);
			ConsoleInsertPrintf("Accelerometer INT Val : %d ", ReadIntSrc());

			flag = TRUE;
		}

		if(ReadGyroLSM330())
		{
			LSM330_G_Buffer[0] = (I2CSlaveBuffer[0] << 8) + I2CSlaveBuffer[1];
			LSM330_G_Buffer[1] = (I2CSlaveBuffer[2] << 8) + I2CSlaveBuffer[3];
			LSM330_G_Buffer[2] = (I2CSlaveBuffer[4] << 8) + I2CSlaveBuffer[5];
			ConsoleInsertPrintf("Gyro : %d, %d, %d ", LSM330_G_Buffer[0], LSM330_G_Buffer[1], LSM330_G_Buffer[2]);

			temperature = ReadLSM330Temperature();
			ConsoleInsertPrintf("Gyro Temp : %d", temperature);
			flag = TRUE;
		}
	}

	return flag ;
}

// ------------------------------------------------------------------------------------------------------------
//
// LSM330_ReadNow
//
// ------------------------------------------------------------------------------------------------------------
uint8_t LSM330_ReadNow(void)
{

	uint8_t flag = FALSE;

	if(ReadAccLSM330())
	{
		LSM330_A_Buffer[0] = (I2CSlaveBuffer[0] << 8) + I2CSlaveBuffer[1];
		LSM330_A_Buffer[1] = (I2CSlaveBuffer[2] << 8) + I2CSlaveBuffer[3];
		LSM330_A_Buffer[2] = (I2CSlaveBuffer[4] << 8) + I2CSlaveBuffer[5];
		ConsolePrintf("Accelerometer : %d, %d, %d \r\n", LSM330_A_Buffer[0], LSM330_A_Buffer[1], LSM330_A_Buffer[2]);
		flag = TRUE;
	}

	if(ReadGyroLSM330())
	{
		LSM330_G_Buffer[0] = (I2CSlaveBuffer[0] << 8) + I2CSlaveBuffer[1];
		LSM330_G_Buffer[1] = (I2CSlaveBuffer[2] << 8) + I2CSlaveBuffer[3];
		LSM330_G_Buffer[2] = (I2CSlaveBuffer[4] << 8) + I2CSlaveBuffer[5];
		ConsolePrintf("Gyro : %d, %d, %d \r\n", LSM330_G_Buffer[0], LSM330_G_Buffer[1], LSM330_G_Buffer[2]);
		flag = TRUE;
	}



	return flag ;
}


