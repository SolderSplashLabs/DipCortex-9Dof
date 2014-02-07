
#include "SolderSplashLpc.h"
#include "i2c.h"

#include "MAG3110.h"


// ------------------------------------------------------------------------------------------------------------
//
// ConfigureMAG3110: Setup MAG3110 device
//
// ------------------------------------------------------------------------------------------------------------
uint8_t ConfigureMAG3110( void )
{
	uint32_t i;
	uint8_t flag = FALSE;

	/* example of MAG3110 use */

	/* the sequence to get the reading is:
	set CTRL2 to 0x80
	set CTRL1 to 0x01

	can now wait for INT to go high, or poll STATUS
	*/

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
	I2CMasterBuffer[0] = MAG3110_ADDR;
	I2CMasterBuffer[1] = MAG3110_WHO_AM_I;
	I2CMasterBuffer[2] = MAG3110_ADDR | RD_BIT;
	I2CEngine( );

	if(I2CSlaveBuffer[0] == MAG3110_ID)
	{

		/* Configure CTRL2 register  */
		for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */                                                                                                                                                                                                                                                                                                                                                                                                          for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */
		{
			I2CMasterBuffer[i] = 0;
		}

		I2CWriteLength = 3;
		I2CReadLength = 0;
		I2CMasterBuffer[0] = MAG3110_ADDR;
		I2CMasterBuffer[1] = MAG3110_CTRL_REG2;
		I2CMasterBuffer[2] = 0x80;		/* AUTO_RST */
		I2CEngine( );


		/* Configure CTRL1 register  */
		for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */                                                                                                                                                                                                                                                                                                                                                                                                          for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */
		{
			I2CMasterBuffer[i] = 0;
		}

		I2CWriteLength = 3;
		I2CReadLength = 0;
		I2CMasterBuffer[0] = MAG3110_ADDR;
		I2CMasterBuffer[1] = MAG3110_CTRL_REG1;
		I2CMasterBuffer[2] = MAG3110_ACTIVE;		/* set AC, active mode */
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
// ReadMAG3110: Read MAG3110 data
//
// ------------------------------------------------------------------------------------------------------------
uint8_t ReadMAG3110( void )
{
	uint32_t i;
	uint8_t flag = FALSE;

	/* example of MAG3110 use */

	/* the sequence to get the reading is:
	read DR_STATUS to see if ZYXDR bit is set
	read

	can now wait for INT to go high, or poll STATUS
	*/


	for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */
	{
		I2CMasterBuffer[i] = 0;
		I2CSlaveBuffer[i] = 0;
	}

	I2CWriteLength = 2;
	I2CReadLength = 1;
	I2CMasterBuffer[0] = MAG3110_ADDR;
	I2CMasterBuffer[1] = MAG3110_DR_STATUS;
	I2CMasterBuffer[2] = MAG3110_ADDR | RD_BIT;
	I2CEngine( );

	if(I2CSlaveBuffer[0] & MAG3110_ZYXDR)
	{

		/* Configure CTRL2 register  */
		for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */                                                                                                                                                                                                                                                                                                                                                                                                          for ( i = 0; i < BUFSIZE; i++ )	/* clear buffer */
		{
			I2CMasterBuffer[i] = 0;
		}

		I2CWriteLength = 2;
		I2CReadLength = 6;
		I2CMasterBuffer[0] = MAG3110_ADDR;
		I2CMasterBuffer[1] = MAG3110_OUT_X_MSB;
		I2CMasterBuffer[2] = MAG3110_ADDR | RD_BIT;
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
// MAG3110_Task: read data
//
// ------------------------------------------------------------------------------------------------------------
uint8_t MAG3110_Task(void)
{
	
	uint8_t flag = FALSE;

	if(ReadMAG3110())
	{
		MAG3110_Buffer[0] = (I2CSlaveBuffer[0] << 8) + I2CSlaveBuffer[1];
		MAG3110_Buffer[1] = (I2CSlaveBuffer[2] << 8) + I2CSlaveBuffer[3];
		MAG3110_Buffer[2] = (I2CSlaveBuffer[4] << 8) + I2CSlaveBuffer[5];

		ConsolePrintf("Mag3110 : %d, %d, %d \r\n", MAG3110_Buffer[0], MAG3110_Buffer[1], MAG3110_Buffer[2]);
		flag = TRUE;
	}


	return flag ;
}
