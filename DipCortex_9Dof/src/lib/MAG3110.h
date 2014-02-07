
extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;


uint16_t MAG3110_Buffer[3];


/* For more info, read MAG3110 datasheet */
#define MAG3110_ADDR			0x1C
#define MAG3110_ID				0xC4

#define MAG3110_DR_STATUS		0x00
#define MAG3110_OUT_X_MSB		0x01
#define MAG3110_OUT_X_LSB		0x02
#define MAG3110_OUT_Y_MSB		0x03
#define MAG3110_OUT_Y_LSB		0x04
#define MAG3110_OUT_Z_MSB		0x05
#define MAG3110_OUT_Z_LSB		0x06
#define MAG3110_WHO_AM_I		0x07
#define MAG3110_SYSMOD			0x08
#define MAG3110_OFF_X_MSB 		0x09
#define MAG3110_OFF_X_LSB 		0x0A
#define MAG3110_OFF_Y_MSB 		0x0B
#define MAG3110_OFF_Y_LSB 		0x0C
#define MAG3110_OFF_Z_MSB 		0x0D
#define MAG3110_OFF_Z_LSB 		0x0E
#define MAG3110_DIE_TEMP		0x0F
#define MAG3110_CTRL_REG1		0x10
#define MAG3110_CTRL_REG2		0x11

#define MAG3110_ZYXDR			BIT3
#define MAG3110_ACTIVE			BIT0


#define MAG3110		0



uint8_t ConfigureMAG3110( void );
uint8_t ReadMAG3110( void );
uint8_t MAG3110_Task(void);


