
#define LSM_DEN_PORT		1
#define LSM_DEN_PIN			26
#define LSM_DEN_MASK		1<<LSM_DEN_PIN
#define LSM_DEN_INIT		{LPC_IOCON->PIO1_26 = 0;}

#define LSM_INT1A_PORT		0
#define LSM_INT1A_PIN		7
#define LSM_INT1A_MASK		1<<LSM_INT1A_PIN
#define LSM_INT1A_INIT		{LPC_IOCON->PIO0_7 = 0;}

#define LSM_INT2A_PORT		1
#define LSM_INT2A_PIN		23
#define LSM_INT2A_MASK		1<<LSM_INT2A_PIN
#define LSM_INT2A_INIT		{LPC_IOCON->PIO1_23 = 0;}


extern volatile uint32_t I2CCount;
extern volatile uint8_t I2CMasterBuffer[BUFSIZE];
extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern volatile uint32_t I2CMasterState;
extern volatile uint32_t I2CReadLength, I2CWriteLength;

int16_t LSM330_A_Buffer[3];
int16_t LSM330_G_Buffer[3];


/* For more info, read LSM330 datasheet */
#define LSM330_ACC_ADDR		0x32
#define LSM330_GYR_ADDR		0xD6
#define LSM330_ID			0xD4



#define CTRL_REG1_A 	0x20
#define CTRL_REG2_A 	0x21
#define CTRL_REG3_A 	0x22
#define CTRL_REG4_A 	0x23
#define CTRL_REG5_A 	0x24
#define CTRL_REG6_A 	0x25
#define REFERENCE_A 	0x26
#define STATUS_REG_A 	0x27
#define OUT_X_L_A 		0x28
#define OUT_X_H_A 		0x29
#define OUT_Y_L_A 		0x2A
#define OUT_Y_H_A 		0x2B
#define OUT_Z_L_A 		0x2C
#define OUT_Z_H_A 		0x2D
#define FIFO_CTRL_REG	0x2E
#define FIFO_SRC_REG 	0x2F
#define INT1_CFG_A 		0x30
#define INT1_SOURCE_A 	0x31
#define INT1_THS_A 		0x32
#define INT1_DURATION_A 0x33
#define INT2_CFG_A 		0x34
#define INT2_SOURCE_A 	0x35
#define INT2_THS_A 		0x36
#define INT2_DURATION_A 0x37
#define CLICK_CFG_A 	0x38
#define CLICK_SRC_A 	0x39
#define CLICK_THS_A 	0x3A
#define TIME_LIMIT_A 	0x3B
#define TIME_LATENCY_A 	0x3C
#define TIME_WINDOW_A 	0x3D
#define Act_THS			0x3E
#define Act_DUR 		0x3F
#define WHO_AM_I_G 		0x0F
#define CTRL_REG1_G 	0x20
#define CTRL_REG2_G 	0x21
#define CTRL_REG3_G 	0x22
#define CTRL_REG4_G 	0x23
#define CTRL_REG5_G 	0x24
#define REFERENCE_G 	0x25
#define OUT_TEMP_G 		0x26
#define STATUS_REG_G 	0x27
#define OUT_X_L_G 		0x28
#define OUT_X_H_G		0x29
#define OUT_Y_L_G 		0x2A
#define OUT_Y_H_G 		0x2B
#define OUT_Z_L_G 		0x2C
#define OUT_Z_H_G 		0x2D
#define FIFO_CTRL_REG_G 0x2E
#define FIFO_SRC_REG_G 	0x2F
#define INT1_CFG_G 		0x30
#define INT1_SRC_G 		0x31
#define INT1_TSH_XH_G 	0x32
#define INT1_TSH_XL_G 	0x33
#define INT1_TSH_YH_G 	0x34
#define INT1_TSH_YL_G 	0x35
#define INT1_TSH_ZH_G 	0x36
#define INT1_TSH_ZL_G 	0x37
#define INT1_DURATION_G 0x38


#define AUTO_INC BIT7


//CTRL_REG1_A bits
#define ODR3	BIT7
#define	ODR2	BIT6
#define	ODR1	BIT5
#define	ODR0	BIT4
#define	LP_EN	BIT3
#define	Z_EN	BIT2
#define	Y_EN	BIT1
#define	X_EN	BIT0

#define A_P_DOWN		0
#define A_ODR_1HZ		ODR0
#define A_ODR_10HZ		ODR1
#define A_ODR_25HZ		ODR1+ODR0
#define A_ODR_50HZ		ODR2
#define A_ODR_100HZ		ODR2+ODR0
#define A_ODR_200HZ		ODR2+ODR1
#define A_ODR_400HZ		ODR2+ODR1+ODR0
#define A_ODR_LPM_0		ODR3
#define A_ODR_LPM_1		ODR3+ODR0

//CTRL_REG2_A bits
#define HPM1	BIT7
#define HPM0	BIT6
#define HPCF2	BIT5
#define HPCF1	BIT4
#define FDS		BIT3
#define HPCLICK	BIT2
#define HPIS2	BIT1
#define HPIS1	BIT0

#define HP_RST_NORMAL	0
#define HP_REF			HPM0
#define HP_NORMAL		HPM1
#define HP_AUTO_RST		HPM1+HPM0

//CTRL_REG3_A bits
#define I1_CLICK 	BIT7
#define I1_AOI1		BIT6
#define I1_DRDY1 	BIT4
#define I1_DRDY2 	BIT3
#define I1_WTM 		BIT2
#define I1_OVERRUN	BIT1


//CTRL_REG4_A bits
#define BLE		BIT6
#define FS1		BIT5
#define FS0		BIT4
#define HR 		BIT3
#define SIM		BIT0

#define DATA_LE	0
#define DATA_BE	BLE

#define FS_2G	0
#define FS_4G	FS0
#define FS_8G	FS1
#define FS_16G	FS1+FS0


//CTRL_REG5_A bits
#define BOOT 		BIT7
#define FIFO_EN		BIT6
#define LIR_INT1 	BIT3
#define D4D_INT1	BIT2


//CTRL_REG6_A bits
#define I2_CLICK 	BIT7
#define I2_INT1 	BIT6
#define BOOT_I2		BIT4
#define H_LACTIVE	BIT1


//STATUS_REG_A bits
#define ZYXOR 		BIT7
#define ZOR 		BIT6
#define YOR 		BIT5
#define XOR 		BIT4
#define ZYXDA 		BIT3
#define ZDA 		BIT2
#define YDA 		BIT1
#define XDA			BIT0

//FIFO_CTRL_REG bits
#define FM1 		BIT7
#define FM0 		BIT6
#define TR 			BIT5
#define FTH4 		BIT4
#define FTH3 		BIT3
#define FTH2 		BIT2
#define FTH1 		BIT1
#define FTH0		BIT0

#define FIFO_BYPASS		0
#define FIFO_FIFO		FM0
#define FIFO_STREAM		FM1
#define FIFO_TRIGGER	FM1+FM0


//FIFO_SRC_REG bits
#define WTM 		BIT7
#define OVRN_FIFO	BIT6
#define EMPTY 		BIT5


//CTRL_REG1_G bits
#define DR1 		BIT7
#define DR0 		BIT6
#define BW1 		BIT5
#define BW0 		BIT4
#define G_PD 		BIT3
#define G_Z_EN		BIT2
#define G_Y_EN		BIT1
#define G_X_EN		BIT0

#define G_ODR_95HZ_BW_12_5HZ	0
#define G_ODR_95HZ_BW_25HZ		BW0
#define G_ODR_190HZ_BW_12_5HZ	DR0
#define G_ODR_190HZ_BW_25HZ		BW0+DR0
#define G_ODR_190HZ_BW_50HZ		BW1+DR0
#define G_ODR_190HZ_BW_70HZ		BW1+BW0+DR0
#define G_ODR_380HZ_BW_20HZ		DR1
#define G_ODR_380HZ_BW_25HZ		BW0+DR1
#define G_ODR_380HZ_BW_50HZ		BW1+DR1
#define G_ODR_380HZ_BW_100HZ	BW1+BW0+DR1
#define G_ODR_760HZ_BW_30HZ		DR1+DR0
#define G_ODR_760HZ_BW_35HZ		BW0+DR1+DR0
#define G_ODR_760HZ_BW_50HZ		BW1+DR1+DR0
#define G_ODR_760HZ_BW_100HZ	BW1+BW0+DR1+DR0


//CTRL_REG4_G bits
#define BDU 	BIT7

#define FS_250DPS	0
#define FS_500DPS	FS0
#define FS_2000DPS	FS1

uint8_t ConfigureLSM330( void );
uint8_t ReadAccLSM330( void );
uint8_t ReadGyroLSM330( void );
//uint8_t LSM330_Task(void);
uint8_t LSM330_Task( bool actionInt );




