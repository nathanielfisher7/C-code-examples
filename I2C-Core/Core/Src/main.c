////////////////////////////////////
// LAB 8 - I2C Program to talk to MPU-9250.
// Editor: Nathaniel Fisher
// MPU-9250 is a System in Package (SiP)= MPU-6500 (which contains a 3-axis gyroscope, a 3-axis accelerometer) +  AK8963 (a 3-axis magnetometer).
// MPU-6500  is an I2C device with an address of 0x68 and AK8963 is another device with an address 0f 0x0C.
//
// The connections
//    I2C1_SCL - PB08
//    I2C1_SDA - PB09
//
// Assignment:
//    1) Implement the I2C1 initialization function  (I2C1_init)
//    2) Implement the I2C1 write function (I2C1_byteWrite)
//    3) Implement the I2C1 read function I2C1_byteRead)
//    Optional:
//    4) Use 4 LEDs and place them on 4 different directions (N, S, E, and W) on the breadboard
//    5) Using the accelerometer X and Y values blink the leds to reflect the tilting status.
//       This is left for you to decide how do implement. One way is to adjust the LED freq by
//       frequency divider function so that when the sensor is set on flat surface the LEDs are
//       blinking very fast when you think they are solid ON. when the sensor is tilted in the X
//       then X LED would slow the blinking, the slower the higher the tilt is.
//
////////////////////////////
#include "stm32f411xe.h"
#include "stdio.h"  // needed for the printing

#define _BV( idx ) ( 1<<(idx) )  // A nice macro for setting up the bit number <idx>


// All these defines are extracted from the device datasheet. Usually we put them in a separate .h file and we jut include it here.
#define MPU9250_ID    (0x68<<1) //MPU-6500 7-bit I2C address = 0x68  (or 0x69 with some circuit change)
#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B  // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75  // Should return 0x71  -- this can be used for testing that you can communicate with the device
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

// Those defines are for the other sensor
#define AK8963_ID        (0x0C<<1) // //AK8963   7-bit I2C address = 0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48 -- this can be used for testing that you can communicate with the device
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04  // data
#define AK8963_YOUT_L    0x05  // data
#define AK8963_YOUT_H    0x06  // data
#define AK8963_ZOUT_L    0x07  // data
#define AK8963_ZOUT_H    0x08  // data
#define AK8963_ST2       0x09  // status2
#define AK8963_CNTL      0x0A
#define AK8963_CNTL2     0x0B
#define AK8963_ASAX      0x10
#define AK8963_ASAY      0x11
#define AK8963_ASAZ      0x12
// All done with the datasheet registers


// Along with the 7-bit I2C device address, you need to send Read or Write bit as defined here
#define I2C_WRITE 0
#define I2C_READ  1

//***************************//
// Function Declaration
void I2C1_init(void);
void TIM2_init(void);
void GPIO_init(void);
void TIM3_init(void);


// Device drivers
void  AK8963init();
void  MPU9250init();
void    I2C1_byteWrite(uint8_t, uint8_t, uint8_t) ;
uint8_t I2C1_byteRead(uint8_t, uint8_t);
int16_t readTemp();
int16_t readGyroX();
int16_t readGyroY();
int16_t readGyroZ();
int16_t readAccelX();
int16_t readAccelY();
int16_t readAccelZ();
int16_t readMagX();
int16_t readMagY();
int16_t readMagZ();

// General utility functions
void delayMs(int n);
void USART2_init(void);
void USART2_write(int c);
int  USART2_read(void);
void myprint(char msg[]);


/* Global Variable */
char txt[256];
int PWM_Detail = 2000; /* Value range to be used in calculating*/
int main(void)
{
    USART2_init();

    GPIO_init();
    TIM3_init();


    I2C1_init();
    MPU9250init();

    // I2C MPU9250 TEST
    uint8_t test_value= I2C1_byteRead(MPU9250_ID, WHO_AM_I_MPU9250);
    // test_value must be 0x71 if you can talk to the MPU-6500 device
    // Try to run in debug and check its value

    // I2C AK8963 TEST
    AK8963init();
    test_value= I2C1_byteRead(AK8963_ID, WHO_AM_I_AK8963);
    // test_value must be 0x48 if you can talk to the AK8963 device
    // Try to run in debug and check its value


//        Re-strobe the mag data
    I2C1_byteWrite(AK8963_ID, AK8963_CNTL, 0x11); // Continuous mode, 16 bit output
    delayMs(100);

    __disable_irq(); /* Disable global IRQs during initialization 		*/

    TIM2_init();

    __enable_irq(); 	/* Enable global IRQs after initialization	 	*/

    // Now the device is configured
    while(1)
    {
    	/* Troubleshoot/development test code */
//       int16_t T=readTemp();    // Read the sensor temperature
//       int16_t Gx=readGyroX();  // Read the sensor Gyrosope for X axis
//       int16_t Gy=readGyroY();  // Read the sensor Gyrosope for Y axis
//       int16_t Gz=readGyroZ();  // Read the sensor Gyrosope for Z axis
//       int16_t Ax=readAccelX(); // Read the sensor Accelerometer for X axis -- This reg will need to control LED X_left, X_right
//       int16_t Ay=readAccelY(); // Read the sensor Accelerometer for Y axis -- This reg will need to control LED Y_up, y_down
//       int16_t Az=readAccelZ(); // Read the sensor Accelerometer for Z axis
//       int16_t Mx=readMagX();   // Read the sensor Magnetometer for X axis
//       int16_t My=readMagY();   // Read the sensor Magnetometer for Y axis
//       int16_t Mz=readMagZ();   // Read the sensor Magnetometer for Z axis
//
////       sprintf(txt, "$%d %d %d %d %d %d %d %d %d %d;", T, Gx, Gy, Gz, Ax, Ay, Az, Mx, My, Mz); // printing all sensors in one shot!
////       sprintf(txt, "$%d %d %d;", Mx, My, Mz); // printing Magnetometer values
////       sprintf(txt, "$%d %d %d;", Ax, Ay, Az); // printing all Accelerometer values
////       sprintf(txt, "$%d %d %d;", Gx, Gy, Gz); // printing Gyro values
////       sprintf(txt, "$%d;", T); // printing temp values
    }
}


// I2C1 Initialization function. This function will
// 1- Enable clocks (I2C and GPIO)
// 2- Enable GPIO in AF and Open-drain. Note that is no need for
//    pull up from the MCU since MPU-9250 has a built in pull-ups
// 3- Reset I2C1 module
// 4- Program its clock (two fields in two different regs, checkout the book for example).
//    The target speed is 100KHz.
// 4- Enable peripheral
void I2C1_init(void) {

	RCC->AHB1ENR 	|= 1<<1;			/* Enable clock for GPIOB */
	RCC->APB1ENR 	|= 1<<21;			/* Enable clock I2C1 */

	GPIOB->MODER 	&= ~(0xF<<16);		/* Clear PB8 and PB9 pins mode bits */
	GPIOB->MODER 	|= 0xA<<16;			/* Set PB6 and PB7 to alternate function mode */
	GPIOB->AFR[0] 	&= ~(0xFF);		/* Clear PB6 and PB7's Alternate function bits */
	GPIOB->AFR[1] 	|= 0x44;		/* Set PB6 and PB7 to AF4 for SCL and SDA respectively */
	GPIOB->OTYPER 	|= 3<<8;			/* Set  PB6 and PB7 to open drain */


	I2C1->CR1 		|= 1<<15;			/* Reset I2C mode */
	I2C1->CR1 		&= ~(1<<15);		/* Disable Reset I2C */
	I2C1->CR2 		&= ~(0x3F);			/* Clear FREQ bits */
	I2C1->CR2 		|= 0x10;			/* Set to 16 MHz = 1/16MHz = 62.5 ns */
	I2C1->CCR 		&= 0;				/* Clear CCR */
	I2C1->CCR 		|= 0x50;			/* Set to 80 to set to standard mode 100 KHz*/
	I2C1->TRISE 	|= 17;				/* Set maximum rise time */
	I2C1->FLTR		|= 0x1;				/* Enable digital filtering */
	I2C1->CR1 		|= 0x0001;			/* Set PE bit to enable I2C1 module */
}



// I2C Write Operation
// Input parameters:
//   slave_addr -> slave device id (address is already shifted one place)
//   reg_addr   -> register address inside the slave device that needs to be written
//   reg_data   -> data to be written to the register
//
// This function will
//  1- make sure that the peripheral is not busy
//  2- issue the START condition
//  3- check that you see start flag (meaning that the master can control the SDA)
//  4- Send the slave ID (7bits) along with the intended operation (1bit for read or write)
//  5- check that you see address flag (meaning there is a device with a matching address)
//  6- clear SR1 and SR2  (by reading them)
//  7- check that the data reg is empty
//  8- send reg_addr
//  9- check that the data reg is empty
//  10- send reg_data
//  11- issue the STOP condition
void I2C1_byteWrite(uint8_t slave_addr, uint8_t reg_addr, uint8_t reg_data) {
	volatile int tmp;

	while (I2C1->SR2 & 2);		/* Wait till not busy */

	I2C1->CR1 |= 0x1<<8;		/* Generate start */
	while (!(I2C1->SR1 & 1));	/* Wait till start bit flag is set */

	I2C1->DR = slave_addr;		/* Write address to data register */
	while (!(I2C1->SR1 & 2));	/* Wait until address end of transmission bit is set */
	tmp = I2C1->SR1;				/* Read SR1 to clear flags */
	tmp = I2C1->SR2;				/* Read SR2 to clear flags */

	while (!(I2C1->SR1 & (0x1<<7)));	/* Wait for TxE data register empty bit. Shows the data was transmitted */
	I2C1->DR = reg_addr;			/* Send data address */

	while (!(I2C1->SR1 & (0x1<<7)));	/* Wait for TxE data register empty bit */
	I2C1->DR = reg_data;			/* Send data */

	while (!(I2C1->SR1 & 0x4));		/* Wait for byte transfer finished bit to see when transmission completes */
	I2C1->CR1 |= 0x200;				/* Send stop bit */

}

// I2C Read Operation
// Input parameters:
//   slave_addr -> slave device id (address is already shifted one place)
//   reg_addr   -> register address inside the slave device that needs to be read
//  Returned
//   reg_data   -> data from register
//
// This function will
//  1- make sure that the peripheral is not busy
//  2- issue the START condition
//  3- check that you see start flag (meaning that the master can control the SDA)
//  4- Send the slave ID (7bits) along with the intended operation (1bit for read or write)
//  5- check that you see address flag (meaning there is a device with a matching address)
//  6- clear SR1 and SR2  (by reading them)
//  7- check that the data reg is empty
//  8- send reg_addr
//  9- check that the data reg is empty
//  10- issue the RESTART condition
//  11- check that you see start flag (meaning that the master can control the SDA)
//  12- Send the slave ID (7bits) along with the intended operation (1bit for read or write)
//  13- check that you see address flag (meaning there is a device with a matching address)
//  14- Disable Acknowledge
//  15- clear SR1 and SR2  (by reading them)
//  16- check that the receive data reg is not empty
//  17- read the received data and store it into reg_data
//  18- issue the STOP condition
//  19- return reg_data
uint8_t I2C1_byteRead(uint8_t slave_addr, uint8_t reg_addr) {
    uint8_t reg_data;
    volatile int tmp;

    while (I2C1->SR2 & 2);		/* Wait until not busy */

	I2C1->CR1 |= 	0x1<<8;		/* Generate start */
	while (!(I2C1->SR1 & 1));	/* Wait till start bit flag is set */

	I2C1->DR = 		slave_addr;	/* Write slave address to data register with write bit set */
	while (!(I2C1->SR1 & 2));	/* Wait until address end of transmission bit is set */
	tmp = 			I2C1->SR2;	/* Read SR2 to clear flags */

	while (!(I2C1->SR1 & (0x1<<7)));	/* Wait for TxE data register empty bit. Shows the data was transmitted */
	I2C1->DR = 		reg_addr;	/* Send data address */

	while (!(I2C1->SR1 & (0x1<<7)));	/* Wait for TxE data register empty bit. Shows the data was transmitted */

	I2C1->CR1 |= 	0x1<<8;		/* Generate start */
	while (!(I2C1->SR1 & 1));	/* Wait till start bit flag is set */
	I2C1->DR = slave_addr | 1;	/* Send slave address with read bit */

	while (!(I2C1->SR1 & 2));	/* Wait until address end of transmission bit is set */
	I2C1->CR1 &= 	~(1<<10);	/* Disable the acknowledge bit to do a NACK */
	tmp = 			I2C1->SR1;	/* Read SR1 to clear flags */
	tmp = 			I2C1->SR2;	/* Read SR2 to clear flags */

	I2C1->CR1 |= 	1<<9;		/* Set up Stop condition */

	while (!(I2C1->SR1 & (0x1<<6)));	/* Wait until RxNE bit to tell when to read data from data register */
	reg_data = 		I2C1->DR;	/* Read data from I2C device */

    return reg_data;
}



void MPU9250init(){
   #define H_RESET 7
   #define CLKSEL  1
   // MPU-9250 Chip

   // Register[PWR_MGMT_1] : H_RESET   SLEEP   CYCLE   GYRO_STANDBY   PD_PTAT   CLKSEL[2:0]
   //                           1        0       0         0             0          1
   I2C1_byteWrite(MPU9250_ID, PWR_MGMT_1, _BV(H_RESET)|CLKSEL); // Clear SLEEP mode bit, enable all sensors

   delayMs(10);                             // Sometime to allow reseting the chip

   I2C1_byteWrite(MPU9250_ID, PWR_MGMT_1, CLKSEL);             //  Auto select clock or use internal
   delayMs(100);                           // Sometime to allow stabilizing the chip

   // Register[CONFIG] : FIFO_MODE   EXT_SYNC_SET[2:0]   DLPF_CFG[2:0]
   //                        0              0                3
   I2C1_byteWrite(MPU9250_ID, CONFIG,     0x03); // Low-pass filter enable

   // Register[SMPLRT_DIV] : SMPLRT_DIV[7:0]
   //                             4
   // SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) = 1K/(1+4)= 200Hz
   I2C1_byteWrite(MPU9250_ID, SMPLRT_DIV, 0x04); // Use a 200 Hz rate; a rate consistent with the filter update rate


   // Register[GYRO_CONFIG]: XGYRO_Cten   YGYRO_Cten    ZGYRO_Cten   GYRO_FS_SEL[1:0]         -     FCHOICE_B[1:0]
   //                           0             0             0          00  (+250dps)         N/A          00
   //                                                                  01  (+500dps)
   //                                                                  10  (+1000dps)
   //                                                                  11  (+2000dps)
   //
   // I2C1_byteWrite(MPU9250_ID, GYRO_CONFIG, (3<<3) );   // Set full scale range for the gyro +2000dps
   I2C1_byteWrite(MPU9250_ID, GYRO_CONFIG, 0 );   // Set full scale range for the gyro +250dps




   // Register[ACCEL_CONFIG]: ax_st_en   ay_st_en   az_st_en   ACCEL_FS_SEL[1:0]   -[2:0]
   //                             0          0         0             00  (+-2g)
   //                                                                01  (+-4g)
   //                                                                10  (+-8g)
   //                                                                11  (+-16g)
   I2C1_byteWrite(MPU9250_ID, ACCEL_CONFIG, (3<<3));  // Set full scale range for the accelerometer +-16g


   // Register[ACCEL_CONFIG2]: -     ACCEL_FCHOICE_B[3:2]       A_DLPF_CFG[1:0]
   //                                           0                       3
   I2C1_byteWrite(MPU9250_ID, ACCEL_CONFIG2, (3<<3));  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz


   // Register[INT_PIN_CFG]: ACTL      OPEN      LATCH_INT_EN      INT_ANYRD_2CLEAR    ACTL_FSYNC    FSYNC_INT_MODE_EN     BYPASS_EN   -
   //                         0         0             0                  1                 0                0                1        N/A
   I2C1_byteWrite(MPU9250_ID, INT_PIN_CFG, 0x12);             // Allows the MCU to access the magnetometer on the I2C b us
}

void AK8963init(){
I2C1_byteWrite(AK8963_ID, AK8963_CNTL, 0x00); // Power down magnetometer
I2C1_byteWrite(AK8963_ID, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
I2C1_byteWrite(AK8963_ID, AK8963_CNTL, 0x12); // Continuous mode, 16 bit output
}

int16_t readTemp(){
int16_t tempInt;
tempInt =I2C1_byteRead(MPU9250_ID, TEMP_OUT_H)<<8;
tempInt|=I2C1_byteRead(MPU9250_ID, TEMP_OUT_L);
float  tempFloat=((float) tempInt) / 333.87 + 21.0;
return (int16_t) tempFloat;
}

int16_t readGyroX(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, GYRO_XOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, GYRO_XOUT_L);
return res;
}

int16_t readGyroY(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, GYRO_YOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, GYRO_YOUT_L);
return res;
}

int16_t readGyroZ(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, GYRO_ZOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, GYRO_ZOUT_L);
return res;
}

int16_t readAccelX(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, ACCEL_XOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, ACCEL_XOUT_L);
return res;
}

int16_t readAccelY(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, ACCEL_YOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, ACCEL_YOUT_L);
return res;
}

int16_t readAccelZ(){
int16_t res;
res  = I2C1_byteRead(MPU9250_ID, ACCEL_ZOUT_H)<<8;
res |= I2C1_byteRead(MPU9250_ID, ACCEL_ZOUT_L);
return res;
}
int16_t readMagX(){
int16_t res;
res  = I2C1_byteRead(AK8963_ID, AK8963_XOUT_H)<<8;
res |= I2C1_byteRead(AK8963_ID, AK8963_XOUT_L);
return res;
}
int16_t readMagY(){
int16_t res;
res  = I2C1_byteRead(AK8963_ID, AK8963_YOUT_H)<<8;
res |= I2C1_byteRead(AK8963_ID, AK8963_YOUT_L);
return res;
}
int16_t readMagZ(){
int16_t res;
res  = I2C1_byteRead(AK8963_ID, AK8963_ZOUT_H)<<8;
res |= I2C1_byteRead(AK8963_ID, AK8963_ZOUT_L);
return (res);
}


void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0' ){ USART2_write(msg[idx++]);}
}

/* initialize USART2 to transmit at 9600 Baud */
void USART2_init (void) {
    RCC->AHB1ENR |= 1;          /* Enable GPIOA clock */
    RCC->APB1ENR |= 0x20000;    /* Enable USART2 clock */

    /* Configure PA2, PA3 for USART2 TX, RX */
    GPIOA->AFR[0] &= ~0xFF00;
    GPIOA->AFR[0] |=  0x7700;   /* alt7 for USART2 */
    GPIOA->MODER  &= ~0x00F0;
    GPIOA->MODER  |=  0x00A0;   /* enable alt. function for PA2, PA3 */

    USART2->BRR = 0x008B;       /* 115200 baud @ 16 MHz */
    USART2->CR1 = 0x000C;       /* enable Tx, Rx, 8-bit data */
    USART2->CR2 = 0x0000;       /* 1 stop bit */
    USART2->CR3 = 0x0000;       /* no flow control */
    USART2->CR1 |= 0x2000;      /* enable USART2 */
}

/* Write a character to USART2 */
void USART2_write (int ch) {
    while (!(USART2->SR & 0x0080)) {}   // wait until Tx buffer empty
    USART2->DR = (ch & 0xFF);
}

/* Read a character from USART2 */
int USART2_read(void) {
    while (!(USART2->SR & 0x0020)) {}   // wait until char arrives
    return USART2->DR;
}


void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        TIM2->CNT = 0;                  /* clear timer counter */
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}
//////////////////////////////////////////////////////////////////////////////////////////
/* Student Added Extra Code */
//////////////////////////////////////////////////////////////////////////////////////////



void TIM2_init(void) {

    RCC->APB1ENR |= 1;              /* enable TIM2 clock 				*/
    TIM2->PSC = 1600000-1;          /* divided by 100ms				*/
    TIM2->ARR = 100-1;        		/* divided by val 					*/
    TIM2->CNT = 0;                  /* clear timer counter 				*/
    TIM2->CR1 |= 1;                 /* enable TIM2 						*/
    TIM2->DIER |= 1;				/* Enable UIE 						*/

    NVIC_EnableIRQ(TIM2_IRQn);		/* Enable the Timer interrupt. 		*/
    NVIC_SetPriority(TIM2_IRQn, 3);	/* Set priority of TIM2 interrupt 	*/
}


/* Set up for four channel PWM */
void TIM3_init(void) {


    /* setup TIM1 */
    RCC->APB1ENR |= 2;              /* enable TIM1 clock */
    TIM3->PSC = 0x16-1;             /* divided by 0x16 -- 1 us */
    TIM3->ARR = PWM_Detail-1;               /* divided by 1000 -- so there are 1000 PWM steps */
    TIM3->CNT = 0;					/* Initialize timer counter at 0 */
    TIM3->CCMR1 = 0x6060;           /* PWM mode ch1 and 2 */
    TIM3->CCMR2 = 0x6060;			/* PWM mode ch3 and 4 */

    TIM3->CCER = 0x1111;			/* Enable each channel */
    TIM3->CCR1 = 100;				/* Initiate channel 1 PWM */
    TIM3->CCR2 = 100;				/* Initialize channel 2 PWM */
    TIM3->CCR3 = 100;				/* Initialize channel 3 PWM */
    TIM3->CCR4 = 100;				/* Initialize channel 4 PWM */
//    TIM3->BDTR |=0x8000;            /* 	Enable timer */
    TIM3->CR1 = 1;					/* Enable Timer 1 */
}

/* Setup PA8 and PA4 for TIM1 usage */
void GPIO_init(void) {
	RCC->AHB1ENR |= 4; 				/* Enable GPIOA clock 			*/
	GPIOC->AFR[0] |= 0x22000000;
	GPIOC->AFR[1] |= 0x00000022;		/* Set PC6-9 to ch1-4 of TIM3 	*/
	GPIOC->MODER &= ~0x000FF000; 	/* Clear GPIOA PA5 Mode Bits 	*/
	GPIOC->MODER |=  0x000AA000; 	/* Set PA5 as an output pin, Set
	 	 	 	 	 	 	 	 	 * PC6-9 as alternate function
	 	 	 	 	 	 	 	 	 * to connect them to timer 1
	 	 	 	 	 	 	 	 	 */
}

//////////////////////////////////////////////////////////////////////////////////////////////////
/* Student Defined Interrupt Handling */
//////////////////////////////////////////////////////////////////////////////////////////////////

/* Updates each channel of Timer 1 at a timed interval of 100ms. */

void TIM2_IRQHandler(void) {
	if (!(I2C1->SR2 & 2)) {
		 int16_t Ax=readAccelX(); // Read the sensor Accelerometer for X axis -- This reg will need to control LED X_left, X_right
		 int16_t Ay=readAccelY(); // Read the sensor Accelerometer for Y axis -- This reg will need to control LED Y_up, y_down
		float FAx = (float) Ax-110;
		float FAy = (float) Ay-90;

    	/* Check X leveling to control the desired PWM of the appropriate LED's.
    	 * Causes the lower LED to get brighter.
    	 * Range -40 to 40 out of 2000 to allow stability when level.
    	 */
    	if(FAx > 40) {
    		TIM3->CCR2 = (int16_t)((FAx/2000)*(PWM_Detail-2))+1; /* Left LED PWM increases in value */
    		TIM3->CCR1 = 1;
    	} else if (FAx < -40){
    		FAx = (-1*FAx); /* Invert x Value when originally negative */
    		TIM3->CCR2 = 1;
    		TIM3->CCR1 = (int16_t)((FAx/2000)*(PWM_Detail-2))+1;	/* Right LED PWM Increases in value */
    	} else {
    		TIM3->CCR2 = 0;
    		TIM3->CCR1 = 0;
    	}
    	/* Check Y leveling to control the desired PWM of the appropriate LED's.
    	 * Causes the lower LED to get brighter.
    	 * Range -40 to 40 out of 2000 to allow stability when level.
    	 */
		if (FAy > 40) {
    		TIM3->CCR3 = (int16_t)((FAy/2000)*(PWM_Detail-2))+1; /* Top LED PWM increases in value */
    		TIM3->CCR4 = 1;
		} else if(FAy < -40) {
			FAy = (-1*FAy); /* Invert y Value when originally negative */
    		TIM3->CCR3 = 1;
    		TIM3->CCR4 = (int16_t)((FAy/2000)*(PWM_Detail-2))+1;	/* Bottom LED PWM increases in value */
		} else {
    		TIM3->CCR3 = 0;
    		TIM3->CCR4 = 0;
		}
		sprintf(txt, "$%d %d;", (int)FAx, (int)FAy); // printing all Accelerometer values
    	myprint(txt);


    	TIM2->CNT = 0;                  /* Restart the timer			*/

    	//Re-strobe the mag data
    	I2C1_byteWrite(AK8963_ID, AK8963_CNTL, 0x11); // Continuous mode, 16 bit output
    	delayMs(100);
	}
}

