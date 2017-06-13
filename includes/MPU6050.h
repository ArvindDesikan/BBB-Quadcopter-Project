/*
 * MPU6050.h
 *
 *  Created on: Mar 25, 2017
 *      Author: arvinddesikan
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>
#include "i2cdev.h"

// ACCELEROMETER-GYRO Self Test Data
#define SELF_TEST_X				0x0D
#define SELF_TEST_XA_TEST		5
#define SELF_TEST_XA_TEST_LEN	3
#define SELF_TEST_XG_TEST		0
#define SELF_TEST_XG_TEST_LEN	5

#define SELF_TEST_Y				0x0E
#define SELF_TEST_YA_TEST		5
#define SELF_TEST_YA_TEST_LEN	3
#define SELF_TEST_YG_TEST		0
#define SELF_TEST_YG_TEST_LEN	5

#define SELF_TEST_Z				0x0F
#define SELF_TEST_ZA_TEST		5
#define SELF_TEST_ZA_TEST_LEN	3
#define SELF_TEST_ZG_TEST		0
#define SELF_TEST_ZG_TEST_LEN	5

// ACCELEROMETER Self Test Data LOWER BITS
#define SELF_TEST_A				0x10
#define SELF_TEST_XA_TEST_L		4
#define SELF_TEST_YA_TEST_L		2
#define SELF_TEST_ZA_TEST_L		0
#define SELF_TEST_A_LEN			2

// SAMPLE RATE DIVISION
#define SMPLRT_DIV				0x19

// DLPF CONFIG
#define CONFIG					0x1A
#define EXT_SYNC_SET			3
#define EXT_SYNC_SET_LEN		3
#define DLPF_CFG				0
#define DLPF_CFG_LEN			3

// GYROSCOPE Configuration
#define GYRO_CONFIG				0x1B
#define FS_SEL					3
#define FS_SEL_LEN				2
#define FS_SEL_0_SENS			131.0
#define FS_SEL_1_SENS			65.5
#define FS_SEL_2_SENS			32.8
#define FS_SEL_3_SENS			16.4
#define ZG_ST					5
#define YG_ST					6
#define XG_ST					7

// ACCELEROMETER Configuration
#define ACCEL_CONFIG			0x1C
#define AFS_SEL					3
#define AFS_SEL_LEN				2
#define AFS_SEL_0_SENS			16384
#define AFS_SEL_1_SENS			8192
#define AFS_SEL_2_SENS			4096
#define AFS_SEL_3_SENS			2048
#define ZA_ST					5
#define YA_ST					6
#define XA_ST					7

// INTERRUPT Configuration
#define INT_PIN_CFG				0x37
#define INT_LEVEL				7
#define INT_OPEN				6
#define LATCH_INT_EN			5
#define INT_RD_CLEAR			4
#define FSYNC_INT_LEVEL			3
#define FSYNC_INT_EN			2
#define I2C_BYPASS_EN			1
#define INT_PIN_CFG_LEN			1

// INTERRUPT ENABLE
#define INT_ENABLE				0x38
#define FIFO_OFLOW_EN			4
#define I2C_MST_INT_EN			3
#define DATA_RDY_EN				0
#define INT_ENABLE_LEN			1

// INTERRUPT Status
#define INT_STATUS				0x3A
#define FIFO_OFLOW_INT			4
#define I2C_MST_INT				3
#define DATA_RDY_INT			0
#define INT_STATUS_LEN			1

// ACCELEROMETER Data Register
#define ACCEL_XOUT_H			0x3B
#define ACCEL_XOUT_L			0x3C
#define ACCEL_YOUT_H			0x3D
#define ACCEL_YOUT_L			0x3E
#define ACCEL_ZOUT_H			0x3F
#define ACCEL_ZOUT_L			0x40

// TEMPERATURE Data Register
#define TEMP_OUT_H				0x41
#define TEMP_OUT_L				0x42

// GYROSCOPE Data Register
#define GYRO_XOUT_H				0x43
#define GYRO_XOUT_L				0x44
#define GYRO_YOUT_H				0x45
#define GYRO_YOUT_L				0x46
#define GYRO_ZOUT_H				0x47
#define GYRO_ZOUT_L				0x48

// SENSOR MEASUREMENT RESET
#define SIGNAL_PATH_RESET		0x68
#define TEMP_RESET				0
#define ACCEL_RESET				1
#define GYRO_RESET				2

// USER CONTROL - CONTROL RESET
#define USER_CTRL				0x6A
#define SIG_COND_RESET			0
#define I2C_MST_RESET			1
#define FIFO_RESET				2
#define I2C_IF_DIS				4
#define I2C_MST_EN				5
#define FIFO_MST_EN				6

// POWER MANAGEMENT 1
#define PWR_MGMT_1				0x6B
#define CLKSEL					0
#define CLKSEL_LEN				3
#define TEMP_DIS				3
#define CYCLE					5
#define SLEEP					6
#define DEVICE_RESET			7

// POWER MANAGEMENT 2
#define PWR_MGMT_2 				0x6C
#define LP_WAKE_CTRL			6
#define LP_WAKE_CTRL_LEN		2
#define STBY_XA					5
#define STBY_YA					4
#define STBY_ZA					3
#define STBY_XG					2
#define STBY_YG					1
#define STBY_ZG					0

// FIFO READ/WRITE
#define FIFO_R_W				0x74

// WHO AM I
#define WHO_AM_I				0x75

#define PI						3.14
#define TS						0.0046
#define G						-9.8

class MPU6050:protected i2cdevice{
public://private:
	unsigned char A_TEST[3];
	unsigned char G_TEST[3];
	int freqAccel;
	int freqGyro;
	unsigned int update;
	unsigned int accelsens;
	float gyrosens;
	float Racc;
	float accelRest;
	float accelXn;
	float accelYn;
	float accelZn;
	float gyroXn;
	float gyroYn;
	float gyroZn;
	float velXinit;
	float velYinit;
	float velZinit;
	float Ax;
	float Ay;
	float Az;
	float accelXB;
	float accelYB;
	float accelZB;
	float accelXest;
	float accelYest;
	float accelZest;
	float trustX;
	float trustY;
	float trustZ;
/*	short accelX;
	short accelY;
	short accelZ;*/
	float gyroX;
	float gyroY;
	float gyroZ;
	float zeroaX;
	float zeroaY;
	float zeroaZ;
	float zerogX;
	float zerogY;
	float zerogZ;
	float sens[6];
	float sensor[6];
	float R[3][3];
	float RT[3][3];
	float PaX[2][2];
	float PaY[2][2];
	float PaZ[2][2];
	float PgX[2][2];
	float PgY[2][2];
	float PgZ[2][2];
	float biasgX;
	float biasgY;
	float biasgZ;
	float QgX_angle;
	float QgX_bias;
	float QgY_angle;
	float QgY_bias;
	float QgZ_angle;
	float QgZ_bias;
	float RgX;
	float RgY;
	float RgZ;
	float KgX[2];
	float KgY[2];
	float KgZ[2];
	float SgX;
	float SgY;
	float SgZ;
	float errorgX;
	float errorgY;
	float errorgZ;
	float ax;
	float ay;
	float az;
	float rot[3];
	float tempG[3];
	float aXn;
	float aYn;
	float aZn;
	float gXn;
	float gYn;
	float gZn;
	float phi;
	float theta;
	float psi;
	float angXaccel;
	float angYaccel;
	float angZaccel;
	float fa;
	float fg;
	float accelXI;
	float accelYI;
	float accelZI;
	float angXI;
	float angYI;
	float angZI;
	float angX;
	float angY;
	float angZ;
	float velphi;
	float veltheta;
	float velpsi;

//private:
/*	short accelX;
	short accelY;
	short accelZ;
	short gyroX;
	short gyroY;
	short gyroZ;
	unsigned int accelsens;
	float gyrosens;
	float trustX;
	float trustY;
	float trustZ;*/
//public:
	float state[15]; // TRANS POS, TRANS VEL, TRANS ACC, ROT POS, ROT VEL
	MPU6050(unsigned int device=0x68);

	// Self Test
	 void selftestDIS();
	 void selftestEN();

	// SMPLRT_DIV Reg
	 uint8_t getSample_Rate();
	 bool setSample_Rate(uint8_t rate);

	// CONFIG Reg
	 uint8_t getDLPFMode();
	 bool setDLPFMode(uint8_t mode);

	// GYRO_CONFIG Reg
	 uint8_t getFullScaleGyroRange();
	 bool setFullScaleGyroRange(uint8_t range);

	// ACCEL_CONFIG Reg
	 uint8_t getFullScaleAccelRange();
	 bool setFullScaleAccelRange(uint8_t range);

	// INTERRUPT Settings
	 bool setINTConfigPin(bool level, bool pinmode, bool latch, bool intrd);
	 bool setINTMode(int mode);		 // DATA_RDY will be the chosen interrupt
	 bool getINTStatus(int mode);	 // DATA_RDY_INT would be read

	// Accelerometer Data
	 uint16_t getAccelX();
	 uint16_t getAccelY();
	 uint16_t getAccelZ();

	// Gyroscope Data
	 uint16_t getGyroX();
	 uint16_t getGyroY();
	 uint16_t getGyroZ();

	// Wake Up device
	 bool wakeup();

	// Signal Reset
	 bool signal_path_reset();
	 bool signal_cond_reset();

	// Clock Mode Selection
	 bool setCLKmode(uint8_t mode);

	// Reset Device
	 bool deviceReset();

	// WHO AM I
	 uint8_t deviceID();

	 uint16_t wordreadtest();

	 uint8_t bytereadtest();

	// PROCESS DATA I
	 void zeroError();

	 void get6DOF();
	 void kalmanfilter();
	 void getIMUStates();
	// CLOSE HANDLE
	 ~MPU6050();

};

#endif /* MPU6050_H_ */
