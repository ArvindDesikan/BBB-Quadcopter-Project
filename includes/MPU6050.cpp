/*
 * MPU6050.cpp
 *
 *  Created on: Mar 25, 2017
 *      Author: arvinddesikan
 */
#include <unistd.h>
#include "i2cdev.h"
#include "MPU6050.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

MPU6050::MPU6050(unsigned int device):i2cdevice(device){
	this->wakeup();
	this->setSample_Rate(0x00);
	this->setDLPFMode(0x01);
	this->setFullScaleAccelRange(0x00);
	this->setFullScaleGyroRange(0x00);
	this->selftestDIS();
	this->setCLKmode(0x01);
	this->setINTMode(0);
	this->freqAccel = 1000;
	this->biasgX = 0.0;
	this->biasgY = 0.0;
	this->biasgZ = 0.0;
	this->accelZest = 0.0;
	this->trustX = 0.7;
	this->trustY = 0.7;
	this->trustZ = 0.7;
	this->Ax     = 0.0;
	this->Ay     = 0.0;
	this->Az     = 0.0;
	this->fa	 = 0.01;
	this->fg	 = 0.01;
	*(this->state+0)  = 0.0;
	*(this->state+1)  = 0.0;
	*(this->state+2)  = 0.0;
	*(this->state+3)  = 0.0;
	*(this->state+4)  = 0.0;
	*(this->state+5)  = 0.0;
	*(this->state+9)  = 0.0;
	*(this->state+10) = 0.0;
	*(this->state+11) = 0.0;
	this->phi  = 0.0;
	this->theta= 0.0;
	this->psi  = 0.0;
	this->update = 1;
	QgX_angle = 0.001;
	QgX_bias  = 0.003;
	RgX 	  = 0.1;
	QgY_angle = 0.001;
	QgY_bias  = 0.003;
	RgY 	  = 0.1;
	QgZ_angle = 0.001;
	QgZ_bias  = 0.003;
	RgZ 	  = 0.1;
	PgX[0][0] = 0.0;
	PgX[0][1] = 0.0;
	PgX[1][0] = 0.0;
	PgX[1][1] = 0.0;
	PgY[0][0] = 0.0;
	PgY[0][1] = 0.0;
	PgY[1][0] = 0.0;
	PgY[1][1] = 0.0;
	PgZ[0][0] = 0.0;
	PgZ[0][1] = 0.0;
	PgZ[1][0] = 0.0;
	PgZ[1][1] = 0.0;
	this->zeroError();
}

/* SELF TEST */
void MPU6050::selftestDIS(){
	// Self Test Disable

	// GYRO_CONFIG reg Self-Test
	this->writebits(GYRO_CONFIG,0x00,3,5);  // Disable Self-Test Gyro

	// ACCEL_CONFIG reg Self-Test
	this->writebits(ACCEL_CONFIG,0x00,3,5); // Disable Self-Test Accel
}

void MPU6050::selftestEN(){
	// Self Test Enable

	// GYRO_CONFIG reg Self-Test
	this->writebits(GYRO_CONFIG,0x07,3,5);  // Enable Self-Test Gyro

	// ACCEL_CONFIG reg Self-Test
	this->writebits(ACCEL_CONFIG,0x07,5,3); // Enable Self-Test Accel

	// Self-Test Data Accelerometer
	this->A_TEST[0]=this->readbits(SELF_TEST_X,SELF_TEST_XA_TEST_LEN,SELF_TEST_XA_TEST);
	this->A_TEST[0]=(this->A_TEST[0]<<2)|(this->readbits(SELF_TEST_A,SELF_TEST_A_LEN,SELF_TEST_XA_TEST));

	this->A_TEST[1]=this->readbits(SELF_TEST_Y,SELF_TEST_YA_TEST_LEN,SELF_TEST_YA_TEST);
	this->A_TEST[1]=(this->A_TEST[1]<<2)|this->readbits(SELF_TEST_Y,SELF_TEST_YA_TEST_LEN,SELF_TEST_YA_TEST);

	this->A_TEST[2]=this->readbits(SELF_TEST_Z,SELF_TEST_ZA_TEST_LEN,SELF_TEST_ZA_TEST);
	this->A_TEST[2]=(this->A_TEST[2]<<2)|this->readbits(SELF_TEST_Z,SELF_TEST_ZA_TEST_LEN,SELF_TEST_ZA_TEST);

	// Self-Test Data Gyroscope
	this->G_TEST[0]=this->readbits(SELF_TEST_X,SELF_TEST_XG_TEST_LEN,SELF_TEST_XG_TEST);

	this->G_TEST[1]=this->readbits(SELF_TEST_Y,SELF_TEST_YG_TEST_LEN,SELF_TEST_YG_TEST);

	this->G_TEST[2]=this->readbits(SELF_TEST_Z,SELF_TEST_ZG_TEST_LEN,SELF_TEST_ZG_TEST);
}


/* SAMPLE RATE DIVIDER */
bool MPU6050::setSample_Rate(uint8_t rate){
	// Sample Rate Divider
	return this->writebyte(SMPLRT_DIV,rate);
}

uint8_t MPU6050::getSample_Rate(){
	// Sample Rate Divider
	return this->readbyte(SMPLRT_DIV);
}


/* DIGITAL LOW PASS FILTER Configuration */
uint8_t MPU6050::getDLPFMode(){
	// Digital low pass filter configure
	return this->readbits(CONFIG,DLPF_CFG_LEN,DLPF_CFG);
}

bool MPU6050::setDLPFMode(uint8_t mode){
	// Digital low pass filter configure
	switch(int(mode)){
	case 0:this->freqGyro=8000;break;
	default:this->freqGyro=1000;break;
	}
	return this->writebits(CONFIG,mode,DLPF_CFG_LEN,DLPF_CFG);
}


/* GYROSCOPE Configuration */
uint8_t MPU6050::getFullScaleGyroRange(){
	// Gyro Scale Range
	return this->readbits(GYRO_CONFIG,FS_SEL_LEN,FS_SEL);
}

bool MPU6050::setFullScaleGyroRange(uint8_t range){
	// Gyro Scale Range
	switch(int(range)){
			case 0:this->gyrosens=131;break;
			case 1:this->gyrosens=65.5;break;
			case 2:this->gyrosens=32.8;break;
			case 3:this->gyrosens=16.4;break;
			default:this->gyrosens=65.5;break;
	}
	return this->writebits(GYRO_CONFIG,range,FS_SEL_LEN,FS_SEL);

}


/* ACCELEROMETER Configuration */
uint8_t MPU6050::getFullScaleAccelRange(){
	// Accel Scale Range
	return this->readbits(ACCEL_CONFIG,AFS_SEL_LEN,AFS_SEL);
}

bool MPU6050::setFullScaleAccelRange(uint8_t range){
	// Accel Scale Range
	switch(int(range)){
			case 0:this->accelsens=16384;break;
			case 1:this->accelsens=8192;break;
			case 2:this->accelsens=4096;break;
			case 3:this->accelsens=2048;break;
			default:this->accelsens=8192;break;
	}
	return this->writebits(ACCEL_CONFIG,range,AFS_SEL_LEN,AFS_SEL);
}

/* INTERRUPT Settings */
bool MPU6050::setINTConfigPin(bool level, bool pinmode, bool latch, bool intrd){
	this->writebits(INT_PIN_CFG,level,INT_PIN_CFG_LEN,INT_LEVEL);
	this->writebits(INT_PIN_CFG,pinmode,INT_PIN_CFG_LEN,INT_OPEN);
	this->writebits(INT_PIN_CFG,latch,INT_PIN_CFG_LEN,LATCH_INT_EN);
	this->writebits(INT_PIN_CFG,intrd,INT_PIN_CFG_LEN,INT_RD_CLEAR);
	return 1;
}

bool MPU6050::setINTMode(int mode){
	switch(mode){
			case 0:return this->writebits(INT_ENABLE,1,INT_ENABLE_LEN,DATA_RDY_EN);
			case 1:return this->writebits(INT_ENABLE,1,INT_ENABLE_LEN,I2C_MST_INT_EN);
			case 2:return this->writebits(INT_ENABLE,1,INT_ENABLE_LEN,FIFO_OFLOW_EN);
			default:{
				   this->writebits(INT_ENABLE,0,INT_ENABLE_LEN,DATA_RDY_EN);
				   this->writebits(INT_ENABLE,0,INT_ENABLE_LEN,I2C_MST_INT_EN);
				   this->writebits(INT_ENABLE,0,INT_ENABLE_LEN,FIFO_OFLOW_EN);
				   return 1;
			}
	}
}

bool MPU6050::getINTStatus(int mode){
	switch(mode){
			case 0:return this->readbits(INT_ENABLE,INT_ENABLE_LEN,DATA_RDY_EN);
			case 1:return this->readbits(INT_ENABLE,INT_ENABLE_LEN,I2C_MST_INT_EN);
			case 2:return this->readbits(INT_ENABLE,INT_ENABLE_LEN,FIFO_OFLOW_EN);
			default:{
				   return 0;
			}
	}
}

/* ACCELEROMETER DATA */
uint16_t MPU6050::getAccelX(){
	// Accelerometer X-axis Data
	return this->readword(ACCEL_XOUT_H);
}

uint16_t MPU6050::getAccelY(){
	// Accelerometer Y-axis Data
	return this->readword(ACCEL_YOUT_H);
}

uint16_t MPU6050::getAccelZ(){
	// Accelerometer Z-axis Data
	return this->readword(ACCEL_ZOUT_H);
}


/* GYROSCOPE DATA */
uint16_t MPU6050::getGyroX(){
	// Gyroscope X-axis Data
	return this->readword(GYRO_XOUT_H);
}

uint16_t MPU6050::getGyroY(){
	// Gyroscope Y-axis Data
	return this->readword(GYRO_YOUT_H);
}

uint16_t MPU6050::getGyroZ(){
	// Gyroscope Z-axis Data
	return this->readword(GYRO_ZOUT_H);
}


/* SIGNAL RESET */
bool MPU6050::signal_path_reset(){
	// Signal Path Reset
	return this->writebits(SIGNAL_PATH_RESET,0x07,3,0);
}

bool MPU6050::signal_cond_reset(){
	// Signal Condition Reset
	return this->writebits(USER_CTRL,0x01,1,0);
}


/* CLOCK SETTING */
bool MPU6050::setCLKmode(uint8_t mode){
	// Clock Selection
	return this->writebits(PWR_MGMT_1,mode,CLKSEL_LEN,CLKSEL);
}

bool MPU6050::wakeup(){
	return this->writebits(PWR_MGMT_1,0x00,1,SLEEP);
}


/* DEVICE RESET */
bool MPU6050::deviceReset(){
	// Device Reset
	this->writebits(PWR_MGMT_1,0x01,1,DEVICE_RESET);
	usleep(100);
	return this->signal_path_reset();
}


/* DEVICE ID */
uint8_t MPU6050::deviceID(){
	// Device ID
	return this->readbyte(WHO_AM_I);
}

uint8_t MPU6050::bytereadtest(){
	return this->readbyte(WHO_AM_I);
}

uint16_t MPU6050::wordreadtest(){
	return this->readword(WHO_AM_I);
}


/* ZERO ERROR MEASUREMENT */
void MPU6050::zeroError(){

	// No bias for Z-axis Accelerometer

	int i=1;
	while(i!=3000){
		this->zeroaX = this->zeroaX + (float)((short)this->getAccelX())/this->accelsens*G;
		this->zeroaY = this->zeroaY + (float)((short)this->getAccelY())/this->accelsens*G;
		this->zeroaZ = this->zeroaZ + (float)((short)this->getAccelZ())/this->accelsens*G;
		this->zerogX = this->zerogX + (float)((short)this->getGyroX())/this->gyrosens*PI/180;
		this->zerogY = this->zerogY + (float)((short)this->getGyroY())/this->gyrosens*PI/180;
		this->zerogZ = this->zerogZ + (float)((short)this->getGyroZ())/this->gyrosens*PI/180;
		++i;
	}
	this->zeroaX = this->zeroaX/2999;
	this->zeroaY = this->zeroaY/2999;
	this->zeroaZ = this->zeroaZ/2999;
	this->zerogX = this->zerogX/2999;
	this->zerogY = this->zerogY/2999;
	this->zerogZ = this->zerogZ/2999;
	*(this->sensor+2)	= this->zeroaZ;
}

/* PROCESSED DATA I */
void MPU6050::get6DOF(){
	while(!this->getINTStatus(0)){
		usleep(200);
	}
		*(this->sensor+0) = *(this->sensor+0)*(1-this->fa)+this->fa*((float)((short)this->getAccelX())/this->accelsens*G-this->zeroaX); 	// Accel X
		*(this->sensor+1) = *(this->sensor+1)*(1-this->fa)+this->fa*((float)((short)this->getAccelY())/this->accelsens*G-this->zeroaY); 	// Accel Y
		*(this->sensor+2) = *(this->sensor+2)*(1-this->fa)+this->fa*((float)((short)this->getAccelZ())/this->accelsens*G);//+this->zeroaZ);		// Accel Z
		*(this->sensor+3) = *(this->sensor+3)*(1-this->fg)+this->fg*((float)((short)this->getGyroX())/this->gyrosens*PI/180-this->zerogX);	// Gyro Rate X
		*(this->sensor+4) = *(this->sensor+4)*(1-this->fg)+this->fg*((float)((short)this->getGyroY())/this->gyrosens*PI/180-this->zerogY);	// Gyro Rate Y
		*(this->sensor+5) = *(this->sensor+5)*(1-this->fg)+this->fg*((float)((short)this->getGyroZ())/this->gyrosens*PI/180-this->zerogZ);	// Gyro Rate Z
		*(this->sens+0)	  =  cos(3*PI/4)*(*(this->sensor+0))+sin(3*PI/4)*(*(this->sensor+1));
		*(this->sens+1)	  = -sin(3*PI/4)*(*(this->sensor+0))+cos(3*PI/4)*(*(this->sensor+1));
		*(this->sens+2)   = *(this->sensor+2);
		*(this->sens+3)	  =  cos(3*PI/4)*(*(this->sensor+3))+sin(3*PI/4)*(*(this->sensor+4));
		*(this->sens+4)	  = -sin(3*PI/4)*(*(this->sensor+3))+cos(3*PI/4)*(*(this->sensor+4));
		*(this->sens+5)   = *(this->sensor+5);
}

void MPU6050::kalmanfilter(){
	// Gather new data
	this->get6DOF();

	/* Accelerometer IMU */
	// Acceleration Magnitude
	this->Racc	= sqrt(pow(*(this->sens+0),2)+pow(*(this->sens+1),2)+pow(*(this->sens+2),2));

	// Acceleration normalized vector
	this->aXn	= *(this->sens+0)/this->Racc;
	this->aYn	= *(this->sens+1)/this->Racc;
	this->aZn	= *(this->sens+2)/this->Racc;

	// Accelerometer Angle
	if(fabs(*(this->sens+1))<1.0 && fabs(*(this->sens+2))<1.0)
		this->angXaccel = 0.0;
	else
		this->angXaccel	= atan2(-this->aYn,-this->aZn);

	if(fabs(*(this->sens+0))<1.0 && fabs(*(this->sens+2))<1.0)
			this->angYaccel = 0.0;
		else
			this->angYaccel	= atan2(+this->aXn,-this->aZn);

	if(fabs(*(this->sens+0))<1.0 && fabs(*(this->sens+1))<1.0)
		this->angZaccel	= 0.0;
	else
		this->angZaccel = atan2(+this->aYn,+this->aXn);

	if(this->angXaccel!=this->angXaccel){
		this->angXaccel	= 0.0;
	}
	if(this->angYaccel!=this->angYaccel){
		this->angYaccel	= 0.0;
	}
	if(this->angZaccel!=this->angZaccel){
		this->angZaccel	= 0.0;
	}

	// Handling tan(PI/2)
	if(fabs(this->angXaccel) == PI/2)
		this->gXn	= 0.0;
	else
		this->gXn	=  sin(this->angYaccel)/(1+pow(cos(this->angYaccel),2)*pow(tan(this->angXaccel),2));

	if(fabs(this->angYaccel) == PI/2)
		this->gYn	= 0.0;
	else
		this->gYn	= -sin(this->angXaccel)/(1+pow(cos(this->angXaccel),2)*pow(tan(this->angYaccel),2));

	if(this->aZn>=0.0)
		this->gZn	=  sqrt((1-pow(this->gXn,2)-pow(this->gYn,2)));
	else
		this->gZn	= -sqrt((1-pow(this->gXn,2)-pow(this->gYn,2)));

	this->accelXest	= (this->aXn+this->trustX*this->gXn)/(1+this->trustX);
	this->accelYest	= (this->aYn+this->trustY*this->gYn)/(1+this->trustY);
	this->accelZest	= (this->aZn+this->trustZ*this->gZn)/(1+this->trustZ);
	this->accelRest	= sqrt(pow(this->accelXest,2)+pow(this->accelYest,2)+pow(this->accelZest,2));

	/* Acceleration Body Frame */
	this->accelXB	= -this->Racc*this->accelXest;
	this->accelYB	= -this->Racc*this->accelYest;
	this->accelZB	= -(this->Racc*this->accelZest-this->zeroaZ);

    // Handling NaN conditions
    if(this->accelXB!=this->accelXB)
    	this->accelXB = *(this->sens+0);
    if(this->accelYB!=this->accelYB)
    	this->accelYB = *(this->sens+1);
    if(this->accelZB!=this->accelZB)
    	this->accelZB = *(this->sens+2);

/* this->angX 	= atan2(-this->accelYest,-this->accelZest);
    this->angY	= atan2(+this->accelXest,-this->accelZest);
    this->angZ	= atan2(+this->accelYest,+this->accelXest);
*/
	if(fabs(this->accelYB)<1.0 && fabs(this->accelZB)<1.0)
		this->angX = 0.0;
	else
		this->angX = atan2(-this->accelYest,-this->accelZest);

	if(fabs(this->accelXB)<1.0 && fabs(this->accelZB)<1.0)
			this->angY = 0.0;
		else
			this->angY = atan2(+this->accelXest,-this->accelZest);

	if(fabs(this->accelXB)<1.0 && fabs(this->accelYB)<1.0)
		this->angZ	= 0.0;
	else
		this->angZ  = 0.0;//atan2(+this->accelYest,+this->accelXest);

    // Gyroscope rate with bias correction
    this->velphi		= *(this->sens+3)-this->biasgX;
    this->veltheta		= *(this->sens+4)-this->biasgY;
    this->velpsi		= *(this->sens+5)-this->biasgZ;

    this->phi			= this->phi   + TS*this->velphi;
    this->theta			= this->theta + TS*this->veltheta;
    this->psi			= this->psi   + TS*this->velpsi;

	this->PgX[0][0] 	= this->PgX[0][0]+TS*(TS*this->PgX[1][1]-this->PgX[0][1]-this->PgX[1][0]+this->QgX_angle);
	this->PgX[0][1] 	= this->PgX[0][1]-TS*this->PgX[1][1];
	this->PgX[1][0] 	= this->PgX[1][0]-TS*this->PgX[1][1];
	this->PgX[1][1] 	= this->PgX[1][1]+TS*this->QgX_bias;

	this->PgY[0][0] 	= this->PgY[0][0]+TS*(TS*this->PgY[1][1]-this->PgY[0][1]-this->PgY[1][0]+this->QgY_angle);
	this->PgY[0][1] 	= this->PgY[0][1]-TS*this->PgY[1][1];
	this->PgY[1][0] 	= this->PgY[1][0]-TS*this->PgY[1][1];
	this->PgY[1][1] 	= this->PgY[1][1]+TS*this->QgY_bias;

	this->PgZ[0][0] 	= this->PgZ[0][0]+TS*(TS*this->PgZ[1][1]-this->PgZ[0][1]-this->PgZ[1][0]+this->QgZ_angle);
	this->PgZ[0][1] 	= this->PgZ[0][1]-TS*this->PgZ[1][1];
	this->PgZ[1][0] 	= this->PgZ[1][0]-TS*this->PgZ[1][1];
	this->PgZ[1][1] 	= this->PgZ[1][1]+TS*this->QgZ_bias;

	this->errorgX		= this->angX-this->phi;
	this->errorgY		= this->angY-this->theta;
	this->errorgZ		= this->angZ-this->psi;

	// Innovation Co-variance
	this->SgX       	= this->PgX[0][0]+this->RgX;
	this->SgY       	= this->PgY[0][0]+this->RgY;
	this->SgZ       	= this->PgZ[0][0]+this->RgZ;

	// Kalman Gain
	this->KgX[0]		= PgX[0][0]/this->SgX;
	this->KgX[1]		= PgX[1][0]/this->SgX;

	this->KgY[0]		= PgY[0][0]/this->SgY;
	this->KgY[1]		= PgY[1][0]/this->SgY;

	this->KgZ[0]		= PgZ[0][0]/this->SgZ;
	this->KgZ[1]		= PgZ[1][0]/this->SgZ;

	this->phi			= this->phi+this->KgX[0]*this->errorgX;
	this->biasgX		= this->biasgX+this->KgX[1]*this->errorgX;

	this->theta			= this->theta+this->KgY[0]*this->errorgY;
	this->biasgY		= this->biasgY+this->KgY[1]*errorgY;

//	this->psi			= this->psi+this->velpsi;
	this->psi			= this->psi+this->KgZ[0]*this->errorgZ;
	this->biasgZ		= this->biasgZ+this->KgZ[1]*this->errorgZ;

	// Error Co-variance
	this->PgX[0][0] 	= this->PgX[0][0]-this->KgX[0]*this->PgX[0][0];
	this->PgX[0][1] 	= this->PgX[0][1]-this->KgX[0]*this->PgX[0][1];
	this->PgX[1][0] 	= this->PgX[1][0]-this->KgX[1]*this->PgX[0][0];
	this->PgX[1][1] 	= this->PgX[1][1]-this->KgX[1]*this->PgX[0][1];

	this->PgY[0][0] 	= this->PgY[0][0]-this->KgY[0]*this->PgY[0][0];
	this->PgY[0][1] 	= this->PgY[0][1]-this->KgY[0]*this->PgY[0][1];
	this->PgY[1][0] 	= this->PgY[1][0]-this->KgY[1]*this->PgY[0][0];
	this->PgY[1][1] 	= this->PgY[1][1]-this->KgY[1]*this->PgY[0][1];

	this->PgZ[0][0] 	= this->PgZ[0][0]-this->KgZ[0]*this->PgZ[0][0];
	this->PgZ[0][1] 	= this->PgZ[0][1]-this->KgZ[0]*this->PgZ[0][1];
	this->PgZ[1][0] 	= this->PgZ[1][0]-this->KgZ[1]*this->PgZ[0][0];
	this->PgZ[1][1] 	= this->PgZ[1][1]-this->KgZ[1]*this->PgZ[0][1];

	R[0][0]  =  cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta);
	R[0][1]  = -cos(phi)*sin(psi);
	R[0][2]  =  cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi);
	R[1][0]  =  cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta);
	R[1][1]  =  cos(phi)*cos(psi);
	R[1][2]  =  sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi);
	R[2][0]  = -cos(phi)*sin(theta);
	R[2][1]  =  sin(phi);
	R[2][2]  =  cos(phi)*cos(theta);

	/* Inertial Frame Acceleration */
	*(this->state+6)	= R[0][0]*this->accelXB+R[0][1]*this->accelYB+R[0][2]*this->accelZB;
	*(this->state+7)	= R[1][0]*this->accelXB+R[1][1]*this->accelYB+R[1][2]*this->accelZB;
	*(this->state+8)	= R[2][0]*this->accelXB+R[2][1]*this->accelYB+R[2][2]*this->accelZB;

	/* Inertial Frame Velocity */
	if(fabs(*(this->state+6))<0.2)
		*(this->state+3) = *(this->state+3);
	else
		*(this->state+3) = *(this->state+3)+TS*(*(this->state+6));
	if(fabs(*(this->state+7))<0.2)
		*(this->state+4) = *(this->state+4);
	else
		*(this->state+4) = *(this->state+4)+TS*(*(this->state+7));
	if(fabs(*(this->state+8))<0.2)
		*(this->state+5) = *(this->state+5);
	else
		*(this->state+5) = *(this->state+5)+TS*(*(this->state+8));

	/* Inertial Frame Position */
	*(this->state+0)	= *(this->state+0)+TS*(*(this->state+3));
	*(this->state+1)	= *(this->state+1)+TS*(*(this->state+4));
	*(this->state+2)	= *(this->state+2)+TS*(*(this->state+5));

	/* Body Frame Angular Position */
	*(this->state+9)	= this->phi;
	*(this->state+10)	= this->theta;
	*(this->state+11)	= this->psi;

	/* Body Frame Angular Velocity */
	*(this->state+12)	= cos(theta)*this->velphi-cos(phi)*sin(theta)*this->velpsi;
	*(this->state+13)	= this->veltheta+sin(phi)*this->velpsi;
	*(this->state+14)	= sin(theta)*this->velphi+cos(phi)*cos(theta)*this->velpsi;
}

/*void MPU6050::kalmanfilter(){

	// Gather new data
	this->get6DOF();

	// Tolerance Band
	if(fabs(*(this->sens+0))<0.1)
		*(this->sens+0)=0.0;
	if(fabs(*(this->sens+1))<0.1)
		*(this->sens+1)=0.0;
	if(fabs(*(this->sens+2))<0.1)
		*(this->sens+2)=0.0;

	// Gyroscope rate with bias correction
    *(this->state+12) = *(this->sens+3)-biasgX;
    *(this->state+13) = *(this->sens+4)-biasgY;
    *(this->state+14) = *(this->sens+5)-biasgZ;

//	 Accelerometer IMU

	// Acceleration Magnitude
	this->Racc	= sqrt(pow(*(this->sens+0),2)+pow(*(this->sens+1),2)+pow(*(this->sens+2),2));

	// Acceleration normalized vector
	this->aXn	= *(this->sens+0)/this->Racc;
	this->aYn	= *(this->sens+1)/this->Racc;
	this->aZn	= *(this->sens+2)/this->Racc;

	// Accelerometer Angle
	this->angXaccel	= atan2(-this->aYn,-this->aZn);
	this->angYaccel = atan2(+this->aXn,-this->aZn);
	if(fabs(*(this->sens+0))<1.0 && fabs(*(this->sens+1))<1.0)
		this->angZaccel	= 0.0;
	else
		this->angZaccel	= atan2(this->aYn,+this->aXn);

	if(this->angXaccel!=this->angXaccel){
		this->angXaccel	= 0.0;
	}
	if(this->angYaccel!=this->angYaccel){
		this->angYaccel	= 0.0;
	}
	if(this->angZaccel!=this->angZaccel){
		this->angZaccel	= 0.0;
	}

	// Handling tan(PI/2)
	if(fabs(this->angXaccel) == PI/2)
		this->gXn	= 0.0;
	else
		this->gXn	=  sin(this->angYaccel)/(1+pow(cos(this->angYaccel),2)*pow(tan(this->angXaccel),2));

	if(fabs(this->angYaccel) == PI/2)
		this->gYn	= 0.0;
	else
		this->gYn	= -sin(this->angXaccel)/(1+pow(cos(this->angXaccel),2)*pow(tan(this->angYaccel),2));

	if(this->aZn>=0.0)
		this->gZn	=  sqrt((1-pow(this->gXn,2)-pow(this->gYn,2)));
	else
		this->gZn	= -sqrt((1-pow(this->gXn,2)-pow(this->gYn,2)));

	this->accelXest	= (this->aXn+this->trustX*this->gXn)/(1+this->trustX);
	this->accelYest	= (this->aYn+this->trustY*this->gYn)/(1+this->trustY);
	this->accelZest	= (this->aZn+this->trustZ*this->gZn)/(1+this->trustZ);
	this->accelRest	= sqrt(pow(this->accelXest,2)+pow(this->accelYest,2)+pow(this->accelZest,2));

//	 Acceleration Body Frame
	this->accelXB	= this->Racc*this->accelXest;
	this->accelYB	= this->Racc*this->accelYest;
	this->accelZB	= this->Racc*this->accelZest;

    // Handling NaN conditions
    if(this->accelXB!=this->accelXB)
    	this->accelXB = *(this->sens+0);
    if(this->accelYB!=this->accelYB)
    	this->accelYB = *(this->sens+1);
    if(this->accelZB!=this->accelZB)
    	this->accelZB = *(this->sens+2);

//	 Gyroscope IMU
	this->phi	= this->phi + TS*(*(this->state+12));
	this->theta	= this->theta + TS*(*(this->state+13));
	this->psi	= this->psi + TS*(*(this->state+14));

    // Gyroscope rate with bias correction
    this->gyroX	= *(this->sens+3)-this->biasgX;
    this->gyroY	= *(this->sens+4)-this->biasgY;
    this->gyroZ	= *(this->sens+5)-this->biasgZ;

	this->PgX[0][0] = this->PgX[0][0]+TS*(TS*this->PgX[1][1]-this->PgX[0][1]-this->PgX[1][0]+this->QgX_angle);
	this->PgX[0][1] = this->PgX[0][1]-TS*this->PgX[1][1];
	this->PgX[1][0] = this->PgX[1][0]-TS*this->PgX[1][1];
	this->PgX[1][1] = this->PgX[1][1]+TS*this->QgX_bias;

	this->PgY[0][0] = this->PgY[0][0]+TS*(TS*this->PgY[1][1]-this->PgY[0][1]-this->PgY[1][0]+this->QgY_angle);
	this->PgY[0][1] = this->PgY[0][1]-TS*this->PgY[1][1];
	this->PgY[1][0] = this->PgY[1][0]-TS*this->PgY[1][1];
	this->PgY[1][1] = this->PgY[1][1]+TS*this->QgY_bias;

	this->PgZ[0][0] = this->PgZ[0][0]+TS*(TS*this->PgZ[1][1]-this->PgZ[0][1]-this->PgZ[1][0]+this->QgZ_angle);
	this->PgZ[0][1] = this->PgZ[0][1]-TS*this->PgZ[1][1];
	this->PgZ[1][0] = this->PgZ[1][0]-TS*this->PgZ[1][1];
	this->PgZ[1][1] = this->PgZ[1][1]+TS*this->QgZ_bias;

	this->errorgX	= this->angXaccel-this->phi;
	this->errorgY	= this->angYaccel-this->theta;
	this->errorgZ	= this->angZaccel-this->psi;

	// Innovation Co-variance
	this->SgX       = this->PgX[0][0]+this->RgX;
	this->SgY       = this->PgY[0][0]+this->RgY;
	this->SgZ       = this->PgZ[0][0]+this->RgZ;

	// Kalman Gain
	this->KgX[0]	= PgX[0][0]/this->SgX;
	this->KgX[1]	= PgX[1][0]/this->SgX;

	this->KgY[0]	= PgY[0][0]/this->SgY;
	this->KgY[1]	= PgY[1][0]/this->SgY;

	this->KgZ[0]	= PgZ[0][0]/this->SgZ;
	this->KgZ[1]	= PgZ[1][0]/this->SgZ;

	*(this->state+9)= *(this->state+9)+this->KgX[0]*this->errorgX;
	this->biasgX	= this->biasgX+this->KgX[1]*this->errorgX;

	*(this->state+10)= *(this->state+10)+this->KgY[0]*this->errorgY;
	this->biasgY	= this->biasgY+this->KgY[1]*errorgY;

	*(this->state+11)= *(this->state+11)+this->KgZ[0]*this->errorgZ;
	this->biasgZ	= this->biasgZ+this->KgZ[1]*this->errorgZ;

	// Error Co-variance
	this->PgX[0][0] = this->PgX[0][0]-this->KgX[0]*this->PgX[0][0];
	this->PgX[0][1] = this->PgX[0][1]-this->KgX[0]*this->PgX[0][1];
	this->PgX[1][0] = this->PgX[1][0]-this->KgX[1]*this->PgX[0][0];
	this->PgX[1][1] = this->PgX[1][1]-this->KgX[1]*this->PgX[0][1];

	this->PgY[0][0] = this->PgY[0][0]-this->KgY[0]*this->PgY[0][0];
	this->PgY[0][1] = this->PgY[0][1]-this->KgY[0]*this->PgY[0][1];
	this->PgY[1][0] = this->PgY[1][0]-this->KgY[1]*this->PgY[0][0];
	this->PgY[1][1] = this->PgY[1][1]-this->KgY[1]*this->PgY[0][1];

	this->PgZ[0][0] = this->PgZ[0][0]-this->KgZ[0]*this->PgZ[0][0];
	this->PgZ[0][1] = this->PgZ[0][1]-this->KgZ[0]*this->PgZ[0][1];
	this->PgZ[1][0] = this->PgZ[1][0]-this->KgZ[1]*this->PgZ[0][0];
	this->PgZ[1][1] = this->PgZ[1][1]-this->KgZ[1]*this->PgZ[0][1];

    // Rotation Transformation R'
    // Z-X-Y
/*
    RT[0][0] =  cos(*(this->state+11))*cos(*(this->state+10))-sin(*(this->state+9))*sin(*(this->state+10))*sin(*(this->state+11));
    RT[0][1] =  cos(*(this->state+10))*sin(*(this->state+11))+cos(*(this->state+11))*sin(*(this->state+9))*sin(*(this->state+10));
    RT[0][2] = -cos(*(this->state+9))*sin(*(this->state+10));
    RT[1][0] = -cos(*(this->state+9))*sin(*(this->state+11));
    RT[1][1] =  cos(*(this->state+9))*cos(*(this->state+11));
    RT[1][2] =  sin(*(this->state+9));
    RT[2][0] =  cos(*(this->state+11))*sin(*(this->state+10))+cos(*(this->state+10))*sin(*(this->state+9))*sin(*(this->state+11));
    RT[2][1] =  sin(*(this->state+11))*sin(*(this->state+10))-cos(*(this->state+11))*cos(*(this->state+10))*sin(*(this->state+9));
    RT[2][2] =  cos(*(this->state+9))*cos(*(this->state+10));


	// Rotation Matrix

	[ cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)]
	[ cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)]
	[                               -cos(phi)*sin(theta),           sin(phi),                                cos(phi)*cos(theta)]


	R[0][0]  =  cos(-*(this->rot+2))*cos(-*(this->rot+1))-sin(-*(this->rot+0))*sin(-*(this->rot+1))*sin(-*(this->rot+2));
	R[0][1]  = -cos(-*(this->rot+0))*sin(-*(this->rot+2));
	R[0][2]  =  cos(-*(this->rot+2))*sin(-*(this->rot+1))+cos(-*(this->rot+1))*sin(-*(this->rot+0))*sin(-*(this->rot+2));
	R[1][0]  =  cos(-*(this->rot+1))*sin(-*(this->rot+2))+cos(-*(this->rot+2))*sin(-*(this->rot+0))*sin(-*(this->rot+1));
	R[1][1]  =  cos(-*(this->rot+0))*cos(-*(this->rot+2));
	R[1][2]  =  sin(-*(this->rot+2))*sin(-*(this->rot+1))-cos(-*(this->rot+2))*cos(-*(this->rot+1))*sin(-*(this->rot+0));
	R[2][0]  = -cos(-*(this->rot+0))*sin(-*(this->rot+1));
	R[2][1]  =  sin(-*(this->rot+0));
	R[2][2]  =  cos(-*(this->rot+0))*cos(-*(this->rot+1));

	RT[0][0] =  R[0][0];
	RT[0][1] =  R[1][0];
	RT[0][2] =  R[2][0];
	RT[1][0] =  R[0][1];
	RT[1][1] =  R[1][1];
	RT[1][2] =  R[2][1];
	RT[2][0] =  R[0][2];
	RT[2][1] =  R[1][2];
	RT[2][2] =  R[2][2];


	// Transform Acceleration
	*(this->state+6)	= this->accelXB;
	*(this->state+7)	= this->accelYB;
	*(this->state+8)	= this->accelZB;
	*(this->state+9)	= this->phi;
	*(this->state+10)	= this->theta;
	*(this->state+11)	= this->psi;
	*(this->state+12)	= this->gyroX;
	*(this->state+13)	= this->gyroY;
	*(this->state+14)	= this->gyroZ;
	*(this->state+3)	= *(this->state+3)+TS*(*(this->state+6));
	*(this->state+4)	= *(this->state+4)+TS*(*(this->state+7));
	*(this->state+5)	= *(this->state+5)+TS*(*(this->state+8));
	*(this->state+0)	= *(this->state+0)+TS*(*(this->state+3));
	*(this->state+1)	= *(this->state+1)+TS*(*(this->state+4));
	*(this->state+2)	= *(this->state+2)+TS*(*(this->state+5));


//    *(this->state+6)=RT[0][0]*this->AccelXB+RT[0][1]*this->AccelYB+RT[0][2]*this->AccelZB;
//    *(this->state+7)=RT[1][0]*this->AccelXB+RT[1][1]*this->AccelYB+RT[1][2]*this->AccelZB;
//    *(this->state+8)=RT[2][0]*this->AccelXB+RT[2][1]*this->AccelYB+RT[2][2]*this->AccelZB;

//  [cos(theta) 0 -cos(phi)*sin(theta);0 1 sin(phi);sin(theta) 0 cos(phi)*cos(theta)]*diffAngle;

/*   *(this->angVel+0)= cos(-*(this->rot+1))*(*(this->state+12))-cos(-*(this->rot+1))*sin(-*(this->rot+0))*(*(this->state+14));
    *(this->angVel+1)= *(this->state+13))+sin(-*(this->rot+0))*(*(this->state+14));
    *(this->angVel+2)= sin(-*(this->rot+1))*(this->state+12))+cos(-*(this->rot+0))*cos(-*(this->rot+1))*(*(this->state+14));*/


/*
    // Tolerance Band
	if(fabs(*(this->sens+0))<0.1)
    	*(this->sens+0)=0.0;
    if(fabs(*(this->sens+1))<0.1)
    	*(this->sens+1)=0.0;
    if(fabs(*(this->sens+2))<0.1)
    	*(this->sens+2)=0.0;

	// Acceleration Magnitude
	this->Racc    = sqrt(pow(*(this->sens+0),2)+pow(*(this->sens+1),2)+pow(*(this->sens+2),2));

	// Normalized Acceleration Vector
	this->accelXn = *(this->sens+0)/Racc;
	this->accelYn = *(this->sens+1)/Racc;
	this->accelZn = *(this->sens+2)/Racc;

	// Gyroscope rate with bias correction
    *(this->state+12) = *(this->sens+3)-biasgX;
    *(this->state+13) = *(this->sens+4)-biasgY;
    *(this->state+14) = *(this->sens+5)-biasgZ;

    // Acceleration vector arguments [-PI,PI]
    this->Ax     = atan2(-this->accelYn,-this->accelZn);
    this->Ay     = atan2(+this->accelXn,-this->accelZn);
    if(fabs(*(this->sens+0))<1 && fabs(*(this->sens+1))<1)
    	this->Az = 0.0;
    else
    	this->Az = atan2(-this->accelYn,+this->accelXn);



 	// Handling NaN conditions
 	if(this->Ax!=this->Ax)
  		this->Ax = 0.0;
  	if(this->Ay!=this->Ay)
  		this->Ay = 0.0;
  	if(this->Az!=this->Az)
  		this->Az =0.0;

 	// Handling tan(PI/2)
     if(fabs(this->Ax)==PI/2)
     	this->gyroXn  = 0.0;
     else
     	this->gyroXn  = sin(this->Ay)/(1+pow(cos(this->Ay),2)*pow(tan(this->Ax),2));
     if(fabs(this->Ay)==PI/2)
     	this->gyroYn  = 0.0;
     else
     	this->gyroYn  = -sin(this->Ax)/(1+pow(cos(this->Ax),2)*pow(tan(this->Ay),2));

     if(this->accelZest>=0){
         this->gyroZn  = sqrt((1-pow(this->gyroXn,2)-pow(this->gyroYn,2)));
     }
     else {
     	this->gyroZn  = -sqrt((1-pow(this->gyroXn,2)-pow(this->gyroYn,2)));
     }

    // Acceleration vector arguments [-PI,PI]
	this->Ax     = atan2(-this->accelYn,+this->accelZn)+TS*(*(this->state+12));
	this->Ay     = atan2(+this->accelXn,+this->accelZn)+TS*(*(this->state+13));
	this->Az     = atan2(-this->accelXn,+this->accelYn)+TS*(*(this->state+14));

	// Handling NaN conditions
    if(this->Ax!=this->Ax)
    	this->Ax = 0.0;
    if(this->Ay!=this->Ay)
    	this->Ay = 0.0;
    if(this->Az!=this->Az)
    	this->Az =0.0;

	// Handling tan(PI/2)
    if(fabs(this->Ax)==PI/2)
    	this->gyroXn  = 0.0;
    else
    	this->gyroXn  = sin(this->Ay)/(1+pow(cos(this->Ay)*tan(this->Ax),2));

    if(fabs(this->Ay)==PI/2)
    	this->gyroYn  = 0.0;
    else
    	this->gyroYn  = -sin(this->Ax)/(1+pow(cos(this->Ax)*tan(this->Ay),2));

    if(this->accelZest>=0){
        this->gyroZn  = sqrt((1-pow(this->gyroXn,2)-pow(this->gyroYn,2)));
    }
    else {
    	this->gyroZn  = -sqrt((1-pow(this->gyroXn,2)-pow(this->gyroYn,2)));
    }

    // Trust Filter Acceleration Estimate
    this->accelXest   = (this->accelXn+this->trustX*this->gyroXn)/(1+this->trustX);
    this->accelYest   = (this->accelYn+this->trustY*this->gyroYn)/(1+this->trustY);
    this->accelZest   = (this->accelZn+this->trustZ*this->gyroZn)/(1+this->trustZ);
    this->accelRest   = sqrt(pow(this->accelXest,2)+pow(this->accelYest,2)+pow(this->accelZest,2));

    // Translational Acceleration
    this->AccelXB  = this->Racc*this->accelXest*this->accelRest;
    this->AccelYB  = this->Racc*this->accelYest*this->accelRest;
    this->AccelZB  = this->Racc*this->accelZest*this->accelRest;

    // Handling NaN conditions
    if(this->AccelXB!=this->AccelXB)
    	this->AccelXB = *(this->sens+0);
    if(this->AccelYB!=this->AccelYB)
    	this->AccelYB = *(this->sens+1);
    if(this->AccelZB!=this->AccelZB)
    	this->AccelZB = *(this->sens+2);

    // Convert from [-PI,PI] to [0,2PI]
	if(this->Ax<0)
		this->Ax = 2*PI+this->Ax;
	if(this->Ay<0)
		this->Ay = 2*PI+this->Ay;
	if(this->Az<0)
		this->Az = 2*PI+this->Az;

	// Gyroscope Kalman
	*(this->state+9)  = *(this->state+9) +TS*(*(this->state+12));
	*(this->state+10) = *(this->state+10)+TS*(*(this->state+13));
	*(this->state+11) = *(this->state+11)+TS*(*(this->state+14));

	PgX[0][0] = PgX[0][0]+TS*(TS*PgX[1][1]-PgX[0][1]-PgX[1][0]+QgX_angle);
	PgX[0][1] = PgX[0][1]-TS*PgX[1][1];
	PgX[1][0] = PgX[1][0]-TS*PgX[1][1];
	PgX[1][1] = PgX[1][1]+TS*QgX_bias;

	PgY[0][0] = PgY[0][0]+TS*(TS*PgY[1][1]-PgY[0][1]-PgY[1][0]+QgY_angle);
	PgY[0][1] = PgY[0][1]-TS*PgY[1][1];
	PgY[1][0] = PgY[1][0]-TS*PgY[1][1];
	PgY[1][1] = PgY[1][1]+TS*QgY_bias;

	PgZ[0][0] = PgZ[0][0]+TS*(TS*PgZ[1][1]-PgZ[0][1]-PgZ[1][0]+QgZ_angle);
	PgZ[0][1] = PgZ[0][1]-TS*PgZ[1][1];
	PgZ[1][0] = PgZ[1][0]-TS*PgZ[1][1];
	PgZ[1][1] = PgZ[1][1]+TS*QgZ_bias;

	errorgX   = this->Ax-*(this->state+9);
	errorgY   = this->Ay-*(this->state+10);
	errorgZ   = this->Az-*(this->state+11);

	// Innovation Co-variance
	SgX       = PgX[0][0]+RgX;
	SgY       = PgY[0][0]+RgY;
	SgZ       = PgZ[0][0]+RgZ;

	// Kalman Gain
	KgX[0] = PgX[0][0]/SgX;
	KgX[1] = PgX[1][0]/SgX;

	KgY[0] = PgY[0][0]/SgY;
	KgY[1] = PgY[1][0]/SgY;

	KgZ[0] = PgZ[0][0]/SgZ;
	KgZ[1] = PgZ[1][0]/SgZ;

	*(this->state+9) = *(this->state+9)+KgX[0]*errorgX;
	biasgX  = biasgX+KgX[1]*errorgX;

	*(this->state+10) = *(this->state+10)+KgY[0]*errorgY;
	biasgY  = biasgY+KgY[1]*errorgY;

	*(this->state+11) = *(this->state+11)+KgZ[0]*errorgZ;
	biasgZ  = biasgZ+KgZ[1]*errorgZ;

	// Error Co-variance
	PgX[0][0] = PgX[0][0]-KgX[0]*PgX[0][0];
	PgX[0][1] = PgX[0][1]-KgX[0]*PgX[0][1];
	PgX[1][0] = PgX[1][0]-KgX[1]*PgX[0][0];
	PgX[1][1] = PgX[1][1]-KgX[1]*PgX[0][1];

	PgY[0][0] = PgY[0][0]-KgY[0]*PgY[0][0];
	PgY[0][1] = PgY[0][1]-KgY[0]*PgY[0][1];
	PgY[1][0] = PgY[1][0]-KgY[1]*PgY[0][0];
	PgY[1][1] = PgY[1][1]-KgY[1]*PgY[0][1];

	PgZ[0][0] = PgZ[0][0]-KgZ[0]*PgZ[0][0];
	PgZ[0][1] = PgZ[0][1]-KgZ[0]*PgZ[0][1];
	PgZ[1][0] = PgZ[1][0]-KgZ[1]*PgZ[0][0];
	PgZ[1][1] = PgZ[1][1]-KgZ[1]*PgZ[0][1];

	*(this->rot+0)=*(this->rot+0)+TS*(*(this->state+12));
	*(this->rot+1)=*(this->rot+1)+TS*(*(this->state+13));
	*(this->rot+2)=0.0;//*(this->rot+2)=*(this->rot+2)+TS*(*(this->state+14));


}*/

void MPU6050::getIMUStates(){
	this->get6DOF();
}

/* CLOSE HANDLE */
MPU6050::~MPU6050(){
}
