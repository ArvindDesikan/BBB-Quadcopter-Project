/*
 * i2cdev.cpp
 *
 *  Created on: Mar 25, 2017
 *      Author: arvinddesikan
 * Description: I2C Function Definition
 */

#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include "i2cdev.h"

i2cdevice::i2cdevice(unsigned int device){
	this->file   =  -1;
	this->device = device;
	this->open();
}

bool i2cdevice::open(){
	this->file=::open("/dev/i2c-1",O_RDWR);
	if(this->file<0){
		perror("I2C:Failed to open bus.\n");
		return false;
	}
	if(ioctl(this->file,I2C_SLAVE,this->device)<0){
		perror("I2C:Failed to connect to device\n");
		return false;
	}
	return true;
}

uint8_t i2cdevice::readbyte(uint8_t regaddr){
	uint8_t buffer[1];
	buffer[0]=regaddr;
	if(write(this->file,buffer,1)!=1){
		perror("I2C:Failed to access device register\n");
		return 0;
	}
	else{
		if(read(this->file,buffer,1)!=1){
			perror("I2C:Failed to read the value\n");
			return 0;
		}
	return buffer[0];
	}
}

uint16_t i2cdevice::readword(uint8_t regaddr){
	int len;
	uint8_t buffer_data[2]={0,0};
	uint8_t buffer_addr[1]={0};
	buffer_addr[0]        =regaddr;
	if(write(this->file,buffer_addr,1)!=1){
		perror("I2C:Failed to access device register\n");
		return 0;
	}
	else{
		if((len=read(this->file,buffer_data,2))!=2){
			return 0;
		}
	return (uint16_t(buffer_data[0]<<8|buffer_data[1]));
	}
}

uint8_t i2cdevice::readbits(uint8_t regaddr,unsigned int length,unsigned int bitnum){
	uint8_t buffer[1];
	buffer[0]=regaddr;
	if(write(this->file,buffer,1)!=1){
		perror("I2C:Failed to access device register\n");
		return 0;
	}
	else{
		if(read(this->file,buffer,1)!=1){
			perror("I2C:Failed to read the value\n");
			return 0;
		}
		buffer[0] = uint8_t(buffer[0]&uint8_t(uint8_t(0xff<<(8-length))>>(8-length-bitnum))>>bitnum);
	}
	return buffer[0];
}

bool i2cdevice::writebyte(uint8_t regaddr,uint8_t data){
	uint8_t buffer[1];
	buffer[0]=regaddr;
	buffer[1]=data;
	if(write(this->file,buffer,2)!=2){
		perror("I2C:Failed to access device register\n");
		return false;
	}
	return true;
}

bool i2cdevice::writebits(uint8_t regaddr,uint8_t data,unsigned int length,unsigned int bitnum){
	uint8_t buffer[1];
	uint8_t temp=this->readbyte(regaddr);
	temp=(temp&~((0xff<<(8-length))>>(8-bitnum-length)))|(data<<bitnum);
	return this->writebyte(regaddr,temp);
}

void i2cdevice::close(){
	::close(this->file);
	this->file = -1;
}

i2cdevice::~i2cdevice(){
	if(file!=-1){
		this->close();
	}
}

