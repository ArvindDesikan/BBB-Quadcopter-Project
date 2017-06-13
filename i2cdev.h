/*
 * i2cdev.h
 *
 *  Created on: Mar 25, 2017
 *      Author: arvinddesikan
 * Description: I2C function instantiation
 */

#ifndef I2CDEV_H_
#define I2CDEV_H_

#include <stdint.h>

class i2cdevice{
private:
	unsigned int device;
	int file;
public:
	i2cdevice(unsigned int device);
	 bool open();
	 uint8_t  readbyte(uint8_t regaddr);
	 uint16_t readword(uint8_t regaddr);
	 uint8_t  readbits(uint8_t regaddr,unsigned int length,unsigned int bitnum);
	 bool writebyte(uint8_t regaddr,uint8_t data);
	 bool writebits(uint8_t regaddr,uint8_t data,unsigned int length,unsigned int bitnum);
	 void close();
	 ~i2cdevice();
};

#endif
