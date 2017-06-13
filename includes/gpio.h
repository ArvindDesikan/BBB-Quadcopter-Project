/*
 * gpio.h
 *
 *  Created on: May 9, 2017
 *      Author: arvinddesikan
 */

#ifndef GPIO_H_
#define GPIO_H_
#define GPIO "/sys/class/gpio/"

class gpio{
	private: int address;
			 char path[80];
			 int fgpio;
			 int len;
			 int value;
			 char buff[50];
	public:
			 gpio(int addr,char const *direct,int state,char const *edge,int val=0);
			 void closegpio();
			 void setvalue(int val);
			 bool getvalue();
};

#endif /* GPIO_H_ */

