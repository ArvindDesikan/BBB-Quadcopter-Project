/*
 * gpio.cpp
 *
 *  Created on: May 9, 2017
 *      Author: arvinddesikan
 */

#include "gpio.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

gpio::gpio(int addr,char const *direct,int state,char const *edge,int val){
	this->address=addr;
	this->value = val;

	// Open GPIO
	sprintf(this->path,GPIO"/export");
	fgpio = open(this->path,O_WRONLY);
	if(fgpio<0){
		printf("GPIO Call failed!\n");
	}
	else {
		this->len=snprintf(this->buff,sizeof(this->buff),"%d",this->address);
		write(this->fgpio,this->buff,this->len);
	}
	close(fgpio);

	// Set Direction
	sprintf(this->path,GPIO"/gpio%d/direction",this->address);
	fgpio = open(this->path,O_WRONLY);
	if(fgpio<0){
		printf("Direction: GPIO set failed!\n");
	}
	else {
		this->len=snprintf(this->buff,sizeof(this->buff),"%s",direct);
		write(this->fgpio,this->buff,len);
	}
	close(fgpio);

	// Set state
	sprintf(this->path,GPIO"/gpio%d/active_low",this->address);
	fgpio = open(this->path,O_WRONLY);
	if(fgpio<0){
		printf("State: GPIO set failed!\n");
	}
	else {
		this->len=snprintf(this->buff,sizeof(this->buff),"%d",state);
		write(this->fgpio,this->buff,len);
	}
	close(fgpio);

	// Set edge
	sprintf(this->path,GPIO"/gpio%d/edge",this->address);
	fgpio = open(this->path,O_WRONLY);
	if(fgpio<0){
		printf("Edge: GPIO set failed!\n");
	}
	else {
		this->len=snprintf(this->buff,sizeof(this->buff),"%s",edge);
		write(this->fgpio,this->buff,this->len);
	}
	close(fgpio);

	// Set value
	if(strcmp(direct,"out")){

	}
	else{
		this->setvalue(val);
	}
}

void gpio::setvalue(int val){
	sprintf(this->path,GPIO"/gpio%d/value",this->address);
	fgpio = open(this->path,O_WRONLY);
	if(fgpio<0){
		printf("Value: GPIO set failed!\n");
	}
	else {
		this->len=snprintf(this->buff,sizeof(this->buff),"%d",val);
		write(this->fgpio,this->buff,len);
	}
	close(fgpio);
}

bool gpio::getvalue(){
	sprintf(this->path,GPIO"/gpio%d/value",this->address);
	this->fgpio = open(this->path,O_RDONLY);
	if(this->fgpio<0){
		printf("Value: GPIO read failed!\n");
	}
	else {
		read(this->fgpio,this->buff,len);
	}
	close(this->fgpio);
	return bool(int(buff[0])-48);
}
