/*
 * pwm.cpp
 *
 *  Created on: May 9, 2017
 *      Author: arvinddesikan
 */

#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include "gpio.h"
#include "pwm.h"

// LED GPIO
gpio gpio115(115,DIRECTION,STATE,EDGE);	// RED

pwm::pwm(int id){
	this->address=id;
	// Call PWM
	sprintf(this->path,PWMLOC"/export");
	this->fpwm = open(this->path,O_WRONLY);
	if(this->fpwm>=0){
		this->len=snprintf(this->buff,sizeof(this->buff),"%u",id);
		write(this->fpwm,this->buff,this->len);
	}
	else{
		printf("PWM call failed!\n");
	}
	close(this->fpwm);

	// Set Period
	sprintf(this->path,PWMLOC"/pwm%d/period_ns",this->address);
	this->fpwm = open(this->path,O_WRONLY);
	if(this->fpwm>=0){
		this->len=snprintf(this->buff,sizeof(this->buff),"%u",SET_PERIOD);
		write(this->fpwm,this->buff,this->len);
	}
	else{
		printf("PWM call failed!\n");
	}
	close(this->fpwm);

	// Set Polarity - ALWAYS AT ZERO
	sprintf(this->path,PWMLOC"/pwm%d/polarity",this->address);
	this->fpwm = open(this->path,O_WRONLY);
	if(this->fpwm>=0){
		this->len=snprintf(this->buff,sizeof(this->buff),"%u",0);
		write(this->fpwm,this->buff,this->len);
	}
	else{
		printf("PWM call failed!\n");
	}
	close(this->fpwm);
	this->duty(SET_DUTYH);
	this->run(1);
}

void pwm::run(bool run){
	// Set run
	sprintf(this->path,PWMLOC"/pwm%d/run",this->address);
	this->fpwm = open(this->path,O_WRONLY);
	if(this->fpwm>=0){
		this->len=snprintf(this->buff,sizeof(this->buff),"%u",run);
		write(this->fpwm,this->buff,this->len);
	}
	else{
		printf("PWM call failed!\n");
	}
	close(this->fpwm);
}

void pwm::duty(unsigned int duty){
	// Set Duty
	sprintf(this->path,PWMLOC"/pwm%d/duty_ns",this->address);
	this->fpwm = open(this->path,O_WRONLY);
	if(this->fpwm>=0){
		this->len=snprintf(this->buff,sizeof(this->buff),"%u",duty);
		write(this->fpwm,this->buff,this->len);
	}
	else{
		printf("PWM call failed!\n");
	}
	close(this->fpwm);
//	usleep(200);
}

void calibrate(pwm &pwm1,pwm &pwm2,pwm &pwm3,pwm &pwm4){
	gpio115.setvalue(0);
	int timer = 30;
//	printf("\nConnect Power Source.\n");
	gpio115.setvalue(1);
	while(timer)
	{
		usleep(1000000);
//		printf("Waiting for %d sec...\n",timer);
		--timer;
	}
	timer=10;
	gpio115.setvalue(0);
	pwm1.duty(SET_DUTYL);
	pwm2.duty(SET_DUTYL);
	pwm3.duty(SET_DUTYL);
	pwm4.duty(SET_DUTYL);
	while(timer)
	{
		gpio115.setvalue(1);
		usleep(250000);
		gpio115.setvalue(0);
		usleep(250000);
		--timer;
	}
}

void initiatePWM(){
	char path[80];
	char buff[20];
	int len;
	sprintf(path,SLOTS);
//	printf("%s\n",path);
	int fp;
	fp = open(path,O_WRONLY);
	if(fp>=0){
		len = snprintf(buff,sizeof(buff),"am33xx_pwm");
		write(fp,buff,len);
		len = snprintf(buff,sizeof(buff),"bone_pwm_P9_14");
		write(fp,buff,len);
		len = snprintf(buff,sizeof(buff),"bone_pwm_P9_21");
		write(fp,buff,len);
		len = snprintf(buff,sizeof(buff),"bone_pwm_P9_42");
		write(fp,buff,len);
		len = snprintf(buff,sizeof(buff),"bone_pwm_P8_13");
		write(fp,buff,len);
	}
	else {
		printf("Cape Initiation failed!!\n");
	}
	close(fp);
	usleep(1000000);
}

