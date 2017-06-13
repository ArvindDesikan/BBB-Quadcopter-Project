#include <stdio.h>
#include "../includes/pwm.h"
#include <unistd.h>
#include "../includes/gpio.h"

// LED GPIO
gpio gpio1(115,DIRECTION,STATE,EDGE);// RED
pwm pwm1(6); // MOTOR 1 P8.13
pwm pwm2(1); // MOTOR 2 P9.21
pwm pwm3(3); // MOTOR 3 P9.14
pwm pwm4(2); // MOTOR 4 P9.42

int main(){
//	initiatePWM();
	int i = 1;


	calibrate(pwm1,pwm2,pwm3,pwm4);
	while(i<=20){
		gpio1.setvalue(1);
		usleep(500000);
		gpio1.setvalue(0);
		usleep(500000);
		++i;
	}
	i=1;
	pwm1.duty(1425000);
	pwm2.duty(1425000);
	pwm3.duty(1425000);
	pwm4.duty(1425000);
	usleep(2000000);
	gpio1.setvalue(1);
	usleep(750000);
	gpio1.setvalue(0);
/*	pwm1.duty(1428000);
	pwm2.duty(1428000);
	pwm3.duty(1428000);
	pwm4.duty(1428000);
	usleep(2000000);
	gpio1.setvalue(1);
	usleep(750000);
	gpio1.setvalue(0);
	pwm1.duty(1430000);
	pwm2.duty(1430000);
	pwm3.duty(1430000);
	pwm4.duty(1430000);
	usleep(3000000);
	gpio1.setvalue(1);
	usleep(750000);
	gpio1.setvalue(0);*/
	usleep(5000000);
	pwm1.duty(1260000);
	pwm2.duty(1260000);
	pwm3.duty(1260000);
	pwm4.duty(1260000);
	pwm1.duty(1060000);
	pwm2.duty(1060000);
	pwm3.duty(1060000);
	pwm4.duty(1060000);
	return 0;
}
