#include <stdio.h>
#include "../includes/pwm.h"
#include <unistd.h>

int main(){
//	initiatePWM();
	pwm pwm1(6); // MOTOR 1 P8.13
	pwm pwm2(1); // MOTOR 2 P9.21
	pwm pwm3(3); // MOTOR 3 P9.14
	pwm pwm4(2); // MOTOR 4 P9.42
	calibrate(pwm1,pwm2,pwm3,pwm4);
	pwm1.duty(1260000);
//	pwm2.duty(1260000);
//	pwm3.duty(1260000);
//	pwm4.duty(1260000);
	usleep(1000000);
/*	pwm1.duty(1560000);
	pwm2.duty(1560000);
	pwm3.duty(1560000);
	pwm4.duty(1560000);
	usleep(10000000);
	pwm1.duty(1360000);
	pwm2.duty(1360000);
	pwm3.duty(1360000);
	pwm4.duty(1360000);
	usleep(3000000);*/
	pwm1.duty(1060000);
/*	pwm2.duty(1060000);
	pwm3.duty(1060000);
	pwm4.duty(1060000);*/
	return 0;
}
