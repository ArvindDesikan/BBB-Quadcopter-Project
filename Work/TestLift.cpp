#include <stdio.h>
#include "../includes/pwm.h"
#include "../includes/gpio.h"
#include <unistd.h>

// LED GPIO
gpio gpio1(115,DIRECTION,STATE,EDGE);// RED
pwm pwm1(6); // MOTOR 1 P8.13
pwm pwm2(1); // MOTOR 2 P9.21
pwm pwm3(3); // MOTOR 3 P9.14
pwm pwm4(2); // MOTOR 4 P9.42

int main(){
	int i=1;
	int duty = 1460000;
//	initiatePWM();
	pwm pwm1(6); // MOTOR 1 P8.13
	pwm pwm2(1); // MOTOR 2 P9.21
	pwm pwm3(3); // MOTOR 3 P9.14
	pwm pwm4(2); // MOTOR 4 P9.42

	calibrate(pwm1,pwm2,pwm3,pwm4);
	while(i<=20){
		gpio1.setvalue(1);
		usleep(500000);
		gpio1.setvalue(0);
		usleep(500000);
		++i;
	}
	i=1;
	while(duty!=15400000){
		printf("%d\n",duty);
		pwm1.duty(duty);
		pwm2.duty(duty);
		pwm3.duty(duty);
		pwm4.duty(duty);
		duty=duty+20000;
		usleep(6000000);
	}
	pwm1.duty(1060000);
	pwm2.duty(1060000);
	pwm3.duty(1060000);
	pwm4.duty(1060000);
	return 0;
}
