/*
 * pwm.h
 *
 *  Created on: May 9, 2017
 *      Author: arvinddesikan
 */

#ifndef PWM_H_
#define PWM_H_
#define SET_PERIOD 10000000
#define SET_DUTYH  1860000
#define SET_DUTYL  1060000
#define PWMLOC	   "/sys/class/pwm/"
#define SLOTS	   "/sys/devices/bone_capemgr.9/slots"
#define PWMCAPE    "am33xx_pwm"
#define DIRECTION "out"
#define STATE 0
#define EDGE  "none"

class pwm{
	public:
		pwm(int id);
		void period(unsigned int period);
		void duty(unsigned int duty);
		void polarity(int polarity);
		void run(bool run);
		void terminate();
	private:
		int len;
		int fpwm;
		int address;
		char buff[50];
		char path[90];
};

void calibrate(pwm &pwm1,pwm &pwm2,pwm &pwm3,pwm &pwm4);
void initiatePWM();

#endif /* PWM_H_ */
