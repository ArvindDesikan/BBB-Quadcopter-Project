#ifndef PID_H_
#define PID_H_

class pid{
//private:
protected:
	float kp;
	float ki;
	float kd;
	float Ts;
	float filter;
	float accul;
	float diff;
	float satp;
	float satn;
	float err0;
	float controlval;
public:
	pid(float KP,float KI,float KD,float N,float sampleTime,float initstate_int=0.0,float initstate_diff=0.0,float SATP=1000000,float SATN=-1000000);
	float controlInput(float err);
};

#endif
