#include "PID.h"

pid::pid(float KP,float KI,float KD,float N,float sampleTime,float initstate_int,float initstate_diff,float SATP,float SATN){
	this->kp 		= KP;
	this->ki 		= KI;
	this->kd 		= KD;
	this->filter	= N;
	this->Ts    	= sampleTime;
	this->accul     = initstate_int;
	this->diff 		= initstate_diff;
	this->satp 		= SATP;
	this->satn 		= SATN;
	this->err0		= 0.0;
}

float pid::controlInput(float err){
	this->diff  	 = this->kd*this->filter*(err-this->err0)-(this->filter*this->Ts-1)*this->diff;
	this->accul 	 = this->ki*this->Ts*err0+this->accul;
	this->controlval = this->kp*err+this->diff+this->accul;
	this->err0  	 = err;
	if(this->controlval>=this->satp)
		this->controlval = this->satp;
	else
		if(this->controlval<=this->satn)
			this->controlval = this->satn;
		else
	return (this->controlval);
}
