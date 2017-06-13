#include "supportfuncs.h"

differentiate::differentiate(float time,float init){
	this->sample_time = time;
	this->vector_k1	  = init;
}

void differentiate::perform(float diff,float vector_k){
	diff = (vector_k-this->vector_k1)/this->sample_time;
	this->vector_k1 = vector_k;
}

ratelimiter::ratelimiter(float rising,float falling,float time,float init){
	this->rising_rate  = rising;
	this->falling_rate = falling;
	this->sample_time  = time;
	this->vector_k1    = init;
}

void ratelimiter::perform(float &vector_k){
	this->rate = (vector_k-this->vector_k1)/this->sample_time;
	if(this->rate>=this->rising_rate)
		vector_k = this->sample_time*this->rising_rate+this->vector_k1;
	else
		if(this->rate<=this->falling_rate)
			vector_k = this->sample_time*this->falling_rate+this->vector_k1;
	this->vector_k1 = vector_k;
}

saturate::saturate(float plim,float nlim){
	this->limp = plim;
	this->limn = nlim;
}

void saturate::perform(float &vector_k){
	if(vector_k>=limp)
		vector_k = limp;
	else
		if(vector_k<=limn)
			vector_k = limn;
}
