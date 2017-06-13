#ifndef SUPPORTFUNCS_H_
#define SUPPORTFUNCS_H_

struct differentiate{
public:
	float vector_k1;
	float sample_time;
	differentiate(float time,float init);
	void perform(float diff,float vector_k);
};

struct ratelimiter{
public:
	float vector_k1;
	float rising_rate;
	float falling_rate;
	float sample_time;
	float rate;
	ratelimiter(float rising,float falling,float time,float init);
	void perform(float &vector_k);
};

struct saturate{
public:
	float limp;
	float limn;
	saturate(float plim,float nlim);
	void perform(float &vector_k);
};

#endif
