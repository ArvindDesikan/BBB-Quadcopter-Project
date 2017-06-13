#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include "../includes/MPU6050.h"
#include "../includes/pwm.h"
#include "../includes/gpio.h"
#include "../includes/PID.h"
#include "../includes/supportfuncs.h"

/* Quadcopter Parameters */
#define L 			0.225						// Arm Length
#define Ixx 		0.0139						// X-Axis MOI
#define Iyy 		0.0139						// Y-Axis MOI
#define Izz 		0.0278						// Z-Axis MOI
#define M			1.1							// Quadcopter Mass

/* Motor Properties */
#define k			8.24						// Motor equation slope
#define b			5.0							// Motor Moment parameter
#define p			1.193						// Motor inverse power

/* Duty Cycle Scaling */
#define m			800.0
#define c			1060.0

/* Control-support function parameters */
#define RateVel 	10.0
#define RateAngPos	20.0
#define RateAngVel 	10.0
#define RateThrust	100.0
#define RateTorque	100.0
#define PhiLim		5*PI/12
#define ThetaLim	5*PI/12
#define ThrustLim	15.0
#define MAXTHRUST	10000

MPU6050 IMU;
pwm pwm1(6); 									// MOTOR 1 P8.13
pwm pwm2(1); 									// MOTOR 2 P9.21
pwm pwm3(3); 									// MOTOR 3 P9.14
pwm pwm4(2); 									// MOTOR 4 P9.42
pid pidX(5.0,0.0,8,100.0,TS,0.0,0.0);			// X-Axis Thrust Control
pid pidY(5.0,0.0,8,100.0,TS,0.0,0.0);			// Y-Axis Thrust Control
pid pidZ(5.0,0.0,8,100.0,TS,0.0,0.0);			// Z-Axis Thrust Control
pid pidphi(6.0,0.0,15,70.0,TS,0.0,0.0);		// Phi   Control
pid pidtheta(6.0,0.0,15,70.0,TS,0.0,0.0);		// Theta Control
pid pidpsi(6.0,0.0,15,70.0,TS,0.0,0.0);		// Psi   Control

/* Reference Translation Position */
float PosXC=0.0,PosYC=0.0,PosZC=0.05;			// Position vector variable

float VelXC,VelYC,VelZC;						// Velocity vector variable

float AccelXC,AccelYC,AccelZC;					// Velocity vector variable

float errX,errY,errZ;							// Translation Position Error
float TX,TY,TZ; 	 							// Thrust vector variable

/* Reference Angular Position */
float PhiC,ThetaC,PsiC; 						// Angle  variable
float angVelPhiC,angVelThetaC,angVelPsiC;		// Angular Velocity variable
float angAccelPhiC,angAccelThetaC,angAccelPsiC;	// Angular Acceleration variable
float errPhi,errTheta,errPsi;					// Angular Position Error
float TPhi,TTheta,TPsi;							// Torque vector variable

/* Motor control */
float U;			 							// Motor Thrust sum
float X1,X2,X3,X4; 								// Motor equation Thrust = 800*X^1.193
int duty1,duty2,duty3,duty4;					// Motor Duty Cycle

differentiate diffX1(TS,0.0),diffX2(TS,0.0),diffY1(TS,0.0),diffY2(TS,0.0),diffZ1(TS,0.02),diffZ2(TS,0.0);
differentiate diffphi1(TS,0.0),diffphi2(TS,0.0),difftheta1(TS,0.0),difftheta2(TS,0.0),diffpsi1(TS,0.0),diffpsi2(TS,0.0);

ratelimiter	ratelimvelX(RateVel,-RateVel,TS,0.0),ratelimvelY(RateVel,-RateVel,TS,0.0),ratelimvelZ(RateVel,-RateVel,TS,0.0);
ratelimiter ratelimphi(RateAngPos,-RateAngPos,TS,0.0),ratelimtheta(RateAngPos,-RateAngPos,TS,0.0),ratelimpsi(RateAngPos,-RateAngPos,TS,0.0);
ratelimiter ratelimvelphi(RateAngVel,-RateAngVel,TS,0.0),ratelimveltheta(RateAngVel,-RateAngVel,TS,0.0),ratelimvelpsi(RateAngVel,-RateAngVel,TS,0.0);
ratelimiter ratelimabsthrust(RateThrust,-RateThrust,TS,0.0),ratelimTPhi(RateTorque,-RateTorque,TS,0.0),ratelimTTheta(RateTorque,-RateTorque,TS,0.0),ratelimTPsi(RateTorque,-RateTorque,TS,0.0);

saturate satTX(ThrustLim,-ThrustLim),satTY(ThrustLim,-ThrustLim),satTZ(ThrustLim,0.0);
saturate satphi(PhiLim,-PhiLim),sattheta(ThetaLim,-ThetaLim),satpsi(100,-100),satmaxthrust(MAXTHRUST,0);
saturate satD1(1.0,0.0),satD2(1.0,0.0),satD3(1.0,0.0),satD4(1.0,0.0);

FILE *fpX;
FILE *fpY;
FILE *fpZ;
FILE *fvX;
FILE *fvY;
FILE *fvZ;
FILE *faX;
FILE *faY;
FILE *faZ;
FILE *fangleX;
FILE *fangleY;
FILE *fangleZ;
FILE *fangleVelX;
FILE *fangleVelY;
FILE *fangleVelZ;
FILE *Duty1;
FILE *Duty2;
FILE *Duty3;
FILE *Duty4;
FILE *absThrust;
FILE *TorqueX;
FILE *TorqueY;
FILE *TorqueZ;

// LED
gpio gpio1(115,DIRECTION,STATE,EDGE);// RED

int main(){
	long i = 1;
	faX = fopen("/media/BBB/IMUTesting/IMUXAccTest.txt","w");
	faY = fopen("/media/BBB/IMUTesting/IMUYAccTest.txt","w");
	faZ = fopen("/media/BBB/IMUTesting/IMUZAccTest.txt","w");
	fvX	= fopen("/media/BBB/IMUTesting/IMUXVelTest.txt","w");
	fvY	= fopen("/media/BBB/IMUTesting/IMUYVelTest.txt","w");
	fvZ	= fopen("/media/BBB/IMUTesting/IMUZVelTest.txt","w");
	fpX	= fopen("/media/BBB/IMUTesting/IMUXPosTest.txt","w");
	fpY	= fopen("/media/BBB/IMUTesting/IMUYPosTest.txt","w");
	fpZ	= fopen("/media/BBB/IMUTesting/IMUZPosTest.txt","w");
	fangleX = fopen("/media/BBB/IMUTesting/IMUXAngTest.txt","w");
	fangleY = fopen("/media/BBB/IMUTesting/IMUYAngTest.txt","w");
	fangleZ = fopen("/media/BBB/IMUTesting/IMUZAngTest.txt","w");
	fangleVelX = fopen("/media/BBB/IMUTesting/IMUXAngVelTest.txt","w");
	fangleVelY = fopen("/media/BBB/IMUTesting/IMUYAngVelTest.txt","w");
	fangleVelZ = fopen("/media/BBB/IMUTesting/IMUZAngVelTest.txt","w");
	Duty1 	   = fopen("/media/BBB/IMUTesting/duty1.txt","w");
	Duty2 	   = fopen("/media/BBB/IMUTesting/duty2.txt","w");
	Duty3 	   = fopen("/media/BBB/IMUTesting/duty3.txt","w");
	Duty4 	   = fopen("/media/BBB/IMUTesting/duty4.txt","w");
	TorqueX    = fopen("/media/BBB/IMUTesting/TPhi.txt","w");
	TorqueY    = fopen("/media/BBB/IMUTesting/TTheta.txt","w");
	TorqueZ    = fopen("/media/BBB/IMUTesting/TPsi.txt","w");
	absThrust  = fopen("/media/BBB/IMUTesting/absThrust.txt","w");

	calibrate(pwm1,pwm2,pwm3,pwm4);
	while(i<=20){
		gpio1.setvalue(1);
		usleep(500000);
		gpio1.setvalue(0);
		usleep(500000);
		++i;
	}
	gpio1.setvalue(0);
	i=1;
	usleep(5000000);
	while(i!=2000){
	IMU.kalmanfilter();

	errX = PosXC-IMU.state[0];		// X-Position Error
	errY = PosYC-IMU.state[1];		// Y-Position Error
	errZ = PosZC-IMU.state[2];		// Z-Position Error

	diffX1.perform(VelXC,PosXC);	// Reference Velocity X-Axis
	diffY1.perform(VelYC,PosYC);	// Reference Velocity Y-Axis
	diffZ1.perform(VelZC,PosZC);	// Reference Velocity Z-Axis

	ratelimvelX.perform(VelXC);		// Rate Limit X-Axis
	ratelimvelY.perform(VelYC);		// Rate Limit Y-Axis
	ratelimvelZ.perform(VelZC); 	// Rate Limit Z-Axis

	diffX2.perform(AccelXC,VelXC);	// Reference Acceleration X-Axis
	diffY2.perform(AccelYC,VelYC);	// Reference Acceleration Y-Axis
	diffZ2.perform(AccelZC,VelZC);	// Reference Acceleration Z-Axis

	// Thrust with gravity Compensation
	TX = M*(AccelXC+pidX.controlInput(errX));
	TY = M*(AccelYC+pidY.controlInput(errY));
	TZ = M*(AccelZC+pidZ.controlInput(errZ)-G);

	// Saturate Thrust Vector
	satTX.perform(TX);
	satTY.perform(TY);
	satTZ.perform(TZ);

	// Reference Generation - Phi Theta Psi U
	if(TZ!=0){
		PsiC    = 0.0;
		PhiC= atan2((TX*sin(PsiC)-TY*cos(PsiC)),TZ);
		ThetaC 	= atan2(((TX*cos(PsiC)+TY*sin(PsiC))*cos(PhiC)),TZ);
		U		= TZ/(cos(PhiC)*cos(ThetaC));
	}
	else{
		PsiC 	= atan2(TY,TX);
		PhiC	= 0.0;
		ThetaC	= PI/2;
		U 		= sqrt(pow(TX,2)+pow(TY,2));
	}

	// Saturate Angle
	satphi.perform(PhiC);
	sattheta.perform(ThetaC);
	satpsi.perform(PsiC);

	// Saturate Thrust
	satmaxthrust.perform(U);

	errPhi 	= PhiC  -IMU.state[9];					// Phi Error
	errTheta= ThetaC-IMU.state[10];					// Theta Error
	errPsi 	= PsiC  -IMU.state[11];					// Psi Error

	ratelimphi.perform(PhiC);						// Rate Limit Phi
	ratelimtheta.perform(ThetaC);					// Rate Limit Theta
	ratelimpsi.perform(PsiC);						// Rate Limit Psi

	diffphi1.perform(angVelPhiC,PhiC);				// Differentiate Phi
	difftheta1.perform(angVelThetaC,ThetaC);		// Differentiate Theta
	diffpsi1.perform(angVelPsiC,PsiC);				// Differentiate Psi

	ratelimvelphi.perform(angVelPhiC);				// Rate Limit Vel Phi
	ratelimveltheta.perform(angVelThetaC);			// Rate Limit Vel Theta
	ratelimvelpsi.perform(angVelPsiC);				// Rate Limit Vel Psi

	diffphi2.perform(angAccelPhiC,angVelPhiC);		// Differentiate Phi
	difftheta2.perform(angAccelThetaC,angVelThetaC);// Differentiate Theta
	diffpsi2.perform(angAccelPsiC,angVelPsiC);		// Differentiate Psi

    // Phi Theta Psi Control (Add Reference Compensation) (evaluate err)
    TPhi 	= Ixx*(angAccelPhiC+pidphi.controlInput(errPhi)-(Iyy*IMU.state[13]*IMU.state[14] - Izz*IMU.state[13]*IMU.state[14])/Ixx);
    TTheta 	= Iyy*(angAccelThetaC+pidtheta.controlInput(errTheta)+(Ixx*IMU.state[12]*IMU.state[14] - Izz*IMU.state[12]*IMU.state[14])/Iyy);
    TPsi   	= Izz*(angAccelPsiC+pidpsi.controlInput(errPsi)-(Ixx*IMU.state[12]*IMU.state[13] - Iyy*IMU.state[12]*IMU.state[13])/Izz);

    ratelimTPhi.perform(TPhi);
    ratelimTTheta.perform(TTheta);
    ratelimTPsi.perform(TPsi);
    ratelimabsthrust.perform(U);

    // Convert Torque and Thrust to motor duty cycles
    // [1/(4*k) 0 -L/(2*k) 1/(4*b);1/(4*k) L/(2*k) 0 -1/(4*b);1/(4*k) 0 L/(2*k) 1/(4*b);1/(4*k) -L/(2*k) 0 -1/(4*b)]
    X1 		= exp(log(fabs(1/(4*k)*U-L/(2*k)*TTheta+1/(4*b)*TPsi))/p);
    X2 		= exp(log(fabs(1/(4*k)*U+L/(2*k)*TPhi  -1/(4*b)*TPsi))/p);
    X3 		= exp(log(fabs(1/(4*k)*U+L/(2*k)*TTheta+1/(4*b)*TPsi))/p);
    X4 		= exp(log(fabs(1/(4*k)*U-L/(2*k)*TPhi  -1/(4*b)*TPsi))/p);

    // Saturate Duty Cycle
	satD1.perform(X1);
	satD2.perform(X2);
	satD3.perform(X3);
	satD4.perform(X4);

	// Duty Cycle slope
	duty1 = (int)(800000.0*X1+1060000.0);
	duty2 = (int)(800000.0*X2+1060000.0);
	duty3 = (int)(800000.0*X3+1060000.0);
	duty4 = (int)(800000.0*X4+1060000.0);

	pwm1.duty(duty1);
	pwm2.duty(duty2);
	pwm3.duty(duty3);
	pwm4.duty(duty4);

	//printf("%f\t%f\t%f\t%f\n",errX,errY,errZ);
	//printf("%f\t%f\t%f\t%f\n",X1,X2,X3,X4);
	fprintf(fpX,"%f\n",IMU.state[0]);
	fprintf(fpY,"%f\n",IMU.state[1]);
	fprintf(fpZ,"%f\n",IMU.state[2]);
	fprintf(fvX,"%f\n",IMU.state[3]);
	fprintf(fvY,"%f\n",IMU.state[4]);
	fprintf(fvZ,"%f\n",IMU.state[5]);
	fprintf(faX,"%f\n",IMU.state[6]);
	fprintf(faY,"%f\n",IMU.state[7]);
	fprintf(faZ,"%f\n",IMU.state[8]);
	fprintf(fangleX,"%f\n",IMU.state[9]);
	fprintf(fangleY,"%f\n",IMU.state[10]);
	fprintf(fangleZ,"%f\n",IMU.state[11]);
	fprintf(fangleVelX,"%f\n",IMU.state[12]);
	fprintf(fangleVelY,"%f\n",IMU.state[13]);
	fprintf(fangleVelZ,"%f\n",IMU.state[14]);
	fprintf(Duty1,"%f\n",X1);
	fprintf(Duty2,"%f\n",X2);
	fprintf(Duty3,"%f\n",X3);
	fprintf(Duty4,"%f\n",X4);
	//usleep(1000);
	++i;
	}
	pwm1.duty(1060000);
	pwm2.duty(1060000);
	pwm3.duty(1060000);
	pwm4.duty(1060000);
 	fclose(fpX);
 	fclose(fpY);
 	fclose(fpZ);
 	fclose(fvX);
 	fclose(fvY);
 	fclose(fvZ);
	fclose(faX);
 	fclose(faY);
 	fclose(faZ);
 	fclose(fangleX);
 	fclose(fangleY);
 	fclose(fangleZ);
 	fclose(fangleVelX);
 	fclose(fangleVelY);
 	fclose(fangleVelZ);
 	fclose(Duty1);
 	fclose(Duty2);
 	fclose(Duty3);
 	fclose(Duty4);
 	fclose(absThrust);
 	fclose(TorqueX);
 	fclose(TorqueY);
 	fclose(TorqueZ);
}
