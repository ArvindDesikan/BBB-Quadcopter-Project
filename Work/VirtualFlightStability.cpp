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

pid pidX(20.0,0.0,10,100.0,TS,0.0,0.0);			// X-Axis Thrust Control
pid pidY(20.0,0.0,10,100.0,TS,0.0,0.0);			// Y-Axis Thrust Control
pid pidZ(20.0,0.0,10,100.0,TS,0.0,0.0);			// Z-Axis Thrust Control
pid pidphi(60.0,0.0,20,70.0,TS,0.0,0.0);		// Phi   Control
pid pidtheta(60.0,0.0,20,70.0,TS,0.0,0.0);		// Theta Control
pid pidpsi(60.0,0.0,20,70.0,TS,0.0,0.0);		// Psi   Control

/* Reference Translation Position */
float PosXC=0.0,PosYC=0.0,PosZC=0.1;			// Position vector variable

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
float TPhix,TThetax,TPsix;

/* Motor control */
float Ux;
float U;			 							// Motor Thrust sum
float X1,X2,X3,X4; 								// Motor equation Thrust = 800*X^1.193
int duty1,duty2,duty3,duty4;					// Motor Duty Cycle

/* State Vector */
float phi,theta,psi;
float P,Q,R;
float Ax,Ay,Az;
float angAccelPhi,angAccelTheta,angAccelPsi;
float Rot[3][3];
float state[14];

differentiate diffX1(TS,0.0),diffX2(TS,0.0),diffY1(TS,0.0),diffY2(TS,0.0),diffZ1(TS,0.1),diffZ2(TS,0.0);
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
FILE *absThrustx;
FILE *TorqueX;
FILE *TorqueXx;
FILE *TorqueY;
FILE *TorqueYx;
FILE *TorqueZ;
FILE *TorqueZx;


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
	TorqueXx   = fopen("/media/BBB/IMUTesting/TPhix.txt","w");
	TorqueY    = fopen("/media/BBB/IMUTesting/TTheta.txt","w");
	TorqueYx   = fopen("/media/BBB/IMUTesting/TThetax.txt","w");
	TorqueZ    = fopen("/media/BBB/IMUTesting/TPsi.txt","w");
	TorqueZx   = fopen("/media/BBB/IMUTesting/TPsix.txt","w");
	absThrust  = fopen("/media/BBB/IMUTesting/absThrust.txt","w");
	absThrustx= fopen("/media/BBB/IMUTesting/absThrustx.txt","w");

	while(i!=5000){

	/* Quadcopter Model */
	Rot[0][0]  =  cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta);
	Rot[0][1]  = -cos(phi)*sin(psi);
	Rot[0][2]  =  cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi);
	Rot[1][0]  =  cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta);
	Rot[1][1]  =  cos(phi)*cos(psi);
	Rot[1][2]  =  sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi);
	Rot[2][0]  = -cos(phi)*sin(theta);
	Rot[2][1]  =  sin(phi);
	Rot[2][2]  =  cos(phi)*cos(theta);

	Ax		 = Rot[0][2]*U/M;
	Ay		 = Rot[1][2]*U/M;
	Az		 = G+Rot[2][2]*U/M;

	angAccelPhi   = (TPhi + Iyy*Q*R - Izz*Q*R)/Ixx;
	angAccelTheta = (TTheta - Ixx*P*R + Izz*P*R)/Iyy;
	angAccelPsi	  = (TPsi + Ixx*P*Q - Iyy*P*Q)/Izz;

	P		 = P + TS*angAccelPhi;
	Q		 = Q + TS*angAccelTheta;
	R		 = R + TS*angAccelPsi;

	phi	 	 = phi	 + TS*(P*cos(theta) + R*sin(theta));
	theta	 = theta + TS*(Q - (R*cos(theta)*sin(phi))/cos(phi) + (P*sin(phi)*sin(theta))/cos(phi));
	psi  	 = psi	 + TS*((R*cos(theta))/cos(phi) - (P*sin(theta))/cos(phi));

	*(state+6) = Ax;
	*(state+7) = Ay;
	*(state+8) = Az;

	*(state+3) = *(state+6)*TS+*(state+3);
	*(state+4) = *(state+7)*TS+*(state+4);
	*(state+5) = *(state+8)*TS+*(state+5);
	*(state+0) = *(state+3)*TS+*(state+0);
	*(state+1) = *(state+4)*TS+*(state+1);
	*(state+2) = *(state+5)*TS+*(state+2);

	*(state+12)= P;
	*(state+13)= Q;
	*(state+14)= R;

	*(state+ 9)= phi;
	*(state+10)= theta;
	*(state+11)= psi;

	errX = PosXC-state[0];			// X-Position Error
	errY = PosYC-state[1];			// Y-Position Error
	errZ = PosZC-state[2];			// Z-Position Error

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

	errPhi 	= PhiC  -state[9];						// Phi Error
	errTheta= ThetaC-state[10];						// Theta Error
	errPsi 	= PsiC  -state[11];						// Psi Error

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
    TPhi 	= Ixx*(angAccelPhiC+pidphi.controlInput(errPhi)-(Iyy*state[13]*state[14] - Izz*state[13]*state[14])/Ixx);
    TTheta 	= Iyy*(angAccelThetaC+pidtheta.controlInput(errTheta)+(Ixx*state[12]*state[14] - Izz*state[12]*state[14])/Iyy);
    TPsi   	= Izz*(angAccelPsiC+pidpsi.controlInput(errPsi)-(Ixx*state[12]*state[13] - Iyy*state[12]*state[13])/Izz);

    Ux		= U;
    TPhix	= TPhi;
    TThetax	= TTheta;
    TPsix	= TPsi;

    ratelimTPhi.perform(TPhix);
    ratelimTTheta.perform(TThetax);
    ratelimTPsi.perform(TPsix);
    ratelimabsthrust.perform(Ux);

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

	//printf("%f\t%f\t%f\t%f\n",errX,errY,errZ);
	//printf("%f\t%f\t%f\t%f\n",X1,X2,X3,X4);
	fprintf(fpX,"%f\n",state[0]);
	fprintf(fpY,"%f\n",state[1]);
	fprintf(fpZ,"%f\n",state[2]);
	fprintf(fvX,"%f\n",state[3]);
	fprintf(fvY,"%f\n",state[4]);
	fprintf(fvZ,"%f\n",state[5]);
	fprintf(faX,"%f\n",state[6]);
	fprintf(faY,"%f\n",state[7]);
	fprintf(faZ,"%f\n",state[8]);
	fprintf(fangleX,"%f\n",state[9]);
	fprintf(fangleY,"%f\n",state[10]);
	fprintf(fangleZ,"%f\n",state[11]);
	fprintf(fangleVelX,"%f\n",state[12]);
	fprintf(fangleVelY,"%f\n",state[13]);
	fprintf(fangleVelZ,"%f\n",state[14]);
	fprintf(Duty1,"%f\n",X1);
	fprintf(Duty2,"%f\n",X2);
	fprintf(Duty3,"%f\n",X3);
	fprintf(Duty4,"%f\n",X4);
	fprintf(TorqueX,"%f\n",TPhi);
	fprintf(TorqueY,"%f\n",TTheta);
	fprintf(TorqueZ,"%f\n",TPsi);
	fprintf(absThrust,"%f\n",U);
	fprintf(absThrustx,"%f\n",Ux);
	fprintf(TorqueXx,"%f\n",TPhix);
	fprintf(TorqueYx,"%f\n",TThetax);
	fprintf(TorqueZx,"%f\n",TPsix);
	++i;
	}
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
 	fclose(TorqueX);
 	fclose(TorqueY);
 	fclose(TorqueZ);
 	fclose(absThrust);
 	fclose(absThrustx);
 	fclose(TorqueXx);
 	fclose(TorqueYx);
 	fclose(TorqueZx);
}
