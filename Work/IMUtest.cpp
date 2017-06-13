#include <stdio.h>
#include "../includes/MPU6050.h"
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

MPU6050 IMU;
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

int main(){
	long i=1;
/*
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

*/	printf("START\n\n");
	printf("ZERO ERROR ACCELERATION:%f\t%f\t%f\n",IMU.zeroaX,IMU.zeroaY,IMU.zeroaZ);
	while(i!=5000){
		IMU.kalmanfilter();

/*
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
*/

//		printf("Position		:%f\t%f\t%f\n",IMU.state[0],IMU.state[1],IMU.state[2]);

//		printf("Velocity    	:%f\t%f\t%f\n",IMU.state[3],IMU.state[4],IMU.state[5]);

//		printf("Acceleration	:%f\t%f\t%f\n",IMU.state[6],IMU.state[7],IMU.state[8]);

		printf("Angular Position:%f\t%f\t%f\n",180/PI*IMU.state[9],180/PI*IMU.state[10],180/PI*IMU.state[11]);

//		printf("Angular Position:%f\t%f\t%f\n",180/PI*IMU.phi,180/PI*IMU.angY,180/PI*IMU.angZ);

//		printf("Angular Velocity:%f\t%f\t%f\n",180/PI*IMU.state[12],180/PI*IMU.state[13],180/PI*IMU.state[14]);

//		++i;
	}
/*
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
*/
 	return 0;
}

