#include <stdio.h>
#include "../includes/MPU6050.h"

MPU6050 IMU;
FILE *fa;
FILE *fv;
FILE *fp;

int main(){
	int i = 1;
	fa = fopen("/media/BBB/IMUZAccQuality.txt","w");
	fv = fopen("/media/BBB/IMUZVelQuality.txt","w");
	fp = fopen("/media/BBB/IMUZPosQuality.txt","w");
	while(i!=2000){
		IMU.TrustFilter();
		fprintf(fa,"%f\n",IMU.state[8]);
		fprintf(fv,"%f\n",IMU.state[5]);
		fprintf(fp,"%f\n",IMU.state[2]);
		++i;
	}
	fclose(fa);
	fclose(fv);
	fclose(fp);
	return 0;
}
