#include <stdio.h>
#include "../includes/i2cdev.h"
#include "../includes/MPU6050.h"

MPU6050 IMU;

int main(){
	while(1){
		IMU.get6DOF();
		printf("%f\t%f\t%f\n",IMU.sens[3],IMU.sens[4],IMU.sens[5]);
	}
	return 0;
}

