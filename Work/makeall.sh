INCLUDES="/root/UnityX/includes"
EXECUTABLES="/root/UnityX/Executables"
WORK="/root/UnityX/Work"

export INCLUDES
export EXECUTABLES

# GPIO TEST MAKE
#g++ $WORK/gpiotest.cpp $INCLUDES/gpio.cpp -o $EXECUTABLES/gpiotest

# PWM TEST MAKE
#g++ $WORK/pwmtest.cpp $INCLUDES/pwm.cpp $INCLUDES/gpio.cpp -o $EXECUTABLES/pwmtest

# IMU TEST MAKE
#g++ $WORK/IMUtest.cpp $INCLUDES/MPU6050.cpp $INCLUDES/i2cdev.cpp -o $EXECUTABLES/IMUtest

# I2C DEV TEST MAKE
#g++ $WORK/i2cdevtest.cpp $INCLUDES/MPU6050.cpp $INCLUDES/i2cdev.cpp -o $EXECUTABLES/i2cdevtest

# IMU QUALITY TEST MAKE
#g++ $WORK/IMUQualitytest.cpp $INCLUDES/MPU6050.cpp $INCLUDES/i2cdev.cpp -o $EXECUTABLES/IMUQualitytest

# FLIGHT STABILITY MAKE
g++ $WORK/FlightStability.cpp $INCLUDES/gpio.cpp $INCLUDES/pwm.cpp $INCLUDES/MPU6050.cpp $INCLUDES/i2cdev.cpp $INCLUDES/supportfuncs.cpp $INCLUDES/PID.cpp -o $EXECUTABLES/FlightStability

# VIRTUAL FLIGHT STABILITY MAKE
g++ $WORK/VirtualFlightStability.cpp $INCLUDES/gpio.cpp $INCLUDES/pwm.cpp $INCLUDES/MPU6050.cpp $INCLUDES/i2cdev.cpp $INCLUDES/supportfuncs.cpp $INCLUDES/PID.cpp -o $EXECUTABLES/VirtualFlightStability

# TEST LIFT MAKE
#g++ $WORK/TestLift.cpp $INCLUDES/pwm.cpp $INCLUDES/gpio.cpp -o $EXECUTABLES/TestLift 

# FLY MAKE
#g++ $WORK/Fly.cpp $INCLUDES/pwm.cpp $INCLUDES/gpio.cpp -o $EXECUTABLES/Fly


