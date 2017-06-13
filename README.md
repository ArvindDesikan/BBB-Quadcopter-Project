# BBB-Quadcopter-Project
Project Code for a BEAGLEBONE BLACK QUADCOPTER

The code implements pipeline stream and i2c communication as suitable. 

So far the sensor,PID and PWM have been tested against a virtual model of the quadcopter - VirtualFlightStability.cpp

Testing against the sensor data is causing the controller to reach duty cycle saturation - FlightStability.cpp
- This is being investigated with MRAS to filter the IMU based estimation of velocity and position which accumulate drift error over time.

Call ./Work/makeall to build a ".cpp". The executable is stored at /Executables/
