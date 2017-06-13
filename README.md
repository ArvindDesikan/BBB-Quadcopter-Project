# BBB-Quadcopter-Project
Project Code for a BEAGLEBONE BLACK QUADCOPTER
![beaglebone quadcopter](https://user-images.githubusercontent.com/9638192/27069399-8f528f76-5025-11e7-8cd4-520262d83d1c.jpg)

The code implements pipeline stream and i2c communication as suitable. 

So far the sensor,PID and PWM have been tested against a virtual model of the quadcopter - VirtualFlightStability.cpp

Testing against the sensor data is causing the controller to reach duty cycle saturation - FlightStability.cpp
- This is being investigated with MRAS to filter the IMU based estimation of velocity and position which accumulate drift error over time.

Call ./Work/makeall to build a ".cpp". The executable is stored at /Executables/

Before executing any file, execute startup.sh
- It will load the necessary pwm device-tree-overlays and create dynamic folders in /sys/class/pwm
