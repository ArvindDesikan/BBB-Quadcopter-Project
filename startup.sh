#!/bin/bash
echo "cape-universaln">>/sys/devices/bone_capemgr.9/slots
config-pin P9.21 pwm
config-pin P9.42 pwm
config-pin P9.14 pwm
config-pin P8.13 pwm
#nice -19 /root/UnityX/Executables/TestLift

