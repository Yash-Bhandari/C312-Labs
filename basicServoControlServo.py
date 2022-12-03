from time import *
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

kit.servo[0].set_pulse_width_range(500, 2500)
kit.servo[1].set_pulse_width_range(500, 2500)

kit.servo[3].set_pulse_width_range(500, 2500)




# == Base: 0 ==
kit.servo[0].angle = 0
sleep(1)
kit.servo[0].angle = 90
sleep(1)
kit.servo[0].angle = 180
sleep(1)

# == Shoulder: 1 == 
kit.servo[1].angle = 0
sleep(1)
kit.servo[1].angle = 90
sleep(1)
kit.servo[1].angle = 180
sleep(1)

# == Elbow - 2: 3 ==
kit.servo[3].angle = 0
sleep(1)
kit.servo[3].angle = 90
sleep(1)
kit.servo[3].angle = 180
sleep(1)


# == end ==  
kit.servo[0].angle = 90
kit.servo[1].angle = 0

kit.servo[3].angle = 0