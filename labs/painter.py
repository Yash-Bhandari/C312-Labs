#!/usr/bin/env python3
import sys
from time import sleep
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B  # type: ignore

motor = LargeMotor(OUTPUT_A)
motor.reset()
while True:
	motor.on_to_position(10, 10, brake=True, block=True)
	sleep(2)
	motor.on_to_position(10, 30, brake=True, block=True)
	sleep(2)
	motor.on_to_position(10, 0, brake=True, block=True)
	sleep(2)
	motor.on_to_position(10, 50, brake=True, block=True)