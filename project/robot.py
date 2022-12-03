import numpy as np 
from time import *
from adafruit_servokit import ServoKit
from kinematics import ForwardKinematics, InverseKinematics

class Robot():
    def __init__(self):
        self.kit = ServoKit(channels=16)

        # == joint measurements (cm) == 
        self.joints = [[4,0,4.8],[0,0,9],[-0.5,0,2.04],
                       [16.68,0,0],[1.3,0,0.6],[1.6,0,6.8]]

        # == servo joints ==  
        self.kit.servo[0].set_pulse_width_range(500, 2500) # Base
        self.kit.servo[1].set_pulse_width_range(500, 2500) # Shoulder 
        self.kit.servo[2].set_pulse_width_range(500, 2500) # Elbow - 1
        self.kit.servo[3].set_pulse_width_range(500, 2500) # Elbow - 2
        self.kit.servo[4].set_pulse_width_range(400, 2400) # Wrist - 1
        self.kit.servo[5].set_pulse_width_range(400, 2400) # Wrist - 2

        # == forward kinematics == 
        self.kin = ForwardKinematics()
        
        # == move to start postion == 
        self.physical_angles = [1.570, 2.443, 2.094, 1.570, 2.792, 3.141] # ture acuator angles used to drive the model 
        self.logical_angles = self.kin.physicalToLogicalAngles(self.physical_angles) # the angles used in the serial linkage model

        self.move2pose(self.physical_angles)
        print('Robot is ready!')
        sleep(2)


    def stopRobot(self):
        """Robot shutdown command"""
        self.physical_angles = [1.570, 2.443, 2.094, 1.570, 2.792, 3.141]
        self.move2pose(self.physical_angles)
        print('Shuting down robot')


    def move2pose(self, pose):
        """physicaly moves the arm to spesified joint angles at a constant speed
        args:
            - pose in logical angles (rad)        
        """

        # self.physical_angles = self.kin.logicalToPhysicalAngles(pose)
        # self.logical_angles = pose

        pose = [np.degrees(self.physical_angles[i]) for i in range(6)]

        self.kit.servo[0].angle = pose[0]
        self.kit.servo[1].angle = pose[1]
        self.kit.servo[2].angle = pose[2]
        self.kit.servo[3].angle = pose[3]
        self.kit.servo[4].angle = pose[4]
        self.kit.servo[5].angle = pose[5]


    def getCurBrush(self):    
        """Returns to brush that is currently bing used"""
        pass


    def dipBrush(self, color):
        """Move robot from to current pose to paint container"""
        pass


    def forwardKin(self, pose):
        """Given arm pose find {x,y,z} cord of end effector"""
        pass


    def inversKin(self, cord):
        """G{x,y,z} cord of end effector find the joint angles needed"""
        pass