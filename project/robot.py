import numpy as np 
from time import *
from adafruit_servokit import ServoKit
from kinematics import ForwardKinematics, InverseKinematics
from inverse_kin import pose_for_location

class Robot():
    def __init__(self):
        self.kit = ServoKit(channels=16)

        # == joint measurements (cm) == 
        self.joints = [[4.05, 0, 6.75], [0, 0, 9], [-0.85, 0, 2.165],
                       [15.8, 0, 0], [1.825, 0, 0.85], [1.55, 0, -6.75]]

        # joint v2 kinda sus

        # == set origin == 
        self.origin = [-8.35, 0, 2.6] 

        # == servo joints ==  
        self.kit.servo[0].set_pulse_width_range(500, 2500) # Base
        self.kit.servo[1].set_pulse_width_range(500, 2500) # Shoulder 
        self.kit.servo[2].set_pulse_width_range(500, 2500) # Elbow - 1
        self.kit.servo[3].set_pulse_width_range(500, 2500) # Elbow - 2
        self.kit.servo[4].set_pulse_width_range(400, 2400) # Wrist - 1
        self.kit.servo[5].set_pulse_width_range(400, 2400) # Wrist - 2

        # == forward kinematics == 
        self.kin = ForwardKinematics(self.joints, self.origin)
        
        # == move to start postion == 
        self.physical_angles = [1.570, 2.443, 2.094, 1.570, 2.792, 3.141] # ture acuator angles used to drive the model 
        self.logical_angles = self.kin.physicalToLogicalAngles(self.physical_angles) # the angles used in the serial linkage model

        self.move2pose(self.physical_angles)
        #self.move2pose([90, 140, 120, 90, 160, 180])
        print('Robot is ready!')
        sleep(1)


    def stopRobot(self):
        """Robot shutdown command"""
        self.physical_angles = [1.570, 2.443, 2.094, 1.570, 2.792, 3.141]
        #self.move2pose(self.physical_angles)
        self.move2pose(self.physical_angles)
        print('Shuting down robot')


    def move2pose(self, pose):
        """physicaly moves the arm to spesified joint angles at a constant speed
        args:
            - pose in logical angles (rad)        
        """

        self.physical_angles = self.kin.logicalToPhysicalAngles(pose)
        self.logical_angles = pose
        
        pose = [np.rad2deg(pose[i]) for i in range(6)]

        self.kit.servo[0].angle = pose[0]
        self.kit.servo[1].angle = pose[1]
        self.kit.servo[2].angle = pose[2]
        self.kit.servo[3].angle = pose[3]
        self.kit.servo[4].angle = pose[4]
        self.kit.servo[5].angle = pose[5]

    def move2location(self, location):
        """Move robot to a specified location"""
        new_pose = pose_for_location(self.kin, self.logical_angles, location)
        self.physical_angles = self.kin.logicalToPhysicalAngles(new_pose)
        self.move2pose(new_pose)

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

    def print_status(self):
        """Prints the current status of the robot"""
        print('Logical Angles: ', self.logical_angles)
        print('Physical Angles: ', self.physical_angles)
        print('Location: ', self.kin.getPos(self.logical_angles))