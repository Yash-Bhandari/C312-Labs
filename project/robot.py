import numpy as np 
from time import *
import config
try:
    from adafruit_servokit import ServoKit
except ImportError:
    pass
from kinematics import ForwardKinematics, InverseKinematics
from inverse_kin import pose_for_location

class Robot():
    def __init__(self):
        self.kit = ServoKit(channels=16)

        # == joint measurements (cm) == 
        self.joints = config.JOINTS
        # joint v2 kinda sus

        # == set origin == 
        self.origin = config.WORLD

        # == servo joints ==  
        self.kit.servo[0].set_pulse_width_range(500, 2500) # Base
        self.kit.servo[1].set_pulse_width_range(500, 2500) # Shoulder 
        self.kit.servo[2].set_pulse_width_range(500, 2500) # Elbow - 1
        self.kit.servo[3].set_pulse_width_range(500, 2500) # Elbow - 2
        self.kit.servo[4].set_pulse_width_range(400, 2400) # Wrist - 1
        self.kit.servo[5].set_pulse_width_range(400, 2400) # Wrist - 2

        # == forward kinematics == 
        self.kin = ForwardKinematics(self.joints, self.origin)
        
        self.goToStartingPose()
        #self.move2pose([90, 140, 120, 90, 160, 180])
        print('Robot is ready!')
        sleep(1)

    def goToStartingPose(self):
        # == move to start postion == 
        self.physical_angles = [1.570, 2.443, 2.094, 1.71042, 1.8, 1.48353] # ture acuator angles used to drive the model 
        self.logical_angles = self.kin.physicalToLogicalAngles(self.physical_angles) # the angles used in the serial linkage model
        self.move2pose(self.logical_angles)
        sleep(0.5)

    def dipPaint(self):
        moves = [[160, 116, 132, 90, 75, 95], [157, 120, 170, 90, 120, 95], [157, 90, 180, 90, 120, 95]]
        for move in moves:
            angles = np.radians(np.array(move))
            self.move2pose(angles, physical=True)
        sleep(1)
        for move in reversed(moves):
            angles = np.radians(np.array(move))
            self.move2pose(angles, physical=True)
        self.goToStartingPose()


    def stopRobot(self):
        """Robot shutdown command"""
        self.physical_angles = [1.570, 2.443, 2.094, 1.570, 2.792, 3.141]
        self.move2pose(self.logical_angles)
        print('Shuting down robot')


    def move2pose(self, pose, physical=False):
        """physicaly moves the arm to specified joint angles at a constant speed
        args:
            - pose in logical angles (rad)        
        """
        if physical:
            pose = self.kin.physicalToLogicalAngles(pose)
        self.physical_angles = self.kin.logicalToPhysicalAngles(pose)
        self.logical_angles = pose
        
        pose = [np.degrees(self.physical_angles[i]) for i in range(6)]

        self.print_status()

        self.kit.servo[0].angle = pose[0]
        self.kit.servo[1].angle = pose[1]
        self.kit.servo[2].angle = pose[2]
        self.kit.servo[3].angle = pose[3]
        self.kit.servo[4].angle = pose[4]
        self.kit.servo[5].angle = pose[5]
        sleep(0.4)

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
        def format_angles(angles):
            degrees = [int(angle / np.pi * 180) for angle in angles]
            return degrees
        print('Logical Angles: ', format_angles(self.logical_angles))
        print('Physical Angles: ', format_angles(self.physical_angles))
        print('Location: ', self.kin.getPos(self.logical_angles))