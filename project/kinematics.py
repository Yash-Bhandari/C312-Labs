import numpy as np 
import copy
import config

class HomogeneousTransform():
    """maps cords from one frame into another"""
    def __init__(self) -> None:
        self.H = np.identity(4)

    def translate(self, V):
        """tranlate frame through vector V"""
        self.H[0:3, 3] = V
        return self.H

    def rotate(self, axis, theta):
        """rotate frame about axis, through theta"""
        if axis == 'X': 
            self.xRot(theta)
        elif axis == 'Y': 
            self.yRot(theta)
        elif axis == 'Z': 
            self.zRot(theta)

        return self.H

    def xRot(self, theta):
        """Rotate coordinate frame about x-axis"""
        self.H[0:3, 0:3] = np.array([[1, 0 ,0 ],
                                    [0, np.cos(theta), -1*np.sin(theta)],
                                    [0, np.sin(theta), np.cos(theta)]])

    def yRot(self, theta):
        """Rotate coordinate frame about y-axis"""
        self.H[0:3, 0:3] = np.array([[np.cos(theta), 0 , np.sin(theta)],
                                    [0, 1, 0],
                                    [-1*np.sin(theta), 0, np.cos(theta)]])

    def zRot(self, theta):
        """Rotate coordinate frame about z-axis"""
        self.H[0:3, 0:3] = np.array([[np.cos(theta), -1*np.sin(theta), 0],
                                     [np.sin(theta), np.cos(theta), 0],
                                     [0, 0, 1]])



class ForwardKinematics():
    def __init__(self, joint_lengths=config.JOINTS, origin=config.WORLD) -> None:
        # == quadrilateral sides == 
        self.a, self.b = 2.5, 9
        self.c, self.d = 3.275, 9.825
        self.joint_lengths = joint_lengths
        self.origin = origin


    def getPos(self, rotation, physical=False):
        """applies Homogeneous Transformations to solve for the (x,y,z) pos of the end effector"""
        if physical:
            rotation = self.physicalToLogicalAngles(rotation)
        translation = self.joint_lengths
        # -- translation from origin to base of joint 0 -- 
        Tb = HomogeneousTransform().translate(self.origin)

        # -- translation, rotation corresponding to joint 0 -- 
        R0 = HomogeneousTransform().rotate('Z', rotation[0]) 
        T0 = HomogeneousTransform().translate(translation[0]) 

        # -- translation, rotation corresponding to joint 1 -- 
        R1 = HomogeneousTransform().rotate('Y', rotation[1]) 
        T1 = HomogeneousTransform().translate(translation[1])

        # -- translation, rotation corresponding to joint 2 -- 
        R2 = HomogeneousTransform().rotate('Y', rotation[2]) 
        T2 = HomogeneousTransform().translate(translation[2]) 

        # -- translation, rotation corresponding to joint 3 --
        R3 = HomogeneousTransform().rotate('X', rotation[3]) 
        T3 = HomogeneousTransform().translate(translation[3]) 

        # -- translation, rotation corresponding to joint 4 --
        R4 = HomogeneousTransform().rotate('Y', rotation[4]) 
        T4 = HomogeneousTransform().translate(translation[4]) 

        # -- translation, rotation corresponding to joint 5 --
        R5 = HomogeneousTransform().rotate('X', rotation[5]) 
        T5 = HomogeneousTransform().translate(translation[5]) 

        H = Tb @ R0 @ T0 @ R1 @ T1 @ R2 @ T2 @ R3 @ T3 @ R4 @ T4 @ R5 @ T5 


        """Returns the (x,y,z) postion of the robot w.r.t the robots base"""
        result = H @ np.array([0,0,0,1]).T
        return result[:3]


    def logicalToPhysicalAngles(self, logical_angles):
        """
        input: 
            - Logical angles: the angles used in the serial linkage model
        output:
            - Physical angle: ture acuator angles used to drive the model
        """

        physical_angles = copy.deepcopy(logical_angles)

        physical_angles[1] *= -1 # I think that this works 
        physical_angles[4] *= -1 # I think that this works 

        beta = self.quadrilateralBeta_arbitrary(physical_angles[2])

        physical_angles[2] = beta + physical_angles[1]


        physical_angles = [self.logicalToPhysicalAxis(physical_angles[i], i) for i in range(6)]

        physical_angles[2] *= -1

        return physical_angles


    def physicalToLogicalAngles(self, physical_angles):
        """
        input:
            - Physical angle: ture acuator angles used to drive the model
        output: 
            - Logical angles: the angles used in the serial linkage model
        """
        logical_angles = copy.deepcopy(physical_angles)

        logical_angles = [self.PhysicalToLogicalAxis(physical_angles[i], i) for i in range(6)]

        beta = logical_angles[2] - logical_angles[1]

        gamma = self.quadrilateralGamma(beta)

        logical_angles[2] = gamma # nice! 
        logical_angles[1] *= -1
        logical_angles[4] *= -1

        return logical_angles


    def logicalToPhysicalAxis(self, theta, joint): 
        """aligns model axes (logical) with servo axes (physical)""" 
        if joint == 0:
            return theta + np.pi/2
        elif joint == 1:
            return theta + 0.4607669 # 26.4 deg
        elif joint == 2:
            return -2*np.pi + theta + 0.986111  # 56.5 deg
        elif joint == 3:
            return theta + 1.71042
        elif joint == 4:
            return theta + 1.274
        elif joint == 5:
            return theta + 1.48353
            
        return theta

    
    def PhysicalToLogicalAxis(self, theta, joint):
        """aligns servo axes (physical) with model axes (logical)""" 
        if joint == 0:
            return theta - np.pi/2
        elif joint == 1:
            return theta - 0.4607669 # 26.4 deg
        elif joint == 2:
            return 2*np.pi - theta - 0.986111  #(np.pi - theta) + (np.pi/2 - 0.986111) + np.pi/2  # 56.5 deg
        elif joint == 3:
            return theta - 1.71042
        elif joint == 4:
            return theta - 1.274
        elif joint == 5:
            return theta - 1.48353
            
        return theta

    def is_valid(self, physical_angles):
        logical = self.physicalToLogicalAngles(physical_angles)
        beta = self.quadrilateralBeta_arbitrary(logical[2])
        if beta < np.radians(5):
            return False
        return True

    def jointLimitsPhysical(self, physical_angles):
        """calulates the upper and lower bounds of the servos"""

        joints1 = physical_angles[1]
        bounds = np.ones((6, 2))

        delta = np.radians(56.5-24.6)
        little_wiggle = np.radians(38)
        big_wiggle = np.radians(85)

        upper = np.pi
        lower = 0
        if joints1+little_wiggle > (np.pi - delta):
            upper = np.pi - (joints1 - (np.pi-delta)) - little_wiggle

        if joints1-big_wiggle < (np.pi - delta):
            lower = (np.pi-delta) - joints1 + big_wiggle

        bounds[0] = [0, np.pi]
        bounds[1] = [0, np.pi]
        bounds[2] = [lower, upper]
        bounds[3] = [np.pi / 3, 2 / 3 * np.pi]
        bounds[4] = [0, np.pi]
        bounds[5] = [np.pi / 3, 2 / 3 * np.pi]
        return bounds


    def quadrilateralGamma(self, beta):
        """Solves for angle gamma in the quadrilatera given beta"""
        e = np.sqrt(self.a**2 + self.b**2 - 2*self.a*self.b*np.cos(beta))
        gamma_1 = np.arccos((self.b**2 + e**2 - self.a**2) / (2*self.b*e))
        gamma_2 = np.arccos((self.c**2 + e**2 - self.d**2) / (2*self.c*e))

        return gamma_1 + gamma_2


    def quadrilateralBeta(self, gamma):
        """
        Uses binary serch to find a beta value that gives the desired gamma value
        input:
            - gamma the gamma value that gives the desered elbow angle 
        output:
            - beta (elbow servo angle)
        """

        beta = np.linspace(np.pi/32, np.pi, num=200)

        low, high = 0, len(beta) - 1
        error = 0.015708 # 0.9 deg (servo precision)
    
        while low <= high:
            mid = (high + low) // 2

            gamma_aprox = self.quadrilateralGamma(beta[mid])
            res = abs(gamma_aprox-gamma)

            if gamma_aprox > gamma and res >= error:
                low = mid + 1
    
            elif gamma_aprox < gamma and res >= error:
                high = mid - 1
    
            elif res <= error:
                return beta[mid]

        raise ValueError('No solution')


    def quadrilateralBeta_arbitrary(self, gamma, error = 0.001):
        """Uses binary serch to find a beta value that gives the desired gamma value"""
        low, high = np.pi/32, np.pi

        while low <= high:
            mid = (high + low) / 2

            gamma_aprox = self.quadrilateralGamma(mid)
            res = abs(gamma_aprox-gamma)

            if gamma_aprox > gamma and res >= error:
                low = mid 
    
            elif gamma_aprox < gamma and res >= error:
                high = mid 
    
            elif res < error:
                return mid

        raise ValueError('No solution')


    # def quadrilateralCalc(self, sides):
    #     beta = self.theta[1] + self.theta[2] + 90 + self.angleCorrection[1] + self.angleCorrection[2]
    #     e = np.sqrt(self.a**2+self.b**2-2*self.a*self.b*np.cos(beta))
    #     gamma_1 = np.arccos((self.b**2+self.e*2-self.a**2) / (2*self.b*self.e))
    #     gamma_2 = np.arccos((self.d**2-self.c**2-self.e**2 / (2*self.c*self.e)))
    #     gamma = gamma_1 + gamma_2
    #     f = np.sqrt(self.b**2+self.c**2-2*self.b*self.c*np.cos(gamma))
    #     alpha = np.arccos((self.a**2+self.d**2-self.f**2)/(2*self.d*self.a))
    #     delta = 360 - alpha - gamma - beta 


class InverseKinematics():
    def __init__(self, pos) -> None:
        pass
         