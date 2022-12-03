import numpy as np 
import copy

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
    def __init__(self):
        # == quadrilateral sides == 
        self.a, self.b = 2.5, 9
        self.c, self.d = 3.275, 9.825


    def getPos(self, rotation, translation, origin):
        """applies Homogeneous Transformations to solve for the (x,y,z) pos of the end effector"""
        
        # -- translation from origin to base of joint 0 -- 
        Tb = HomogeneousTransform().translate(origin)

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
        R5 = HomogeneousTransform().rotate('Z', rotation[5]) 
        T5 = HomogeneousTransform().translate(translation[5]) 

        H = Tb @ R0 @ T0 @ R1 @ T1 @ R2 @ T2 @ R3 @ T3 @ R4 @ T4 #@ R5 @ T5 


        """Returns the (x,y,z) postion of the robot w.r.t the robots base"""
        return H @ np.array([0,0,0,1]).T
    

    def logicalToPhysicalAngles(self, logical_angles):
        """
        input: 
            - Logical angles: the angles used in the serial linkage model
        output:
            - Physical angle: ture acuator angles used to drive the model
        """
        physical_angles = logical_angles

        # == align elbow to shoulder frame == 
        physical_angles[2] += physical_angles[1]

        # == elbow correction (solve for betta given gamma) == 
        physical_angles[2] = self.quadrilateralBeta(np.pi/2 + logical_angles[2]) # beta w.r.t physical_angles[1]

        # == align angle to servo axis == 
        physical_angles = [self.logicalToPhysicalAxis(physical_angles[i], i) for i in range(6)]

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

        # == quadrilatera unique cases ==  
        # -- case - 1 -- 
        if (0 <= logical_angles[1] <= np.pi/2) and (np.pi <= logical_angles[2] <= 2*np.pi):
            alpha_1 = np.pi - logical_angles[1]
            alpha_2 = logical_angles[2] - np.pi
            alpha_3 = logical_angles[1] - np.pi/2 
            beta = alpha_1 + alpha_2

        # -- case - 2 -- 
        elif (np.pi/2 <= logical_angles[1] <= np.pi) and (np.pi <= logical_angles[2] <= 2*np.pi):
            alpha_1 = np.pi - logical_angles[1]
            alpha_2 = logical_angles[2] - np.pi
            alpha_3 = -1*(logical_angles[1] - np.pi/2)
            beta = alpha_1 + alpha_2

        # -- case - 3 -- 
        elif (0 <= logical_angles[1] <= np.pi/2) and (np.pi/2 <= logical_angles[2] <= np.pi):
            alpha_1 = np.pi/2 - logical_angles[1]
            alpha_2 = logical_angles[2] - np.pi/2
            alpha_3 = alpha_1
            beta = alpha_1 + alpha_2   

        # -- case - 4 -- 
        elif (np.pi/2 <= logical_angles[1] <= np.pi) and (np.pi/2 <= logical_angles[2] <= np.pi):
            alpha_1 = np.pi- logical_angles[1]
            alpha_2 = np.pi- logical_angles[2]
            alpha_3 = -1*(logical_angles[1] - np.pi/2)
            beta = alpha_1 - alpha_2   

        gamma = self.quadrilateralGamma(beta)

        logical_angles[2] = gamma # nice! 
        logical_angles[1] = -1*logical_angles[1] # I think that this works 

        return logical_angles


    def logicalToPhysicalAxis(self, theta, joint): 
        if joint == 1:
            return theta + 0.4607669 # 26.4 deg
        elif joint == 2:
            return theta + 0.986111  # 56.5 deg

        return theta

    
    def PhysicalToLogicalAxis(self, theta, joint):
        """aligns servo axes (physical) with model axes (logical)""" 
        if joint == 0:
            return theta - np.pi/2
        elif joint == 1:
            return theta - 0.4607669 # 26.4 deg
        elif joint == 2:
            return (np.pi - theta) + (np.pi/2 - 0.986111) + np.pi/2  # 56.5 deg
            
        return theta


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


    def quadrilateralBeta_arbitrary(self, beta, gamma):
        """Uses binary serch to find a beta value that gives the desired gamma value"""
        low, high = np.pi/32, np.pi
        error = 0.000000001
    
        while low <= high:
            mid = (high + low) / 2

            gamma_aprox = self.quadrilateralGamma(beta[mid])
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
         