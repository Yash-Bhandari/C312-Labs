import numpy as np 

class HomogeneousTransform():
    """maps cords from one frame into another"""
    def __init__(self, theta, V) -> None:
        self.V = V
        self.theta = theta #values in range [-90, 90] 
        self.H = np.identity(4)
        self.angleCorrection = [0,0,0,0,0,0]  #Still need to measure 

    def transform(self, axisOfRot):
        """Create homogeneous transform matrix"""

        # Translate cordanite from s.t its origin is at V
        self.H[0:3, 3] = self.V

        # Change orntaion of frame by theta along spesified axis 
        if axisOfRot == 'X': self.xRot()
        elif axisOfRot == 'Y': self.yRot()
        else: self.zRot()

        return self.H

    def xRot(self):
        """Rotate coordinate frame about x-axis"""
        self.H[0:3, 0:3] = np.array([[1, 0 ,0 ],
                                    [0, np.cos(self.theta), -1*np.sin(self.theta)],
                                    [0, np.sin(self.theta), np.cos(self.theta)]])

    def yRot(self):
        """Rotate coordinate frame about y-axis"""
        self.H[0:3, 0:3] = np.array([[np.cos(self.theta), 0 , np.sin(self.theta)],
                                    [0, 1, 0],
                                    [-1*np.sin(self.theta), 0, np.cos(self.theta)]])

    def zRot(self):
        """Rotate coordinate frame about z-axis"""
        self.H[0:3, 0:3] = np.array([[np.cos(self.theta), -1*np.sin(self.theta), 0],
                                    [np.sin(self.theta), np.cos(self.theta), 0],
                                    [0, 0, 1]])


        
class ForwardKinematics():
    def __init__(self):
        # == quadrilateral sides == 
        self.a, self.b = 2, 9
        self.c, self.d = 3.3, 9.7


    def getPos(self, logical_angles, JointVec):
        """applies homogunuse matrix transforms to solve for the (x,y,z) pos of the end effector"""
        self.H0 = HomogeneousTransform(logical_angles[0], JointVec[0]).transform('Z')
        self.H1 = HomogeneousTransform(logical_angles[1], JointVec[1]).transform('Y')
        self.H2 = HomogeneousTransform(logical_angles[2], JointVec[2]).transform('Y')
        self.H3 = HomogeneousTransform(logical_angles[3], JointVec[3]).transform('X')
        self.H4 = HomogeneousTransform(logical_angles[4], JointVec[4]).transform('Y')
        self.H5 = HomogeneousTransform(logical_angles[5], JointVec[5]).transform('X')

        self.H = self.H0 @ self.H1 @ self.H2 @ self.H3 @ self.H4 @ self.H5

        """Returns the (x,y,z) postion of the robot w.r.t the robots base"""
        return self.H @ (np.array([0,0,0,1]).T)
    

    def logicalToPhysicalAngles(self, logical_angles):
        """
        input: 
            - Logical angles: the angles used in the serial linkage model
        output:
            - Physical angle: ture acuator angles used to drive the model
        """
        physical_angles = [logical_angles[i] for i in range(6)] 

        # == align elbow to shoulder frame == 
        physical_angles[2] += physical_angles[1]

        # == elbow correction (solve for betta given gamma) == 
        print(np.pi-logical_angles[2])
        physical_angles[2] = self.quadrilateralBeta(np.pi-logical_angles[2]) # beta w.r.t physical_angles[1]

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
        logical_angles = physical_angles

        logical_angles = [self.logicalToPhysicalAxis(physical_angles[i], i) for i in range(6)]

        # == align elbow to shoulder frame == 
        logical_angles[2] -= logical_angles[1]

        # == elbow correction (solve for gamma given beta)== 
        logical_angles[2] = np.pi - self.quadrilateralGamma(physical_angles[2])

        return logical_angles


    def logicalToPhysicalAxis(self, theta, joint): 
        return theta

    
    def PhysicalToLogicalAxis(self, theta, joint): 
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
        vars:
            - gamma the gamma value that gives the desered elbow angle 
        returns:
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
    #     e = np.sqrt(a**2+b**2-2*a*b*np.cos(beta))
    #     gamma_1 = np.arccos((b**2+e*2-a**2) / (2*b*e))
    #     gamma_2 = np.arccos((d**2-c**2-e**2 / (2*c*e)))
    #     gamma = gamma_1 + gamma_2
    #     f = np.sqrt(b**2+c**2-2*b*c*np.cos(gamma))
    #     alpha = np.arccos((a**2+d**2-f**2)/(2*d*a))
    #     delta = 360 - alpha - gamma - beta 


class InverseKinematics():
    def __init__(self, pos) -> None:
        pass
         