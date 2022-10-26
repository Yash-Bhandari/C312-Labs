#!/usr/bin/env python3
from matrix import Matrix
from VS.color_tracking import Tracker
from VS.server import Server, init_server
from time import sleep


class VisualServo:
    """Implements methods for achieving visual goals"""

    def __init__(self, tracker: Tracker, server: Server):
        self.tracker = tracker 
        self.server = server 
        sleep(3)
        self.goal = self.avg_goal() 

        print("Goal:", self.goal)
        # while self.goal[0][0] == 0: # Check to ensure goal is set properly
        #     print("Goal:", self.goal)
        #     self.goal = self.get_goal()
        #     sleep(0.5)

    def initJacobian(self): 
        """Initialize the jacobian for some local region"""
        J, cur = Matrix(2,2), self.avg_pos()
        theta1 = theta2 = 15

        # == move joint 1 == 
        old = cur 
        self.server.sendAngles(theta1, 0) # preforme delta theta1
        cur = self.avg_pos()
        J.setElement(0, 0, (cur[0][0]-old[0][0])/(theta1)) #du/dtheta1
        J.setElement(1, 0, (cur[1][0]-old[1][0])/(theta1)) #dv/dtheta1

        sleep(0.5)
        self.server.sendAngles(-theta1, 0) # undo delta theta1
        sleep(0.5)

        # == move joint 2 == 
        old = self.avg_pos()
        self.server.sendAngles(0, theta2) # preforme delta theta2
        cur = self.avg_pos()
        J.setElement(0, 1, (cur[0][0]-old[0][0])/(theta2)) #du/dtheta2
        J.setElement(1, 1, (cur[1][0]-old[1][0])/(theta2)) #dv/dtheta2

        sleep(0.5)
        self.server.sendAngles(0, -theta2) # undo delta theta2
        sleep(0.5)

        return J

    def get_point(self):
        return Matrix.from_array([[self.tracker.point[0,0]], [self.tracker.point[0,1]]])

    def get_goal(self):
        return Matrix.from_array([[self.tracker.goal[0,0]], [self.tracker.goal[0,1]]])

    def avg_pos(self):
        values = []
        for i in range(20):
            values.append(self.get_point())
            sleep(0.1)
        return sum(values, start=Matrix(2, 1)) * (1/len(values))

    def avg_goal(self):
        values = []
        for i in range(20):
            values.append(self.get_goal())
            sleep(0.1)
        return sum(values, start=Matrix(2, 1)) * (1/len(values))

    def SubdividePath(self, size=20):
        goal, cur, path = self.get_goal(), self.avg_pos(), []
        error = goal - cur

        slope = (goal[1][0]-cur[1][0])/(goal[0][0]-cur[0][0]) 
        perpSlope = -1/(slope)
        PathLength = error.vec_norm() # pixel lenght of path 

        for i in range(len(PathLength%20)):
            pass

        return path 

    def Plot(self):
        pass

    def UncalibratedVisualServoing(self, THRESHOLD = 7):
        """
        Uses Newtons method and a byroden update to move end effector to goal pos 
        within a margin of threshold.
            ARGS: 
                THRESHOLD (int): The pixel distance we need to be within
        """

        J = self.initJacobian()
        cur = self.avg_pos()

        res = self.goal - cur
        i = 0
        while res.vec_norm() > THRESHOLD:
            # Solve for motion 
            if J.det() == 0:
                print("Jacobian is singular")
                J[0][0] += 0.01
                J[1][1] += 0.01
            # print("Jacobian")
            # print(J)
            J_inv = J.TwoByTwoInverse()
            delta_x = J_inv*(res)

            # Move robot joints (move arm)
            error = res.vec_norm()
            step_size = 10
            if error < 30:
                step_size = 5
            if error < 15:
                step_size = 2
            delta_x = delta_x.normalize().scale(step_size)
            # print("Delta x")
            # print(delta_x)
            # breakpoint()
            self.server.sendAngles(delta_x[0][0], delta_x[1][0])
            sleep(0.5)

            # Read actual visual move (update cur)
            old = cur
            cur = self.avg_pos()
            delta_y = cur-old


            # Update Jacobian 
            a = delta_y - (J*delta_x)
            b = a * delta_x.T
            c = b.scale(1 / delta_x.vec_norm()**2)
            J = J + c
            
            res = self.goal-cur
            print('Distance from goal:', res.vec_norm())
        breakpoint()
        print("Done baby", res, res.vec_norm())



def main():
    tracker = Tracker('b', 'g')
    server = init_server()
    VS = VisualServo(tracker, server)
    VS.UncalibratedVisualServoing()


if __name__ == '__main__':
    main()
