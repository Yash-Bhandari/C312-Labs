#!/usr/bin/env python3
from matrix import Matrix
from VS.color_tracking import Tracker
from VS.server import Server, init_server
from time import sleep


class VisualServo:
    def __init__(self, c1, c2, server: Server):
        self.tracker = Tracker(c1, c2)
        sleep(3)
        self.server = server
        self.goal = self.get_goal()

    def initJacobian(self):

        J = Matrix(2,2)
        cur = self.avg_pos()

        # move joint 1
        theta1 = 5

        # self.arm.set_angles(self.x[0][0] + theta1, self.x[1][0])
        self.server.sendAngles(theta1, 0)

        old = cur 
        cur = self.avg_pos()

        J.setElement(0, 0, (cur[0][0]-old[0][0])/(theta1)) #du/dtheta1
        J.setElement(1, 0, (cur[1][0]-old[1][0])/(theta1)) #dv/dtheta1

        # move joint 2
        theta2 = 5

        old = cur 
        cur = self.get_point()

        self.server.sendAngles(0, theta2)
        
        J.setElement(0, 1, (cur[0][0]+old[0][0]*(-1))/(theta2)) #du/dtheta2
        J.setElement(1, 1, (cur[1][0]+old[1][0]*(-1))/(theta2)) #dv/dtheta2

        return J

    def get_point(self):
        return Matrix.from_array([[self.tracker.point[0,0]], [self.tracker.point[0,1]]])

    def get_goal(self):
        return Matrix.from_array([[self.tracker.goal[0,0]], [self.tracker.goal[0,0]]])

    def avg_pos(self):
        values = []
        for i in range(10):
            values.append(self.get_point())
            sleep(0.1)
        return sum(values, start=Matrix(2, 1)) * (1/len(values))

    def reachVisualGoals(self, n, THRESHOLD):
        cur, goal = Matrix(2,1), Matrix(2,1)
        cur = self.get_point()
        res = self.goal - cur


        print("bbbbbbbbbbbbbbbbb")
        J = self.initJacobian()

        i = 0
        while True:
            # Solve for motion 
            if J.det() == 0:
                J[0][0] += 0.01
                J[1][1] += 0.01
            J_inv = J.TwoByTwoInverse()
            delta_x = J_inv*(res)

            # Move robot joints (move arm)
            self.server.sendAngles(delta_x[0][0], delta_x[1][0])
            sleep(0.5)

            # Read actual visual move (update cur)
            old = cur
            cur = self.avg_pos()
            delta_y = cur-old

            # Update Jacobian 
            a = delta_y - (J*delta_x)
            print(a)
            b = a * delta_x.T
            c = b.scale(delta_x.vec_norm()**2)
            J = J + c
            
            res = goal+cur*(-1)


def main():
    server = init_server()
    VS = VisualServo('b', 'g', server)
    VS.reachVisualGoals(10, 1)


if __name__ == '__main__':
    main()
