#!/usr/bin/env python3
from matrix import Matrix
from VS.color_tracking import Tracker
from VS.server import Server, init_server
from time import sleep


class VisualServo:
    def __init__(self, tracker: Tracker, server: Server):
        sleep(3)
        self.tracker = tracker
        self.server = server
        self.goal = self.get_goal()
        print("Goal:", self.goal)
        while self.goal[0][0] == 0:
            print("Goal:", self.goal)
            self.goal = self.get_goal()
            sleep(0.5)

    def initJacobian(self):

        J = Matrix(2,2)
        cur = self.avg_pos()

        # move joint 1
        theta1 = 8

        # self.arm.set_angles(self.x[0][0] + theta1, self.x[1][0])
        self.server.sendAngles(theta1, 0)

        old = cur 
        cur = self.avg_pos()
        print('old:', old)
        print('cur:', cur)
        J.setElement(0, 0, (cur[0][0]-old[0][0])/(theta1)) #du/dtheta1
        J.setElement(1, 0, (cur[1][0]-old[1][0])/(theta1)) #dv/dtheta1

        # move joint 2
        self.server.sendAngles(-theta1, 0)
        theta2 = 8

        old = self.avg_pos()
        self.server.sendAngles(0, theta2)
        cur = self.avg_pos()
        J.setElement(0, 1, (cur[0][0]-old[0][0])/(theta2)) #du/dtheta2
        J.setElement(1, 1, (cur[1][0]-old[1][0])/(theta2)) #dv/dtheta2

        self.server.sendAngles(0, -theta2)
        return J

    def get_point(self):
        return Matrix.from_array([[self.tracker.point[0,0]], [self.tracker.point[0,1]]])

    def get_goal(self):
        return Matrix.from_array([[self.tracker.goal[0,0]], [self.tracker.goal[0,0]]])

    def avg_pos(self):
        values = []
        for i in range(20):
            values.append(self.get_point())
            sleep(0.1)
        return sum(values, start=Matrix(2, 1)) * (1/len(values))

    def reachVisualGoals(self, n, THRESHOLD):


        J = self.initJacobian()
        cur = self.avg_pos()

        res = self.goal - cur
        i = 0
        while True:
            # Solve for motion 
            if J.det() == 0:
                print("Jacobian is singular")
                J[0][0] += 0.01
                J[1][1] += 0.01
            print("Jacobian")
            print(J)
            J_inv = J.TwoByTwoInverse()
            delta_x = J_inv*(res)

            # Move robot joints (move arm)
            delta_x = delta_x.normalize().scale(10)
            print("Delta x")
            print(delta_x)
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


def main():
    tracker = Tracker('b', 'g')
    server = init_server()
    VS = VisualServo(tracker, server)
    VS.reachVisualGoals(10, 1)


if __name__ == '__main__':
    main()
