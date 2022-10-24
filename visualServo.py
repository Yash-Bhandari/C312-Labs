from matrix import Matrix
from VS.color_tracking import Tracker
from driver import RoboticArm


class VisualServo:
    def __init__(self, c1, c2):
        self.tracker = Tracker(c1, c2)
        self.arm = RoboticArm()
        self.x = Matrix(2,1) # (u, v)

    def initJacobian(self):
        # read joint angels 
        self.x.setElement(0, 0, self.joint1.position) 
        self.x.setElement(1, 0, self.joint2.position) 

        cur, J = Matrix(2,1), Matrix(2,2)
        cur.list2Matrix(self.tracker.point)

        # move joint 1
        theta1 = 5

        self.arm.set_angles(self.x[0][0] + theta1, self.x[1][0])

        old = cur 
        cur.list2Matrix(self.tracker.point)

        J.setElement(0, 0, (cur[0][0]+old[0][0]*(-1))/(theta1)) #du/dtheta1
        J.setElement(1, 0, (cur[1][0]+old[1][0]*(-1))/(theta1)) #dv/dtheta1

        # move joint 2
        theta2 = 5

        old = cur 
        cur.list2Matrix(self.tracker.point)

        self.arm.set_angles(self.x[0][0] + theta1, self.x[1][0]+theta2)
        
        J.setElement(0, 1, (cur[0][0]+old[0][0]*(-1))/(theta2)) #du/dtheta2
        J.setElement(1, 1, (cur[1][0]+old[1][0]*(-1))/(theta2)) #dv/dtheta2

        return J


    def reachVisualGoals(self, n, THRESHOLD):
        # read joint angels 
        self.x.setElement(0, 0, self.joint1.position) 
        self.x.setElement(1, 0, self.joint2.position) 

        cur, goal = Matrix(2,1), Matrix(2,1)
        cur.list2Matrix(self.tracker.point) # current (u,v) pos of end effector --> red dot 
        goal.list2Matrix(self.tracker.goal) # (u,v) position of the goal pos --> blue dot
        res = goal+cur*(-1)

        J = self.initJacobian()

        i = 0
        while (i <= n and res.vec_norm() > THRESHOLD):
            # Solve for motion 
            J_inv = J.TwoByTwoInverse()
            delta_x = J_inv*(res)

            # Move robot joints (move arm)
            self.arm.set_angles(self.x[0][0]+delta_x[0][0], self.x[1][0]+delta_x[1][0])
            self.x = self.x+delta_x

            # Read actual visual move (update cur)
            old = cur
            cur.list2Matrix(self.tracker.point)
            delta_y = cur+old*(-1)

            # Update Jacobian 
            J = J+(delta_y+J*delta_x*(-1))*delta_x.transpose() / (delta_x.transpose()*delta_x)
            
            res = goal+cur*(-1)