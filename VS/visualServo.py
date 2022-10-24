from tkinter.tix import ListNoteBook
from matrix import Matrix
from color_tracking import Tracker


class VisualServo:
    def __init__(self, c1, c2):
        self.tracker = Tracker(c1, c2)

    def initJacobian(self):
        pass

    def reachVisualGoals(self, n, THRESHOLD):
        cur, goal = Matrix(2,1), Matrix(2,1)
        cur.list2Matrix(self.tracker.point) # current (u,v) pos of end effector --> red dot 
        goal.list2Matrix(self.tracker.goal) # (u,v) position of the goal pos --> blue dot
        res = goal+cur*(-1)
        J = self.initJacobian()

        i = 0
        while (i <= n and res.vec_norm() > THRESHOLD):
            
            # Solve for motion (solve for joint movment)
            J_inv = J.TwoByTwoInverse()
            delta_x = J_inv*(res)

            # Move robot joints (move arm)

            # Read actual visual move (update cur)
            old = cur
            cur = cur.list2Matrix(self.tracker.point)
            delta_y = cur+old*(-1)

            # Update Jacobian 
            J = J+(delta_y+J*delta_x*(-1))*delta_x.transpose() / (delta_x.transpose()*delta_x)
            
            # 
            res = goal+cur*(-1)
