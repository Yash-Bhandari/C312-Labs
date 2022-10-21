from matrix import Matrix
from VS.color_tracking import Tracker


class VisualServo:
    def __init__(self):
        pass

    def reachVisualGoals(self, n, THRESHOLD):
        cur = 0  #current u,v pos of end effector 
        res = 0  #u,v position of the end pos
        i = 0

        while (i <= n and res.vec_norm() > THRESHOLD):
            #Solve for motion:
            #Move robot joints:
            #Read actual visual move
            #Update Jacobian: 
            pass
