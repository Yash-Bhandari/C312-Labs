import math
from matrix import Matrix

# == lengths of joints [l1, l2] (meters) == 
l = Matrix(2,1)
l[0][0] = l1 = 1
l[1][0] = l2 = 1

# == joint angles (rad) == 
START_THETA = Matrix(2,1)
START_THETA[0][0] = math.pi/2
START_THETA[1][0] = math.pi/2

THETA = Matrix(2,1)
THETA[0][0] = math.pi/4
THETA[1][0] = math.pi/4

# == step size == 
h = 0.1 


def eval_robot(l, theta):
    """
    Computes the (x,y) postion of a the end effector given the joint angles 
    Args:   
        l (array 1x2): lenght of joint segments 
        theta (array 1x2): angle of joints
    """
    pos = Matrix(2,1)

    # x, y position of end effector  
    pos.setElement(0, 0, l[0][0]*math.cos(theta[0][0])+l[1][0]*math.cos(theta[0][0]+theta[1][0]))
    pos.setElement(1, 0, l[0][0]*math.sin(theta[0][0])+l[1][0]*math.sin(theta[0][0]+theta[1][0]))
    
    # Jacobian 
    J = Matrix(2,2)
    J.setElement(0, 0, -l[0][0]*math.sin(theta[0][0]) - l[1][0]*math.sin(theta[0][0]+theta[1][0])) #dx/dtheta1
    J.setElement(0, 1, -l[1][0]*math.sin(theta[0][0]+theta[1][0])) #dx/dtheta2
    J.setElement(1, 0, l[0][0]*math.cos(theta[0][0]) + l[1][0]*math.cos(theta[0][0]+theta[1][0])) #dy/dtheta1
    J.setElement(1, 1, l[1][0]*math.cos(theta[0][0]+theta[1][0])) #dy/dtheta2

    return pos, J


def inverse_kinematics(l, theta, pos, n, mode):
    """
    Computes the angles of the arm to move the end effector to pos.
    Implements Newton's and Broyden's method and an analytical solution
    Args:   
        l (array 1x2): lenght of joint segments (meters)
        theta (array 1x2): angle of joints (degrees)
        pos (array 1x2): desiered (x,y) postion of end effector
        n (int): max number of iterations on newton/broden
        mode (str): specifies the method 
    """
    THRESHOLD = 0.01
    new_pos, B = eval_robot(l, theta)
    res = new_pos+pos*(-1)
    i = 0

    if mode == 'newton': 
        jiggle = Matrix(2,1)
        jiggle[0][0] = 0.1
        jiggle[1][0] = 0.1
        while (i <= n and res.vec_norm() > THRESHOLD):
            new_pos, J = eval_robot(l, theta)
            if J.det() == 0:
                print("Jacobian is singular")
                theta = theta = theta + jiggle
                continue
            J_inv = J.TwoByTwoInverse()*(-1)
            s = J_inv*(new_pos+pos*(-1))
            theta = theta+s
            res = new_pos+pos*(-1)
            i += 1

    elif mode == 'broyden':
        while (i <= n and res.vec_norm() > THRESHOLD):
            f = new_pos + pos*(-1)
            s = f * B.TwoByTwoInverse()*(-1) # s = -B\f
            theta = theta + s
            new_pos, _ = eval_robot(l, theta)
            y = new_pos + pos*(-1) + f*(-1)
            #B = B + ((y-B*s')) / (s'*s)
            res = new_pos + pos*(-1)
            i+=1 

    else:  # analytical solution 
        dist = (l[0][0]**2+l[1][0]**2)**(1/2) # distance from origin to pos 
        alpha_1 = math.atan(pos[1][0]/pos[0][0])
        alpha_2 = math.acos((l[0][0]**2+dist**2-l[1][0]**2)/(2*l[0][0]*dist))
        alpha_3 = math.acos((l[0][0]**2+l[1][0]**2-dist**2)/(2*l[0][0]*l[1][0]))
        theta[0][0] = alpha_1+alpha_2
        theta[1][0] = alpha_3-180

    return theta[0][0], theta[1][0]

if __name__ == '__main__':
    pos, B = eval_robot(l, THETA)
    ret = inverse_kinematics(l, START_THETA, pos, 1, 'newton')
    print(ret, 'fin')
