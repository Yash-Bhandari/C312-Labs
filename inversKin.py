import math 

l = [1, 1] # lengths of joints [l1, l2] (meters)
h = 0.1 # step size 
THETA = [0, 0] # joint angles [theta1, theta2] (degrees)

def mat_sub(A,B):
    """Subtracts two matrices (A-B)"""
    if type(A[0]) == float:
        for i in range(len(A)):
            A[i] = A[i] - B[i]
    else:
        for i in range(len(A)):
            for j in range(len(A[0])):
                A[i][j] = A[i][j] - B[i][j]
    return A


def vec_norm(A):
    """Finds the norm of a vector A"""
    ret = [i**2 for i in A]
    return (sum(ret))**(1/2)


def eval_robot(l, theta):
    """
    Computes the (x,y) postion of a the end effector given the joint angles 
    Args:   
        l (array 1x2): lenght of joint segments 
        theta (array 1x2): angle of joints
    """
    pos = [0, 0]

    # x, y position of end effector  
    pos[0] = l[0]*math.cos(theta[0])+l[1]*math.cos(theta[0]+theta[1])
    pos[1] = l[0]*math.sin(theta[0])+l[1]*math.sin(theta[0]+theta[1])

    # Jacobian 
    J = [[0,0], [0,0]] # cols --> [x, y] rows --> [d_theta_1, d_theta_1]

    J[0][0] = -l[0]*math.sin(theta[0])-l[1]*math.sin(theta[0]+theta[1])
    J[0][1] = l[0]*math.cos(theta[0])+l[1]*math.cos(theta[0]+theta[1])

    J[1][0] = -l[1]*math.sin(theta[0]+theta[1]) 
    J[1][1] = l[1]*math.cos(theta[0]+theta[1])
    return pos, J


def inverse_kinematics(l, theta, pos, n, mode):
    """
    Computes the angles of the arm to mov the end effector to pos.
    Implements Newton's and Broyden's methos and an analytical solution
    Args:   
        l (array 1x2): lenght of joint segments (meters)
        theta (array 1x2): angle of joints (degrees)
        pos (array 1x2): desiered (x,y) postion of end effector
        n (int): max number of iterations on newton/broden
        mode (str): specifies the method 
    """
    threshold = 0.01
    new_pos, B = eval_robot(l, theta)
    res = mat_sub(new_pos, pos)
    i = 0

    print(new_pos)

    if mode == 'newton': 
        while (i <= n and vec_norm(res) > threshold):
            new_pos, J = eval_robot(l, theta)
            # s = -J\(new_pos-pos)
            # theta = theta + s
            res = new_pos - pos
            i += 1
    elif mode == 'broyden':
        while (i <= n and vec_norm(res) > threshold):
            f = mat_sub(new_pos, pos)
            # s = -B\f
            # theta = theta + s
            # new_pos, _ = eval_robot(l, theta)
            # y = new_pos-pos-f
            # B = B + ((y-B*s'))/(s'*s)
            res = new_pos - pos
            i+=1 
    else:  # analytical solution 
        dist = (l[0]**2+l[1]**2)**(1/2) # distance from origin to pos 
        alpha_1 = math.atan(pos[1]/pos[0])
        alpha_2 = math.acos((l[0]**2+dist**2-l[1]**2)/(2*l[0]*dist))
        alpha_3 = math.acos((l[0]**2+l[1]**2-dist**2)/(2*l[0]*l[1]))
        theta[0] = alpha_1+alpha_2
        theta[1] = alpha_3-180

    return theta

pos, B = eval_robot(l, THETA)
inverse_kinematics(l, THETA, pos, 1, 'newton')