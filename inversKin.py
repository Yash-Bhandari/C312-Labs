import math 

l = [1, 1] # lengths of joints [l1, l2] (meters)
h = 0.1 # step size 
THETA = [0, 0] # joint angles [theta1, theta2] (degrees)

class Matrix:
    """Provides matrix operations for inverse kinematics"""
    def __init__(self, n, m):
        """Initialize nxm matrix with zeros"""
        self.row_size = n
        self.col_size = m 
        self.matrix = self.get_matrix(n, m)

    def get_matrix(self, n, m):
        """Generate nxm matrix"""
        matrix = [[0 for j in range(m)] for i in range(n)]
        return matrix

    def get_readable_matrix_string(self, matrix):
        """separates rows by newline characters"""
        strings = []
        for row in matrix:
            strings.append(str(row))
        return '\n'.join(strings)  

    def __str__(self):
        """Method called when when print() or str() is invoked on an object"""
        return self.get_readable_matrix_string(self.matrix)
    
    def __len__(self):
        """Method called when when len() is invoked on an object"""
        return len(self.matrix[0])

    def __getitem__(self, i):
        return self.matrix[i]

    def getElement(self, i, j):
        """Returns matrix element in row i column j"""
        return self.matrix[i][j]
    
    def setElement(self, i, j, element):
        """Sets matrix elements in row i column j with value = element"""
        self.matrix[i][j] = element
    
    def transpose(self, matrix):
        """Returns the transpose of the matrix"""
        return [list(i) for i in zip(*matrix)]
    
    def multiply(self, other):
        """Implementation of matrix multiply"""
        res = [[0 for j in range(len(other[0]))] for i in range(len(self.matrix))]
        for i in range(len(self.matrix)):
            for j in range(len(other[0])):
                for k in range(len(other)):
                    res[i][j] += self.matrix[i][k] * other[k][j]
        return res
    
    def add(self, other):
        """Add two matrices"""
        res = [[0 for j in range(len(self.matrix))] for i in range(len(self.matrix[0]))]
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                res[i][j] = self.matrix[i][j] + other[i][j]
        return res


    def TwoByTwoInverse(self):
        scalar = 1/(self.getElement(0,0)*self.getElement(1,1)-self.getElement(0,1)*self.getElement(1,0))
        return self.multiply(scalar)

    def pseudoinverse(self):
        pass
    
    def __mul__(self, other):
        """Method called when when * is invoked between two Matrix objects, or between a matrix and a scalar"""
        if isinstance(other, Matrix):
            return self.multiply(other)
        # scalar multiplcation 
        return [[num*other for num in row] for row in self.matrix]

    def __add__(self, other):
        """Method called when when + is invoked between two matrix objects"""
        return self.add(other)
        


m1 = Matrix(2, 2)
m2 = Matrix(2, 2)
m2.setElement(0,0,5)
m2.setElement(1,1,5)
m1.setElement(0,0,2)
m1.setElement(1,1,3)

m3 = m1*m2
print(m3)
m4 = m1.TwoByTwoInverse()
print(m4)


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
    THRESHOLD = 0.01
    new_pos, B = eval_robot(l, theta)
    res = mat_sub(new_pos, pos)
    i = 0


    if mode == 'newton': 
        while (i <= n and vec_norm(res) > THRESHOLD):
            new_pos, J = eval_robot(l, theta)
            # s = -J\(new_pos-pos)
            # theta = theta + s
            res = new_pos - pos
            i += 1
    elif mode == 'broyden':
        while (i <= n and vec_norm(res) > THRESHOLD):
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