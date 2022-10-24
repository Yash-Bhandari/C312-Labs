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

    def prettyMatrix (self, matrix):
        """separates rows by newline characters"""
        strings = []
        for row in matrix:
            strings.append(str(row))
        return '\n'.join(strings)  

    def list2Matrix(self, l):
        self.matrix = self.get_matrix(len(l), len(l[0]))
        for i in range(len(l)):
            for j in range(len(l[i])):
                self.matrix[i][j] = l[i][j]
        return self.matrix

    def __str__(self):
        """Method called when when print() or str() is invoked on an object"""
        return self.prettyMatrix(self.matrix)
    
    def __len__(self):
        """Method called when when len() is invoked on an object"""
        return len(self.matrix)

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
        res = Matrix(len(self.matrix), len(other[0]))
        for i in range(len(self.matrix)):
            for j in range(len(other[0])):
                for k in range(len(other)):
                    res[i][j] += self.matrix[i][k] * other[k][j]
        return res
    
    def __mul__(self, other):
        """Method called when when * is invoked between two Matrix objects, or between a matrix and a scalar"""
        if isinstance(other, Matrix):
            return self.multiply(other)

        res = Matrix(2, 2)
        for i in range(self.row_size):
            for j in range(self.col_size):
                res.setElement(i, j, other*self.matrix[i][j])

        return res 

    def add(self, other):
        """Add two matrices"""
        res = Matrix(len(self.matrix), len(self.matrix[0]))
        for i in range(len(self.matrix)):
            for j in range(len(self.matrix[0])):
                res[i][j] = self.matrix[i][j] + other[i][j]
        return res

    def __add__(self, other):
        """Method called when when + is invoked between two matrix objects"""
        return self.add(other)

    def TwoByTwoInverse(self):
        """Finds inverse of a 2x2 matrix"""
        scalar = 1/ self.det()
        res = Matrix(2,2)
        res.setElement(0, 0, self.matrix[1][1])
        res.setElement(0, 1, -1*self.matrix[0][1])
        res.setElement(1, 0, -1*self.matrix[1][0])
        res.setElement(1, 1, self.matrix[0][0])
        for i in range(self.row_size):
            for j in range(self.col_size):
                res.setElement(i, j, scalar*res[i][j])
        return res

    def det(self):
        return self.matrix[0][0]*self.matrix[1][1] - self.matrix[0][1]*self.matrix[1][0]

    def pseudoinverse(self):
        pass
    
    def vec_norm(self):
        """Finds the norm of a vector A"""
        ret = []
        for i in range(len(self.matrix)):
            ret.append(self.matrix[i][0]**2)
        return (sum(ret))**(1/2)

    def __repr__(self):
        return str(self)