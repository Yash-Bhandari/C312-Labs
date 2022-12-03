import numpy as np

a, b, c, d = 2, 9, 3.3, 9.7 


beta = np.radians(105)

e = np.sqrt(a**2 + b**2 - 2*a*b*np.cos(beta))

gamma_1 = np.arccos((b**2 + e**2 - a**2) / (2*b*e))
gamma_2 = np.arccos((c**2 + e**2 -d**2) / (2*c*e))

gamma = gamma_1 + gamma_2

f = np.sqrt(b**2 + c**2 - 2*b*c*np.cos(gamma))

alpha = np.arccos((a**2 + d**2 - f**2)/(2*d*a))

delta = 360 - alpha - gamma - beta 


# gamma = 94.562 @ beta = 100
# gamma = 91.485 @ beta = 105

beta = np.linspace(np.pi/32, np.pi, num=200)

def foo(beta):
    e = np.sqrt(a**2 + b**2 - 2*a*b*np.cos(beta))
    gamma_1 = np.arccos((b**2 + e**2 - a**2) / (2*b*e))
    gamma_2 = np.arccos((c**2 + e**2 - d**2) / (2*c*e))

    return gamma_1 + gamma_2


def elbowMove_bs(beta, gamma):
    low = 0
    high = len(beta) - 1
    mid = 0
    error = 0.015708
 
    while low <= high:
 
        mid = (high + low) // 2

        # print(mid, beta[mid])

        val = foo(beta[mid])
        res = abs(val-gamma)

        # print(np.degrees(foo(beta[mid])), np.degrees(gamma), np.degrees(res))
        # print((val < gamma and res >= error), (val > gamma and res >= error))

        if val > gamma and res >= error:
            low = mid + 1
 
        elif val < gamma and res >= error:
            high = mid - 1
 
        elif res <= error:
            return beta[mid]

    return -1
 

def elbowMove_bs_arbitrary(beta, gamma):
    low = np.pi/32
    high = np.pi
    error = 0.000000001
 
    while low <= high:
 
        mid = (high + low) / 2
        print(mid)

        # print(mid, beta[mid])

        val = foo(mid)
        res = abs(val-gamma)

        # print(np.degrees(foo(beta[mid])), np.degrees(gamma), np.degrees(res))
        # print((val < gamma and res >= error), (val > gamma and res >= error))

        if val > gamma and res >= error:
            low = mid 
 
        elif val < gamma and res >= error:
            high = mid 
 
        elif res < error:
            return mid

    return -1

print(binarySearchElbowMove2(beta, np.radians(91.485)))