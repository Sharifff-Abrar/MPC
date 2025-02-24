import numpy as np

def ACC_Controller(t, x, param):
    vd = param["vd"]
    v0 = param["v01"]
    m = param["m"]
    Cag = param["Cag"]
    Cdg = param["Cdg"]

    # cost function and constraints for the QP
    P = np.zeros((2,2))
    q = np.zeros([2, 1])
    A = np.zeros([5, 2])
    b = np.zeros([5])

    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################

    # set the parameters
    D = x[0]
    v = x[1]


    lam = 4e+5
    alpha = 4e-1
    w = 4e-0

    B = (D - ( 0.5*( (v0-v) * (v0-v)  / Cdg)  ) - (1.8*v))

    # construct the cost function
    P = np.array([[2,0],[0,2*w]])
    q = np.array([[0],[0]])
    
    # construct the constraints
    h = (v-vd)*(v-vd)/2

    A = np.array([
        [(v-vd)/m ,-1],
        [(1/m)*(1.8 + ((v-v0)/(Cdg))), 0],
        [1,0],
        [-1,0],
        [0,-1]
    ])
    
    b = np.array([
        -lam*h,
        v0 - v + alpha*B,
        Cag*m,
        Cdg*m,
        0
    ])

    #############################################################################
    #                            END OF YOUR CODE                               #
    #############################################################################
    
    return A, b, P, q