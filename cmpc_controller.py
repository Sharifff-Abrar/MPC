import numpy as np
import cvxpy as cp

def calc_Jacobian(x, u, param):

    L_f = param["L_f"]
    L_r = param["L_r"]
    dt   = param["h"]

    psi = x[2]
    v   = x[3]
    delta = u[1]
    a   = u[0]

    # Jacobian of the system dynamics
    A = np.zeros((4, 4))
    B = np.zeros((4, 2))

    #############################################################################
    #                    TODO: Implement your code here                         #
    #############################################################################
    beta = np.arctan((L_r / (L_r + L_f)) * np.arctan(delta))


    A[0, 2] = -dt * v * np.sin(psi + beta)
    A[0, 3] = dt * np.cos(psi + beta)
    A[1, 2] = dt * v * np.cos(psi + beta)
    A[1, 3] = dt * np.sin(psi + beta)
    A[2, 3] = dt * np.arctan(delta) / (np.sqrt((L_r**2 * np.arctan(delta)**2) / (L_f + L_r)**2 + 1) * (L_f + L_r))
    A[0, 0] = A[1, 1] = A[2, 2] = A[3, 3] = 1

    # Jacobian of the system dynamics with respect to the control input (B matrix)
 
    B[0, 1] = -dt * L_r * v * np.sin(psi + beta) / ((delta**2 + 1) * (np.sqrt((L_r**2 * np.arctan(delta)**2) / (L_f + L_r)**2 + 1) * (L_f + L_r)))
    B[1, 1] = dt * L_r * v * np.cos(psi + beta) / ((delta**2 + 1) * (np.sqrt((L_r**2 * np.arctan(delta)**2) / (L_f + L_r)**2 + 1) * (L_f + L_r)))
    B[2, 1] = dt * v / ((delta**2 + 1) * (np.sqrt((L_r**2 * np.arctan(delta)**2) / (L_f + L_r)**2 + 1)**3 * (L_f + L_r)))
    B[3, 0] = dt


    return [A, B]

def LQR_Controller(x_bar, u_bar, x0, param):
    len_state = x_bar.shape[0]
    dim_state = x_bar.shape[1]
    dim_ctrl  = u_bar.shape[1]


      
    Q = np.eye(4) * 200
    R = np.eye(2) * 25
    P = np.eye(4)  * 200

    # Preallocate matrices
    A_list = []  
    B_list = []  
    delta_s_init = [x0 - x_bar[0, :]] 
    
    delta_s_var = cp.Variable((len_state, dim_state)) 
    delta_u_var = cp.Variable((len_state - 1, dim_ctrl))  
    
    cost = 0
    constraints = [delta_s_var[0] == delta_s_init[0]] 
    
    for k in range(len_state - 1):
        A_k, B_k = calc_Jacobian(x_bar[k, :], u_bar[k, :], param)  
        A_list.append(A_k)
        B_list.append(B_k)
        cost += cp.quad_form(delta_s_var[k], Q) + cp.quad_form(delta_u_var[k], R)  
        constraints += [
            delta_s_var[k+1] == A_list[k] @ delta_s_var[k] + B_list[k] @ delta_u_var[k]
        ]
    

    cost += cp.quad_form(delta_s_var[-1], P)
    
    

    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(verbose=False, max_iter=10000)
    u_act = delta_u_var.value[0] + u_bar[0, :]
    
    return u_act



def CMPC_Controller(x_bar, u_bar, x0, param):

    len_state = x_bar.shape[0]
    dim_state = x_bar.shape[1]
    dim_ctrl  = u_bar.shape[1]

      
    Q = np.eye(4) * 1
    R = np.eye(2) * 0.3
    P = np.eye(4)  * 80

    a_min, a_max = param["a_lim"]
    delta_min, delta_max = param["delta_lim"]

    # Preallocate matrices
    A_values = []  
    B_values = []  
    del_s_initial = [x0 - x_bar[0, :]] 
    
    delta_s_var = cp.Variable((len_state, dim_state)) 
    delta_u_var = cp.Variable((len_state - 1, dim_ctrl))  
    
    cost = 0
    constraints = []
    constraints.append (delta_s_var[0] == del_s_initial[0])
    
    for k in range(len_state - 1):
        A_k, B_k = calc_Jacobian(x_bar[k, :], u_bar[k, :], param)  
        A_values.append(A_k)
        B_values.append(B_k)
        cost += cp.quad_form(delta_s_var[k], Q) + cp.quad_form(delta_u_var[k], R)  
        constraints.append(delta_s_var[k+1] == A_values[k] @ delta_s_var[k] + B_values[k] @ delta_u_var[k])
        constraints.append(delta_u_var[k, 0] >= a_min-u_bar[k, 0])
        constraints.append(delta_u_var[k, 0] <= a_max - u_bar[k, 0])
        constraints.append(delta_u_var[k, 1] >= delta_min - u_bar[k, 1])
        constraints.append(delta_u_var[k, 1] <= delta_max - u_bar[k, 1])

    cost += cp.quad_form(delta_s_var[-1], P)
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(verbose=False, max_iter=10000)
    u_act = delta_u_var.value[0] + u_bar[0, :]
    
    return u_act



    