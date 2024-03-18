import numpy as np
from scipy.optimize import minimize
import copy


class Car_Dynamics:
    def __init__(self, x_0, y_0, v_0, psi_0, length, dt):
        self.dt = dt            
        self.L = length          
        self.x = x_0
        self.y = y_0
        self.v = v_0
        self.psi = psi_0
        self.state = np.array([[self.x, self.y, self.v, self.psi]]).T

    def move(self, accelerate, delta): 
        x_dot = self.v*np.cos(self.psi)
        y_dot = self.v*np.sin(self.psi)
        v_dot = accelerate
        psi_dot = self.v*np.tan(delta)/self.L
        return np.array([[x_dot, y_dot, v_dot, psi_dot]]).T

    def update_state(self, state_dot):   
       
        self.state = self.state + self.dt*state_dot
        self.x = self.state[0,0]
        self.y = self.state[1,0]
        self.v = self.state[2,0]
        self.psi = self.state[3,0]

    
class MPC_Controller:
    def __init__(self):
        self.horiz = None
        self.R = np.diag([0.01, 0.01])                 
        self.Rd = np.diag([0.01, 1.0])                 
        self.Q = np.diag([1.0, 1.0])                   
        self.Qf = self.Q                               

    def mpc_cost(self, u_k, my_car, points):      
        mpc_car = copy.copy(my_car)
        u_k = u_k.reshape(self.horiz, 2).T
        z_k = np.zeros((2, self.horiz+1))
    
        desired_state = points.T
        cost = 0.0

        for i in range(self.horiz):
            state_dot = mpc_car.move(u_k[0,i], u_k[1,i])
            mpc_car.update_state(state_dot)
        
            z_k[:,i] = [mpc_car.x, mpc_car.y]
            cost += np.sum(self.R@(u_k[:,i]**2))
            cost += np.sum(self.Q@((desired_state[:,i]-z_k[:,i])**2))
            if i < (self.horiz-1):     
                cost += np.sum(self.Rd@((u_k[:,i+1] - u_k[:,i])**2))
        return cost

    def optimize(self, my_car, points):   
        self.horiz = points.shape[0]
        bnd = [(-5, 5),(np.deg2rad(-60), np.deg2rad(60))]*self.horiz
        result = minimize(self.mpc_cost, args=(my_car, points), x0 = np.zeros((2*self.horiz)), method='SLSQP', bounds = bnd)
        return result.x[0],  result.x[1]





