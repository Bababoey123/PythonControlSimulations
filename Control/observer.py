import numpy as np

class Observer:
    def __init__(self,Ad,Bd,Cd,L) :

        self.Ad=Ad
        self.Bd=Bd
        self.Cd=Cd
        self.L=L
        self.Xhat=np.array([[0.5], [0.0]])
        return
    def reconstruct(self,y,u):
        ## implementation of the Ludenberg Observer, in discrete time 
        ## x_hat[n+1]=(A-LC)x_hat[n]+B*u+L*y

        Xhat_n1 = ( self.Ad @ self.Xhat + self.Bd @ u + self.L @ (y - self.Cd @ self.Xhat))
        self.Xhat=Xhat_n1
        return Xhat_n1
        