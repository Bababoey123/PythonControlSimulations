import numpy as np

class Simulator:
    def __init__(self,model,X0):
        self.model=model
        self.X=X0
        return
    def step(self,u):
        self.X=self.model.Ad @ self.X + self.model.Bd @ u
        y=self.model.Cd @ self.X 
        return y, self.X
class ControlLoop:
    def __init__(self,config_file,model,controller,observer,simulator):
        self.config_file=config_file
        self.model=model
        self.controller=controller
        self.observer=observer
        self.simulator=simulator
        
        ## for continuous integration
        self.dt_plant=1e-3
        return
    def run_discrete_control_loop(self,X_0,Logger):
        ##for loop config
        N = int(self.config_file.T / self.config_file.dt) #nuber of array elements 
        u = np.array([[0.0]]) #initial control input 
        
        ## control loop
        for k in range(0,N):

            ## 1  simulate plant 
            y, X = self.simulator.step(u)

            ## 2 estimate state
            Xhat=self.observer.reconstruct(y,u)

            ## 3 calculate the control input 
            u=self.controller.compute(y)

            ## 4 log into arrays
            t=k*self.config_file.dt
            Logger.log(t,y,u,Xhat)
        return Logger
    def run_continuous_control_loop(self,X_0,Logger):
        N_substep=int(self.config_file.dt/self.dt_plant) ## number of substeps between each controller update
        self.simulator.X=X_0
        X=self.simulator.X.copy()
        t=0
        u = np.array([[0.0]]) #initial control input 
        while t<self.config_file.T:
            y=self.model.C @ X
            Xhat=self.observer.reconstruct(y,u)
            u=self.controller.compute(Xhat)
            
            for i in range (N_substep):
                x_dot=self.model.A @ X + self.model.B @ u
                ## simple forward euler 
                X+= self.dt_plant*x_dot
                
                ## update time 
                t+=self.dt_plant
                Logger.log(t,X[0,0],u,Xhat)
        return Logger