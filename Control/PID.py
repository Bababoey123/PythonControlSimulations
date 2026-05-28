import numpy as np
class PID_Controller:
    def __init__(self,kp,ki,kd,Max_actuation,dt):
        self.kp=kp
        self.ki=ki
        self.kd=kd

        self.integral=0

        self.dt=dt

        self.prev_error=0
        self.reference=0

        self.Max_actuation=Max_actuation

        return
    def setReference(self,r):
        self.reference=r
        return
    def compute(self,y):
        #error 
        error=self.reference-y.item()

        #integral 
        self.integral+=error*self.dt

        #derivative
        derivative=(error-self.prev_error)/self.dt

        #PID control output 
        u=self.kp*error+self.ki*self.integral+self.kd*derivative
        u = np.clip(u, -self.Max_actuation, self.Max_actuation)
        #update error
        self.prev_error=error

        # return as column vector
        return np.array([[u]])