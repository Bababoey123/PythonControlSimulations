import numpy as np
import control as ct

class BallBeam:
    def __init__(self,H,dt):
        self.A=np.array([[0,1],[0,0]])
        self.B=np.array([[0],[H]])
        self.C=np.array([[1,0]])
        self.D=np.array([0])

        BBS_continuous=ct.StateSpace(self.A,self.B,self.C,self.D)
        BBS_discrete=BBS_continuous.sample(dt,'zoh')
        
        self.Ad=BBS_discrete.A
        self.Bd=BBS_discrete.B
        self.Cd=BBS_discrete.C
        self.Dd=BBS_discrete.D


        return