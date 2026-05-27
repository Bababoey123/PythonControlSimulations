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