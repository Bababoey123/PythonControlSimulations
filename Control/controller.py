import numpy as np
import time

class StateFeedbackController:
    def __init__(self,K,X_REF):
        self.K=K
        self.X_REF=X_REF
        return
      
    def compute(self,Xhat):
        u=-self.K@(Xhat-self.X_REF)  
        return u