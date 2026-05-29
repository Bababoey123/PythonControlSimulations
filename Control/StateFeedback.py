import numpy as np

class StateFeedbackController:
    def __init__(self,K,X_REF,Max_actuation):
        self.K=K
        self.X_REF=X_REF
        self.Max_actuation=Max_actuation
        return
      
    def compute(self,X):
        u=-self.K@(X-self.X_REF)  
        u = np.clip(u, -self.Max_actuation, self.Max_actuation)
        return u
