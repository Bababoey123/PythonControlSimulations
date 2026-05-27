import numpy as np

class StateFeedbackController_Observer:
    def __init__(self,K,X_REF,observer):
        self.K=K
        self.X_REF=X_REF
        self.observer=observer

        self.u_prev=np.zeros((1,1))
        return
      
    def compute(self,y):
        # the observation is being done IN the feedback controller to guarantee that simulantion only sees y and u 
        
        u=-self.K@(self.observer.Xhat-self.X_REF)  
        Xhat=self.observer.reconstruct(y,self.u_prev)
        #update the previous input 
        self.u_prev=u
        return u
class StateFeedbackController:
    def __init__(self,K,X_REF):
        self.K=K
        self.X_REF=X_REF
        return
      
    def compute(self,X):
        u=-self.K@(X-self.X_REF)  
        return u
