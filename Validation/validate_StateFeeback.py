import control as ct
import numpy as np
import matplotlib.pyplot as plt 
from Metrics_Plotting import Plotting

class Validate_StateFeedback:
    def __init__(self,model,K,ballbeam_config):
        self.model=model
        self.K=K
        self.y_ref=0
        self.t_ref=0
        Acl = model.Ad - model.Bd @ K
        self.sys_validate=ct.ss(Acl,model.Bd,model.Cd,model.Dd,ballbeam_config.dt)
        return
    def Validate(self,ballbeam_config,Log,X_0,Plotter):
        t_ref = np.arange(0, ballbeam_config.T, ballbeam_config.dt)
        t_ref, y_ref = ct.initial_response(self.sys_validate,T=t_ref,X0=X_0)
        Plotter.plotValidation(t_ref,y_ref,Log)

