import numpy as np

class SimLog:
    def __init__(self):
        self.t_hist = []
        self.y_hist = []
        self.u_hist = []
        self.xhat_hist = []
        self.err_hist=[]
        return
    def log(self,t,y,u,Xhat):
        self.t_hist.append(t)
        self.y_hist.append(y.item())
        self.u_hist.append(u.item())
        self.xhat_hist.append(Xhat.copy())
        self.err_hist.append(np.abs(y.item()-Xhat[0,0]))

        return