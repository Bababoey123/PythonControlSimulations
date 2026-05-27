import numpy as np
import matplotlib.pyplot as plt 

class Plotting:
    def __init__(self):
        return
    def plotAll(self,Log):
        ## plotting
        plt.figure()
        plt.plot(Log.t_hist, Log.y_hist)

        plt.grid(True)

        plt.xlabel("time [s]")
        plt.ylabel("position")

        plt.figure()

        plt.plot(Log.t_hist, Log.u_hist)

        plt.grid(True)

        plt.xlabel("time [s]")
        plt.ylabel("control input")

        plt.figure()
        plt.plot(Log.t_hist,Log.err_hist)
        plt.grid(True)
        plt.xlabel("time [s]")
        plt.ylabel("observer position error")
        plt.show()
        
        return
    def plotValidation(self,t_ref,y_ref,Log):
        plt.figure()

        plt.plot(Log.t_hist,Log.y_hist,
         label="Custom simulation")

        plt.plot(t_ref,y_ref,'--',label="Control toolbox")

        plt.legend()

        plt.grid(True)
        plt.show()
        
        return
    