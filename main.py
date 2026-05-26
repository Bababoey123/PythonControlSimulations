from Models.ballbeam import BallBeam
from Models import ballbeam_config
from Simulation.simulation import Simulator
from Control.observer import Observer
from Control.controller import StateFeedbackController

import control as ct
import numpy as np
import matplotlib.pyplot as plt 

model=BallBeam(ballbeam_config.H,ballbeam_config.dt)

## state feeddback gain
K = ct.place(model.Ad, model.Bd, [0.9, 0.95])
## observer feedback gain 
L = ct.place(model.Ad.T,model.Cd.T, [0.4, 0.5]).T

##setting up the controller
X_REF=np.array([[1.0], [0.0]])
controller=StateFeedbackController(K,X_REF)
##setting up the observer 
observer=Observer(model.Ad,model.Bd,model.Cd,L)

##plotting initialisations
N = int(ballbeam_config.T / ballbeam_config.dt) #nuber of array elements 
u = np.array([[0.0]]) #initial control input 
t_hist = []
y_hist = []
u_hist = []
xhat_hist = []
err_hist=[]

## creating the simulator
BBS_sim=Simulator(model)

## control loop
for k in range(0,N):

    ## 1  simulate plant 
    y, X = BBS_sim.step(u)

    ## 2 estimate state
    Xhat=observer.reconstruct(y,u)

    ## 3 calculate the control input 
    u=controller.compute(Xhat)

    ## 4 log into arrays
    t=k*ballbeam_config.dt
    t_hist.append(t)
    y_hist.append(y.item())
    u_hist.append(u.item())
    xhat_hist.append(Xhat.copy())
    err_hist.append(np.abs(y.item()-Xhat[0,0]))

## plotting
plt.figure()
plt.plot(t_hist, y_hist)

plt.grid(True)

plt.xlabel("time [s]")
plt.ylabel("position")

plt.figure()

plt.plot(t_hist, u_hist)

plt.grid(True)

plt.xlabel("time [s]")
plt.ylabel("control input")

plt.figure()
plt.plot(t_hist,err_hist)
plt.grid(True)
plt.xlabel("time [s]")
plt.ylabel("observer position error")

plt.show()
