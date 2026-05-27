from Models.ballbeam import BallBeam
from Models import ballbeam_config
from Simulation.simulation import Simulator
from Control.observer import Observer
from Control.StateFeedback import StateFeedbackController
from Control.PID import PID_Controller
from Metrics_Plotting.SimLog import SimLog
from Metrics_Plotting.Plotting import Plotting
from Validation.validate_StateFeeback import Validate_StateFeedback



import control as ct
import numpy as np
import matplotlib.pyplot as plt 

model=BallBeam(ballbeam_config.H,ballbeam_config.dt)

## state feeddback gain
K = ct.place(model.Ad, model.Bd, [0.9, 0.95])
## observer feedback gain 
L = ct.place(model.Ad.T,model.Cd.T, [0.4, 0.3]).T

##setting up the observer 
observer=Observer(model.Ad,model.Bd,model.Cd,L)

## initial state 
X_0=np.array([[0.5],[0]])

##setting up the controller
X_REF=np.array([[0.0], [0.0]])
controller=StateFeedbackController(K,X_REF)
#controller=PID_Controller(10,0,5,ballbeam_config.dt)
#controller.setReference(0.5)


##logging setup
Logger=SimLog()

##plotting initialisations
Plotter=Plotting()
##for loop config
N = int(ballbeam_config.T / ballbeam_config.dt) #nuber of array elements 
u = np.array([[0.0]]) #initial control input 


## creating the simulator
BBS_sim=Simulator(model,X_0)

## control loop
for k in range(0,N):

    ## 1  simulate plant 
    y, X = BBS_sim.step(u)

    ## 2 estimate state
    Xhat=observer.reconstruct(y,u)

    ## 3 calculate the control input 
    u=controller.compute(X)

    ## 4 log into arrays
    t=k*ballbeam_config.dt
    Logger.log(t,y,u,Xhat)

## plotting
Plotter.plotAll(Logger)
## validation
Validation=Validate_StateFeedback(model,K,ballbeam_config)