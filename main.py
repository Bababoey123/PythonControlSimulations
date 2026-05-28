from Models.ballbeam import BallBeam
from Models import ballbeam_config
from Simulation.simulation import Simulator
from Simulation.simulation import ControlLoop
from Control.observer import Observer
from Control.StateFeedback import StateFeedbackController
from Control.StateFeedback import StateFeedbackController_Observer
from Control.PID import PID_Controller
from Metrics_Plotting.SimLog import SimLog
from Metrics_Plotting.Plotting import Plotting
from Validation.validate_StateFeeback import Validate_StateFeedback



import control as ct
import numpy as np
import matplotlib.pyplot as plt 

model=BallBeam(ballbeam_config.H,ballbeam_config.dt)

## Cost Matrices 
Q = np.diag([2, 2])
R = np.array([[20]])

## state feeddback gain
K,_,_=ct.dlqr(model.Ad,model.Bd,Q,R)
#K = ct.place(model.Ad, model.Bd, [0.9, 0.95])
## observer feedback gain 
L = ct.place(model.Ad.T,model.Cd.T, [0.5, 0.6]).T

##setting up the observer 
observer=Observer(model.Ad,model.Bd,model.Cd,L)

## initial state 
X_0=np.array([[0.0],[0]])

##setting up the controller
X_REF=np.array([[1.0], [0.0]])
controller=StateFeedbackController_Observer(K,X_REF,observer,10)
#controller=PID_Controller(10,0,5,ballbeam_config.dt)
#controller.setReference(0.5)


##logging setup
Logger=SimLog()

##plotting initialisations
Plotter=Plotting()



##control loop setup
BBS_sim=Simulator(model,X_0)
controlSim=ControlLoop(ballbeam_config,model,controller,observer,BBS_sim)
## run control loop
Logger=controlSim.run_continuous_control_loop(X_0,Logger)

## plotting
Plotter.plotAll(Logger)
## validation
Validation=Validate_StateFeedback(model,K,ballbeam_config)
Validation.Validate(ballbeam_config,Logger,X_0,Plotter)