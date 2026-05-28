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

## functions
def SIM_PID_Control(Kp,Ki,Kd,Max_actuation,X_0,X_REF,config_file):
    model=BallBeam(config_file.H,config_file.dt)
    ## observer feedback gain 
    L = ct.place(model.Ad.T,model.Cd.T, [0.5, 0.6]).T

    ##setting up the observer 
    observer=Observer(model.Ad,model.Bd,model.Cd,L)


    controller=PID_Controller(Kp,Ki,Kd,Max_actuation,config_file.dt)
    controller.setReference(X_REF[0,0])
    ##logging setup
    Logger=SimLog()
    ##sim setup
    BBS_sim=Simulator(model,X_0)
    controlSim=ControlLoop(ballbeam_config,model,controller,observer,BBS_sim)
    ## run control loop
    Logger=controlSim.run_continuous_control_loop(X_0,Logger)
    plt.figure()
    plt.plot(Logger.t_hist, Logger.y_hist)
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("position")
    plt.title('PID control')
    

    plt.figure()
    plt.plot(Logger.t_hist, Logger.u_hist)
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("control input")
    plt.title('PID control')

    plt.show()
    return Logger
def SIM_Full_State_Feedback(poles,Max_actuation,X_0,X_REF,config_file):
    model=BallBeam(config_file.H,config_file.dt)
    
     ## observer feedback gain 
    L = ct.place(model.Ad.T,model.Cd.T, [0.5, 0.6]).T

    ##setting up the observer 
    observer=Observer(model.Ad,model.Bd,model.Cd,L)

    K = ct.place(model.Ad, model.Bd, poles)
    controller=StateFeedbackController(K,X_REF,Max_actuation)

    BBS_sim=Simulator(model,X_0)
    controlSim=ControlLoop(ballbeam_config,model,controller,observer,BBS_sim)

    ##logging setup
    Logger=SimLog()

    ## run control loop
    Logger=controlSim.run_continuous_control_loop(X_0,Logger)

    ##plotting initialisations
    Plotter=Plotting()

    ## plotting
    plt.figure()
    plt.plot(Logger.t_hist, Logger.y_hist)

    plt.grid(True)

    plt.xlabel("time [s]")
    plt.ylabel("position")
    plt.title('State Feedback, No oberver, Pole Placement')

    plt.figure()

    plt.plot(Logger.t_hist, Logger.u_hist)

    plt.grid(True)

    plt.xlabel("time [s]")
    plt.ylabel("control input")
    plt.title('State Feedback, No oberver, Pole Placement')
    return Logger

def SIM_Observer_Feedback_Poles(poles,Max_actuation,X_0,X_REF,config_file):
    model=BallBeam(config_file.H,config_file.dt)
    
     ## observer feedback gain 
    L = ct.place(model.Ad.T,model.Cd.T, [0.5, 0.6]).T

    ##setting up the observer 
    observer=Observer(model.Ad,model.Bd,model.Cd,L)

    K = ct.place(model.Ad, model.Bd, poles)
    controller=StateFeedbackController_Observer(K,X_REF,Max_actuation,observer)

    BBS_sim=Simulator(model,X_0)
    controlSim=ControlLoop(ballbeam_config,model,controller,observer,BBS_sim)

    ##logging setup
    Logger=SimLog()

    ## run control loop
    Logger=controlSim.run_continuous_control_loop(X_0,Logger)

    ##plotting initialisations
    Plotter=Plotting()

    ## plotting
    Plotter.plotAll(Logger,'State feedback with observer, pole placement')

    return Logger
def SIM_Observer_Feedback_LQR(Q,R,Max_actuation,X_0,X_REF,config_file):
    model=BallBeam(config_file.H,config_file.dt)
    
     ## observer feedback gain 
    L = ct.place(model.Ad.T,model.Cd.T, [0.5, 0.6]).T

    ##setting up the observer 
    observer=Observer(model.Ad,model.Bd,model.Cd,L)

    K,_,_=ct.dlqr(model.Ad,model.Bd,Q,R)
    controller=StateFeedbackController_Observer(K,X_REF,Max_actuation,observer)

    BBS_sim=Simulator(model,X_0)
    controlSim=ControlLoop(ballbeam_config,model,controller,observer,BBS_sim)

    ##logging setup
    Logger=SimLog()

    ## run control loop
    Logger=controlSim.run_continuous_control_loop(X_0,Logger)

    ##plotting initialisations
    Plotter=Plotting()

    ## plotting
    Plotter.plotAll(Logger,'LQR')

    return Logger

