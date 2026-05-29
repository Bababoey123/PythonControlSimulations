from Utils import BallBeam_Control_Sims

import control as ct
import numpy as np
import matplotlib.pyplot as plt 


## Cost Matrices 
Q = np.diag([5, 5])
R = np.array([[2]])
## Discrete PID parameters
Kp=4
Ki=0
Kd=5
##Poles z-domain
Poles=[0.95,0.98]

## initial state 
X_0=np.array([[0.0],[0]])

##setting up the controller
X_REF=np.array([[1.0], [0.0]])
Max_actuation=10

Logger=BallBeam_Control_Sims.SimLog()

BallBeam_Control_Sims.SIM_Observer_Feedback_LQR(Q,R,Max_actuation,X_0,X_REF,BallBeam_Control_Sims.ballbeam_config)


BallBeam_Control_Sims.SIM_PID_Control(Kp,Ki,Kd,Max_actuation,X_0,X_REF,BallBeam_Control_Sims.ballbeam_config)


BallBeam_Control_Sims.SIM_Observer_Feedback_Poles(Poles,Max_actuation,X_0,X_REF,BallBeam_Control_Sims.ballbeam_config)