# Import all the necessary lib

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint

'''
This project aims to develop a 1-D drone control which is controlled by PID.


The controller will control the z direction of the drone.

The dymanic model for the same can be written as:

m * z_dbldot = - m * g + u ;

for the odeint function the model used is
z_dbldot = - g + u/m

where u is the thurst input controlled by the controller

'''

# define the constants 
m = 0.5  # kg
g = 9.81 # m/s2

# PID tuning parameters
Kp = 170
Kd = 20
Ki = 9

# Simulation Time Parameters
simulation_time = 40 # seconds
time_points = simulation_time * 100 + 1
#define the function model

def zsolver(x,t,u):
    z , zdot = x
    xdot = [ zdot, -g + (u / m)]
    return xdot

# X is the vector required to solve

# Initial Condition 
x0 = [0,0]

# set the timepoints
t = np.linspace(0,simulation_time,time_points)

delta_t = t[1] - t[0] # The time difference between the two time points

# give the  to the model input
u = np.zeros(time_points)

# Arrays to store the solutions
z = np.empty_like(t)
zdot = np.empty_like(t)

# Record the intial condition in the new solution arrays
z[0],zdot[0] = x0

# Set the desired z values
# Sets the desired position and velocity to the values inside. The desired values now can change with time.
zdes = np.vstack( (np.empty_like(t) , np.empty_like(t)) )       
zdes[0] = np.zeros(t.size)
zdes[0][501:1001] = 1
zdes[0][1501:2001] = -1
zdes[0][2501:3501] = np.linspace(0,4,1000)
zdes[1] = np.zeros(t.size)
# zdes[0] is the desired postion and zdes[1] is desired the velocity

# Setiing up the error array
error = np.empty_like(zdes)
error[:,0] = zdes[:,0] - x0
int_error = 0

# The error[0] is position error and error[1] is the velocity error

#solve the ODE

for i in range(1,time_points):
    tspan = [t[i-1],t[i]]
    error[:,i] = zdes[:,i] - x0  # x0 contains the current state for the error calculation
    int_error = int_error + error[0][i] * delta_t 
    u[i] =  m * ( g + Kp * error[0][i] + Kd * error[1][i]  + Ki * int_error )

    x = odeint(zsolver,x0,t,args=(u[i],))

    # Store the solution for plotting

    z[i] , zdot[i]  = x[1]
    # giving the next initial condition to the solver
    x0 = x[1]

print('The max value of the z is :' , np.nanmax(z))

# Plot the result

plt.plot(t,zdes[0],'g:',label = 'z desired')
plt.plot(t,z,'r-',label = 'z(t)')
plt.plot(t,zdot,'b--', label = 'zdot(t)')
plt.xlabel('Time')
plt.legend(loc = 'best')
plt.show()




 



