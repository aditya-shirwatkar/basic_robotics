import numpy as np
import matplotlib.pyplot as plt
import time
import matplotlib.animation as animation

#########################################################
#########################################################
################# Quadrotor Model #######################
#########################################################
#########################################################

class Quadrotor():
    def __init__(self):
        super().__init__()
        self.N = 80
        self.dt = 1/self.N
        self.m = 1.
        self.g = 9.81
        self.l = 2.5
        self.r = self.l/2
        self.I = self.m*(self.l**2)/12
        self.state = None
        self.statedot = None
        self.maplimit = 5
        # M(q) & B(q) since C(q, q_dot) and Tau(q) is zero
        self.M = np.matrix([
            [self.m, 0, 0],
            [0, self.m, 0],
            [0, 0, self.I],
        ])

        self.b = np.matrix([
            np.zeros(2),
            [1, 1],
            [self.r, -self.r]
        ])
    
        self.iM = np.linalg.inv(self.M)

        self.A, self.B = self.returnLinearizedDynamics(self.iM, self.b)

    def returnLinearizedDynamics(self, iM, b):
        db = np.matrix([
                        [0 , 0, self.m*self.g],
                        np.zeros(3),
                        np.zeros(3)
                        ])

        A = np.concatenate((
                        np.concatenate((np.zeros((3, 3)), np.eye(3)), axis=1),
                        np.concatenate((np.dot(iM, db), np.zeros((3, 3))), axis=1)
                    ), axis=0)

        B = np.concatenate((
                        np.zeros((3, 2)),
                        np.dot(iM, b)
                    ), axis=0)

        return A, B

    def simulate(self, u1, u2):
        x, y, theta, x_dot, y_dot, theta_dot = self.state

        costheta = np.cos(theta)
        sintheta = np.sin(theta)

        theta = np.arctan2(sintheta, costheta)

        x_ddot = -((u1 + u2)*theta)/self.m
        y_ddot = (((u1 + u2)*1.) - (self.m*self.g))/self.m
        theta_ddot = ((u1 - u2)*self.r)/self.I

        theta_dot = theta_dot + theta_ddot*self.dt
        y_dot = y_dot + y_ddot*self.dt
        x_dot = x_dot + x_ddot*self.dt

        # theta = np.arctan2(np.sin(theta + theta_dot * self.dt), np.cos(theta + theta_dot * self.dt))
        theta = theta + theta_dot * self.dt
        y = y + y_dot * self.dt
        x = x + x_dot * self.dt

        self.state = np.array([x, y, theta, x_dot, y_dot, theta_dot]).reshape(6, 1)
        self.statedot = np.array([x_dot, y_dot, theta_dot, x_ddot, y_ddot, theta_ddot]).reshape(6, 1)

#########################################################
#########################################################
################# PID Code Loop #########################
#########################################################
#########################################################

ki = 0.1
kd = 1.
kp = 5.

kp_x = 24
kv_x = 4

kp_y = 77
kv_y = 25

kp_t = 75
kv_t = 400

kiy = 1.
kpy = 1.
kpt = 1.
kit = 1.

TUNING_MATRIX = np.array([
    [5,10,0],
    [5,10,0],
    [5,10,0]    
])
TUNING_MATRIX = np.array([
    [5,20,0],
    [5,20,0],
    [1,20,0]    
])


initial_state = np.array([np.random.uniform(low=-2.5+0.01, high=2.5-0.01),
                      np.random.uniform(low=-2.5+0.01, high=2.5-0.01),
                      np.random.uniform(low=-np.pi/3, high=np.pi/3),
                      0,0,0]).reshape(6, 1)

initial_statedot = np.zeros((6, 1))
goal_state = np.zeros((6, 1))

done = False

model = Quadrotor()

model.state = initial_state
model.statedot = initial_statedot

time_counter = 0
x = [model.state[0, 0]]; y = [model.state[1, 0]]; theta = [model.state[2, 0]]
dx = [model.state[3, 0]]; dy = [model.state[4, 0]]; dtheta = [model.state[5, 0]]

del_x = 0; del_y = 0; del_theta = 0

Kp_z, Kd_z , Ki_z = TUNING_MATRIX[0,:]
Kp_y, Kd_y , Ki_y = TUNING_MATRIX[1,:]
Kp_th, Kd_th , Ki_th = TUNING_MATRIX[2,:]

des_state = np.array([0,0,0,0,0,0]) # z,y,zdot,ydot,zdotdot,y dotdot

count = 0
while not done:

    # del_x += -model.state[0, 0]
    # del_y += -model.state[1, 0]
    # del_theta += -np.arctan2(np.sin(model.state[2, 0]), np.cos(model.state[2, 0]))

    u = np.array([0, 0])
    ###############################################################
    ######## My First Try of u1, u2 which Failed ##################
    ###############################################################
    # u1 = ((kd*(model.state[0, 0] - x[-1]) + kd*(model.state[1, 0] - y[-1]) 
    #     + kd*(model.state[2, 0] - theta[-1]))
    #     + (kp*(- x[-1]) + kp*(- y[-1]) 
    #     + kp*(- theta[-1]))
    #     + (ki*(del_x) + ki*(del_y) 
    #     + ki*(del_theta))
    #     )/2 
    # u2 = ((kd*(model.state[0, 0] - x[-1]) + kd*(model.state[1, 0] - y[-1]) 
    #     - kd*(model.state[2, 0] - theta[-1])
    #     + (kp*(- x[-1]) + kp*(- y[-1]) 
    #     - kp*(- theta[-1]))
    #     + (ki*(del_x) + ki*(del_y) 
    #     - ki*(del_theta))
    #     ))/2

    model.state[2] = np.arctan2(np.sin(model.state[2]), np.cos(model.state[2]))
    phi_c = (-1/model.g) * (Kd_y * (- model.state[3]) +
                                    Kp_y * (- model.state[0]))

    u1 = ((model.m * ( model.g + 
                            Kd_z * (- model.state[4]) +
                            Kp_z * (- model.state[1])
                    )) + 
        (model.I * (Kd_th * (-1 * model.state[5]) +
                    Kp_th * (phi_c - model.state[2]) ))
        )/2

    u2 = ((model.m * ( model.g + 
                            Kd_z * (- model.state[4]) +
                            Kp_z * (- model.state[1])
                    )) - 
        (model.I * (Kd_th * (-1 * model.state[5]) +
                    Kp_th * (phi_c - model.state[2]) ))
        )/2

    ###################################################################
    ######## My Second Try of u1, u2 which Failed too #################
    ###################################################################
    # x = 0.5sin(0.5t) , y = 5sin(t)
    # phi_c = ((-1/model.g)*((-(0.5**3)*np.sin(0.5*time_counter)) + kv_x*(((0.5**2)*np.cos(0.5*time_counter)) - model.state[3, 0]) 
    #                 + kp_x*( 0.5*np.sin(0.5*time_counter) - model.state[0, 0])))
    # u1 = ((model.m*(-5*np.sin(time_counter) + model.g + kv_y*(5*np.cos(time_counter) - model.state[4, 0]) - kp_y*(5*np.sin(time_counter)- model.state[1, 0])))
    #     + (model.I*(kv_t*model.statedot[5, 0] - kp_t*(phi_c - model.state[2, 0]))))/2
    # u2 = ((model.m*(-5*np.sin(time_counter) + model.g + kv_y*(5*np.cos(time_counter) - model.state[4, 0]) - kp_y*(5*np.sin(time_counter)- model.state[1, 0])))
    #     - (model.I*(kv_t*model.statedot[5, 0] - kp_t*(phi_c - model.state[2, 0]))))/2
    # print(u1, u2)

    ###################################################################
    ######## My Thrid Try of u1, u2 which Failed too ##################
    ###################################################################
    # u1 = ((((kp*(model.statedot[3, 0]) + ki*del_x)*model.m - kd*model.m*model.g)/np.sin(model.state[2, 0]))
    #     + ((kp*(model.statedot[5, 0]) + ki*del_theta)*model.m/model.r))/2
    # u2 = ((((kp*(model.statedot[3, 0]) + ki*del_x)*model.m - kd*model.m*model.g)/np.sin(model.state[2, 0]))
    #     - ((kp*(model.statedot[5, 0]) + ki*del_theta)*model.m/model.r))/2

    ###################################################################
    ######## My Fourth Try of u1, u2 which Failed too #################
    ###################################################################
    # u1 = ((model.m*(kpy*model.statedot[4, 0] + kiy*model.statedot[1, 0]) + model.m*model.g)/np.sin(model.state[2, 0]) 
    #     + model.I*((kpt*model.statedot[5, 0] + kiy*model.statedot[2, 0])/model.r))/2

    # u2 = ((model.m*(kpy*model.statedot[4, 0] + kiy*model.statedot[1, 0]) + model.m*model.g)/np.sin(model.state[2, 0]) 
    #     - model.I*((kpt*model.statedot[5, 0] + kiy*model.statedot[2, 0])/model.r))/2

    u = [u1, u2]

    # Simulate the Quadrotor based on inputs u1 and u2
    model.simulate(u[0], u[1])

    # xdot = np.dot(model.A, model.state) + np.dot(model.B, u) 
    # model.state = model.state + self.dt*xdot
    # print(model.state)
    
    # for animation store variables
    x.append(model.state[0, 0])
    y.append(model.state[1, 0])
    theta.append(np.arctan2(np.sin(model.state[2, 0]), np.cos(model.state[2, 0])))

    dx.append(model.state[3, 0])
    dy.append(model.state[4, 0])
    dtheta.append(np.arctan2(np.sin(model.state[5, 0]), np.cos(model.state[5, 0])))


    # Termination of Loop

    done = bool(
            abs(x[-1]) >= model.maplimit
            or abs(y[-1]) >= model.maplimit
        # or    
        # (   abs(x[-1]) < 0.1
        #     and abs(y[-1]) < 0.1
        #     # and abs(theta[-1]) < 0.1
        #     )
        or time_counter >= model.N    
    )

    time_counter += model.dt



#########################################################
#########################################################
##################### Animation #########################
#########################################################
#########################################################

t = np.linspace(0, len(x), model.N)

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-model.maplimit, model.maplimit), ylim=(-model.maplimit, model.maplimit))
ax.grid()

# state_plot = fig.add_subplot(122, autoscale_on=True, xlim=(0, len(x)), ylim=(-5, 5))
# state_plot.grid()

# posx, = state_plot.plot([], [], '-', label='x')
# posy, = state_plot.plot([], [], '-', label='y')
# postheta, = state_plot.plot([], [], '-', label='theta')

# vely, = state_plot.plot([], [], '-', label='dx')
# velx, = state_plot.plot([], [], '-', label='dy')
# veltheta, = state_plot.plot([], [], '-', label='dtheta')


time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
base, = ax.plot([], [], '-', lw=5, color='tan')
fan1, = ax.plot([], [], '-', lw=8, color='black')
fan2, = ax.plot([], [], '-', lw=8, color='black')
goal, = ax.plot([], [], linestyle='none', marker='+', color='green')
cross, = ax.plot([], [], linestyle='none', marker='+', color='red')
frame, = ax.plot([], [], '-', lw=10, color='tan')

# lx,ly,lt,ldx,ldy,ldt = [],[],[],[],[],[]

def init():
    # lx,ly,lt,ldx,ldy,ldt = [],[],[],[],[],[]
    # posx.set_data([], [])
    # posy.set_data([], [])
    # postheta.set_data([], [])
    # velx.set_data([], [])
    # vely.set_data([], [])
    # veltheta.set_data([], [])

    base.set_data([], [])
    frame.set_data([], [])
    fan1.set_data([], [])
    fan2.set_data([], [])
    goal.set_data([], [])
    cross.set_data([], [])
    time_text.set_text('')
    # posx, posy, postheta, velx, vely, veltheta, 
    return base, frame, fan1, fan2, goal, cross,time_text

def animate(i):
    # global lx, ly, lt, ldy, ldx, ldt
    # if i == len(x):
    #     lx,ly,lt,ldx,ldy,ldt = [],[],[],[],[],[]

    # lx.append(x[i]);ly.append(y[i]);lt.append(theta[i]);ldx.append(dx[i]);ldy.append(dy[i]);ldt.append(dtheta[i]);
    
    thisx = [x[i] - model.r*np.cos(theta[i]), 
             x[i] + model.r*np.cos(theta[i])]
    thisy = [y[i] - model.r*np.sin(theta[i]), 
                y[i] + model.r*np.sin(theta[i])]
    thisx2 = [x[i] - (model.r/4)*np.cos(theta[i]), 
             x[i] + (model.r/4)*np.cos(theta[i])]
    thisy2 = [y[i] - (model.r/4)*np.sin(theta[i]), 
                y[i] + (model.r/4)*np.sin(theta[i])]

    g = [0., 0.]
    cx = [x[i]]
    cy = [y[i]]

    f1x = [ x[i] + (9.9*model.r/10)*np.cos(theta[i]), 
            x[i] + (10.1*model.r/10)*np.cos(theta[i])]
    f1y = [y[i] + (9.9*model.r/10)*np.sin(theta[i]) + 0.15,  
           y[i] + (10.1*model.r/10)*np.sin(theta[i]) + 0.15]
    
    f2x = [ x[i] - (9.9*model.r/10)*np.cos(theta[i]), 
            x[i] - (10.1*model.r/10)*np.cos(theta[i])]
    f2y = [y[i] - (9.9*model.r/10)*np.sin(theta[i]) + 0.15,  
           y[i] - (10.1*model.r/10)*np.sin(theta[i]) + 0.15]

    base.set_data(thisx, thisy)
    frame.set_data(thisx2, thisy2)

    fan1.set_data(f1x, f1y)
    fan2.set_data(f2x, f2y)
    goal.set_data(g)
    cross.set_data(cx, cy)
    time_text.set_text(time_template % (i*20/len(t)))

    # posx.set_data(t[0:i], lx[0:i])
    # posy.set_data(t[0:i], ly[0:i])
    # postheta.set_data(t[0:i], lt[0:i])
    # velx.set_data(t[0:i], ldx[0:i])
    # vely.set_data(t[0:i], ldy[0:i])
    # veltheta.set_data(t[0:i], ldt[0:i])

    # posx, posy, postheta, velx, vely, veltheta, 

    return base, frame, fan1, fan2, goal, cross, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(0, len(x)),
                              interval=25, blit=True)

# ani.save('spoiler.mp4')
plt.show()
    

