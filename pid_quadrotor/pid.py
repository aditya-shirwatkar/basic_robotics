import numpy as np
import matplotlib.pyplot as plt
import time
import matplotlib.animation as animation

kd = 1
ki = 0.01
kp = 5.

kvy = 1.
kvt = 1.
kpy = 1.
kpt = 1.
kvx = 1.
kpx = 1.

class Quadrotor():
    def __init__(self):
        super().__init__()
        self.N = 80
        self.dt = 1/self.N
        self.m = 0.5
        self.g = 9.81
        self.l = 2.5
        self.r = self.l/2
        self.I = self.m*(self.l**2)/12
        self.state = None
        self.statedot = None
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

        x_ddot = -((u1 + u2)*sintheta)/self.m
        y_ddot = (((u1 + u2)*costheta) - (self.m*self.g))/self.m
        theta_ddot = ((u1 - u2)*self.r)/self.I

        theta_dot = theta_dot + theta_ddot*self.dt
        y_dot = y_dot + y_ddot*self.dt
        x_dot = x_dot + x_ddot*self.dt

        theta = theta + theta_dot * self.dt
        y = y + y_dot * self.dt
        x = x + x_dot * self.dt
        self.state = np.array([x, y, theta, x_dot, y_dot, theta_dot]).reshape(6, 1)
        self.statedot = np.array([x_dot, y_dot, theta_dot, x_ddot, y_ddot, theta_ddot]).reshape(6, 1)

initial_state = np.array([np.random.uniform(low=-2.5+0.01, high=2.5-0.01),
                      np.random.uniform(low=-2.5+0.01, high=2.5-0.01),
                      np.random.uniform(low=-np.pi/6, high=np.pi/6),
                      0,0,0]).reshape(6, 1)

initial_statedot = np.zeros((6, 1))
goal_state = np.zeros((6, 1))

done = False

model = Quadrotor()

model.state = initial_state
model.statedot = initial_statedot

jj = 0
x = [model.state[0, 0]]; y = [model.state[1, 0]]; theta = [model.state[2, 0]]
del_x = 0; del_y = 0; del_theta = 0
while not done:
    del_x += -model.state[0, 0]
    del_y += -model.state[1, 0]
    del_theta += -model.state[2, 0]

    # u = np.array([0, 0])
    u1 = ((kd*(model.state[0, 0] - x[-1]) + kd*(model.state[1, 0] - y[-1]) 
        + kd*(model.state[2, 0] - theta[-1]))
        + (kp*(- x[-1]) + kp*(- y[-1]) 
        + kp*(- theta[-1]))
        + (ki*(del_x) + ki*(del_y) 
        + ki*(del_theta))
        )/2 
    u2 = ((kd*(model.state[0, 0] - x[-1]) + kd*(model.state[1, 0] - y[-1]) 
        - kd*(model.state[2, 0] - theta[-1])
        + (kp*(- x[-1]) + kp*(- y[-1]) 
        - kp*(- theta[-1]))
        + (ki*(del_x) + ki*(del_y) 
        - ki*(del_theta))
        ))/2
    
    # phi_c = (-1/model.g)*(-kvx*model.state[3, 0] - kpx*model.state[0, 0])

    # u1 = ((model.m*(model.g - kvy*model.state[4, 0] - kpy*model.state[1, 0]))
    #     + (model.I*(kvt*model.statedot[5, 0] - kpt*(phi_c - model.state[2, 0]))))/2

    # u2 = ((model.m*(model.g - kvy*model.state[4, 0] - kpy*model.state[1, 0]))
    #     - (model.I*(kvt*model.statedot[5, 0] - kpt*(phi_c - model.state[2, 0]))))/2
    # print(u1, u2)
    # u1 = ((((kp*(model.statedot[3, 0]) + ki*del_x)*model.m - kd*model.m*model.g)/np.sin(model.state[2, 0]))
    #     + ((kp*(model.statedot[5, 0]) + ki*del_theta)*model.m/model.r))/2
    
    # u2 = ((((kp*(model.statedot[3, 0]) + ki*del_x)*model.m - kd*model.m*model.g)/np.sin(model.state[2, 0]))
    #     - ((kp*(model.statedot[5, 0]) + ki*del_theta)*model.m/model.r))/2
    
    u = [u1, u2]

    model.simulate(u[0], u[1])
    # xdot = np.dot(model.A, model.state) + np.dot(model.B, u) 
    # model.state = model.state + self.dt*xdot
    # print(model.state)
    ## for animation
    x.append(model.state[0, 0])
    y.append(model.state[1, 0])
    theta.append(model.state[2, 0])
    done = bool(
            abs(x[-1]) < 0.1
            and abs(y[-1]) < 0.1
            and abs(theta[-1]) < 0.01
            # and abs(x_dot) < 0.1
            # and abs(y_dot) < 0.1
            # and abs(theta_dot) < 0.1
        )
    done = bool(
            abs(x[-1]) >= 5
            or abs(y[-1]) >= 5
    )
    # jj += 1
    # if jj > N:
    #     breakd

# print(x)
t = np.linspace(0, len(x), model.N)

fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-5, 5), ylim=(-5, 5))
ax.grid()

time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)
base, = ax.plot([], [], '-', lw=5, color='tan')
fan1, = ax.plot([], [], '-', lw=8, color='black')
fan2, = ax.plot([], [], '-', lw=8, color='black')
goal, = ax.plot([], [], linestyle='none', marker='+', color='green')
cross, = ax.plot([], [], linestyle='none', marker='+', color='red')
frame, = ax.plot([], [], '-', lw=10, color='tan')
def init():

    base.set_data([], [])
    frame.set_data([], [])
    fan1.set_data([], [])
    fan2.set_data([], [])
    goal.set_data([], [])
    cross.set_data([], [])
    time_text.set_text('')
    return base, frame, fan1, fan2, goal, cross,time_text

def animate(i):

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
    return base, frame, fan1, fan2, goal, cross, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(0, len(x)),
                              interval=25, blit=True, init_func=init)

# ani.save('double_pendulum.mp4', fps=60)
plt.show()
    

