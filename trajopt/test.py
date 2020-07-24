import casadi as ca
from matplotlib import pyplot as plt
import numpy as np
import time
opti = ca.Opti()

x = 0
dx = 0
u = 0
T = 0.1
t = ca.linspace(0, T, 20)
a = {}
for i in range(5):    
    a.update({'a'+str(i) : opti.variable(1)})
    x += a['a'+str(i)]*(t**i)
    if i-1 >= 0:
        dx += a['a'+str(i)]*(t**(i-1))
    if i-2 >= 0:
        u += a['a'+str(i)]*(t**(i-2))

# opti.subject_to(ca.jacobian(dx, t) == u)
# opti.subject_to(ca.jacobian(x, t) == dx)

opti.subject_to(x[-1] == 0)
opti.subject_to(x[0] == 5)
opti.subject_to(dx[-1] == 0)
# opti.subject_to(dx[0] == 0)
# opti.subject_to(u[-1] == 0)
# opti.subject_to(u[0] == 0)


p_opts = {"expand":True}
s_opts = {"max_iter": 500}
opti.solver("ipopt",p_opts, s_opts)

# try:
fig = plt.figure()
# debug = fig.add_subplot(121, autoscale_on=False, xlim=(0, 2), ylim=(-50, 50))
# debug.grid()

ax = fig.add_subplot(111, autoscale_on=False, xlim=(0, T), ylim=(-10, 10))
ax.grid()

# def callback(x):
#     return debug.plot(t, opti.debug.value(x))

# opti.callback(lambda i: callback(opti.debug.value(x)))
try:
    sol = opti.solve() 
    print(sol.stats()["iter_count"])

    # print(sol.value(x))
    # t = np.linspace(0, 2, 20)
    # f = - 1 - 2*t - 3*(t**2) - 2*(t**3) - 5*(t**4) - 6*(t**5)

    ax.plot(t, sol.value(x), 'r-' , label='x')
    ax.plot(t, sol.value(dx), 'b-' , label='dx')
    ax.plot(t, sol.value(u), 'g-' , label='u')

    plt.show()
except:
    debug_x = opti.debug.value(x)
    debug_dx = opti.debug.value(dx)
    debug_u = opti.debug.value(u)

    print(debug_x)
    print(debug_dx)
    print(debug_u)

    ax.plot(t, debug_x, 'r-' , label='x')
    ax.plot(t, debug_dx, 'b-' , label='dx')
    ax.plot(t, debug_u, 'g-' , label='u')

    plt.show()
