 # -*- coding: utf-8 -*-
"""
Created on Tue Jan 17 16:54:42 2017

@author: user
"""
from scipy.integrate import ode
from ComputeEventsHyQ import ComputeEventsHyQ
import matplotlib.pyplot as plt
import math
import numpy as np
import matplotlib.animation as animation

plt.clf()
plt.close('all')


numberOfLegs = 4  # Define the total number of legs of the robot
x_0 =  [[0 ] for i in range(0,numberOfLegs*2)] # Define initial state for max-plus algebra 
Ts = 0.01 # Sampling time for the simulation
EventsList0 = [[0 for i in range(0,numberOfLegs*2)]] # List of lift-off and touchdown events for each of the legs
total_time = 80# Total simulation time
time_axis = np.arange(0,total_time,Ts) # Time vector
compare1 = [0 for i in range (0,numberOfLegs)] # Comparison to verify if an element from the events list needs to be created
h = 0# Counter for the EventsList
w = [[0 for i in range(numberOfLegs)] for j in range(len(time_axis))] # Angular frequency initialization for each of the legs
#w = [0 for i in range(0,len(time_axis))]
# Oscillator constant parameters
Df_vector = [0 for i in range (0,len(time_axis))]
Df_matrix = [[0 for i in range(0,numberOfLegs)] for j in range(0,len(time_axis))]
bp = 100

alpha = 100
beta = 100
a = 0.6
b = 0.4 

St = 2

t1 = 10
t2 = 20
t3 = 30

# Setting of the gait parameter variation and gait changes throughout time
for i in range(0,len(time_axis)):    
    if time_axis[i] < t1:
        Df = 0.8
        Tf = St*(1 - Df)
        Tg = St*Df
        Gr = 2
        MaxTD1 = St*(1 - Gr*(1 - Df))
        Td = [0.2*MaxTD1,0.4*MaxTD1] # MaxTd = 0.6
        gait = [[1,4],[2,3]]
    elif (time_axis[i] >= t1) and (time_axis[i] < t2):
        Df = 0.2
        Tf = St*(1 - Df)
        Tg = St*Df
        Gr = 2
        MaxTD2 = St*(1 - Gr*(1 - Df))
        Td = [-0.2,-1]  # MaxTd = -0.6
        Df_vector[i] = Tg/(Tg + Tf) 
        gait = [[1,4],[2,3]]
    elif (time_axis[i] >= t2) and (time_axis[i] < t3):
        Df = 0.8
        Tf = St*(1 - Df)
        Tg = St*Df
        Gr = 4
        MaxTD3 = St*(1 - Gr*(1 - Df))
        Td = [0.1,0.1,0.1,0.1] # MaxTd = 0.2
        gait = [[1],[2],[3],[4]] 
    else:
        Df = 0.2
        Tf = St*(1 - Df)
        Tg = St*Df
        Gr = 4
        MaxTD4 = St*(1 - Gr*(1 - Df))
        Td = [-1.4,-0.8,-1.4,-0.8] # MaxTd = -2.2
        gait = [[1],[2],[3],[4]]

    for j in range(0,numberOfLegs):  
        compare1[j] = min([EventsList0[h][j],EventsList0[h][j + numberOfLegs]]) - time_axis[i]
        if compare1[j] <= 0:
            h = h + 1   
            EventsList0,x_0 = ComputeEventsHyQ(EventsList0,gait,numberOfLegs,Tf,Tg,Td,x_0)              


for i in range(0,len(time_axis)):  
    
    for j in range(1,len(EventsList0)):

        for k in range(0,numberOfLegs):
                                     
            if time_axis[i] >= EventsList0[j-1][k] and time_axis[i] < EventsList0[j][k + numberOfLegs]: #Stance phase
        
                  w[i][k] = math.pi*1.18/(EventsList0[j][k + numberOfLegs] -
                                          EventsList0[j-1][k])
                                          
            elif time_axis[i] >= EventsList0[j][k + numberOfLegs] and time_axis[i] <= EventsList0[j][k]: #Swing phase
                
                  w[i][k] = math.pi*1.18/(EventsList0[j][k] -
                                          EventsList0[j][k + numberOfLegs])

            
def f1(t, u, w):
    import math
    return [alpha*(1 - math.pow(u[0],4)/math.pow(a,4) - 
            math.pow(u[1],4)/math.pow(b,4))*u[0] + (w*a)*math.pow(u[1],3)/math.pow(b,3), #x_dot
            beta*(1 - math.pow(u[0],4)/math.pow(a,4) - math.pow(u[1],4)/math.pow(b,4)
            )*u[1] - (w*b)*math.pow(u[0],3)/math.pow(a,3)] #z_dot       
u_matrix = np.zeros((len(time_axis), 2, numberOfLegs))


# First oscillator solution
x0 = 0.6
z0 = 0
u0 = [x0,z0]
r = ode(f1).set_integrator('zvode', method='adams',
                          order=10, atol=1e-6,
                          with_jacobian=False)             
r.set_initial_value(u0, 0).set_f_params(w[0][0])
u = [];  t = []; y =[];
while r.successful() and r.t <= total_time:
    r.integrate(r.t + Ts)
    u.append(r.y);  t.append(r.t)
    r.set_f_params(w[len(t) - 1][0])
u1 = u

# Second oscillator solution
x0 = 0.6
z0 = 0
u0 = [x0,z0]
r = ode(f1).set_integrator('zvode', method='adams',
                          order=10, atol=1e-6,
                          with_jacobian=False)             
r.set_initial_value(u0, 0).set_f_params(w[0][1])
u = [];  t = []; y =[];
while r.successful() and r.t <= total_time:
    r.integrate(r.t + Ts)
    u.append(r.y);  t.append(r.t)
    r.set_f_params(w[len(t) - 1][1])
u2 = u

# Third oscillator solution
x0 = 0.6
z0 = 0
u0 = [x0,z0]
r = ode(f1).set_integrator('zvode', method='adams',
                          order=10, atol=1e-6,
                          with_jacobian=False)             
r.set_initial_value(u0, 0).set_f_params(w[0][2])
u = [];  t = []; y =[];
while r.successful() and r.t <= total_time:
    r.integrate(r.t + Ts)
    u.append(r.y);  t.append(r.t)
    r.set_f_params(w[len(t) - 1][2])
u3 = u

# Fourth oscillator solution
x0 = 0.6
z0 = 0
u0 = [x0,z0]
r = ode(f1).set_integrator('zvode', method='adams',
                          order=10, atol=1e-6,
                          with_jacobian=False)             
r.set_initial_value(u0, 0).set_f_params(w[0][3])
u = [];  t = []; y =[];
while r.successful() and r.t <= total_time:
    r.integrate(r.t + Ts)
    u.append(r.y);  t.append(r.t)
    r.set_f_params(w[len(t) - 1][3])
u4 = u

z_matrix = [[row[1] for row in u1],[row[1] for row in u2],[row[1] for row in u3],[row[1] for row in u4]]

l_matrix = [[0 for i in range(0,numberOfLegs)] for j in range(0,len(time_axis))]

# Get variables for synchronization plot


for i in range(0,len(time_axis)):
    for j in range(0,numberOfLegs):
        if z_matrix[j][i] < 0:
            l_matrix[i][j] = j + 1
            
        else:
            l_matrix[i][j] = 'nan'

fig5 = plt.figure()
plt.title('Synchronization plot')
plt.axes(ylim=(4.4,0.6),xlim = (0,80))
plt.ylabel('Leg number[-]',fontsize = 20)
plt.xlabel('time[s]',fontsize = 20)

line1 = plt.plot(t,[row[0] for row in l_matrix],color = '0.75',linewidth=130,solid_capstyle="butt")
line2 = plt.plot(t,[row[1] for row in l_matrix],color = '0.75',linewidth=130,solid_capstyle="butt")
line3 = plt.plot(t,[row[2] for row in l_matrix],color = '0.75',linewidth=130,solid_capstyle="butt")
line4 = plt.plot(t,[row[3] for row in l_matrix],color = '0.75',linewidth=130,solid_capstyle="butt")
plt.axvline(x=t1,linestyle = "dashed")
plt.axvline(x=t2,linestyle = "dashed")
plt.axvline(x=t3,linestyle = "dashed")
plt.axhline(y=3.5,color = 'k')
plt.axhline(y=2.5,color = 'k')
plt.axhline(y=1.5,color = 'k')

plt.tick_params(labelsize=20)
leg_axis = [1,2,3,4]
plt.yticks(leg_axis)



      

# Plotting    
fig1 = plt.figure()
plt.title('Limit cycle plot',fontsize=30)
plt.ylabel('z',fontsize=30)
plt.xlabel('x',fontsize=30)
plt.tick_params(labelsize=30)
line1 = plt.plot([row[0] for row in u],[row[1] for row in u],linewidth=1)

fig2 = plt.figure()
plt.title('Variation of z coordinate with respect to time')
plt.axes(ylim=(-0.5,0.5),xlim = (5,25))
plt.tick_params(labelsize=20)
#line1 = plt.plot(t,[row[0] for row in u],linewidth=1)
line2 = plt.plot(t,[row[1] for row in u1],linewidth=1)
plt.axhline(y=0,linestyle = "dashed")
plt.ylabel('z[-]',fontsize = 20)
plt.xlabel('time[s]',fontsize = 20)
for i in range(0, len(EventsList0)):
    plt.axvline(x = EventsList0[i][4], linestyle = "dashed")
    plt.axvline(x = EventsList0[i][0], linestyle = "dashed")
    


fig3 = plt.figure()
plt.title('Angular frequency with respect to time leg 1')
plt.axes(xlim=(5,25))
plt.tick_params(labelsize=20)
plt.ylabel('w [rad/s]',fontsize = 20)
plt.xlabel('time[s]',fontsize = 20)
line1 = plt.plot(t,[row[0] for row in w])
for i in range(0, len(EventsList0)):
    plt.axvline(x = EventsList0[i][4], linestyle = "dashed")
    plt.axvline(x = EventsList0[i][0], linestyle = "dashed")
    
Writer = animation.writers['ffmpeg']
writer = Writer(fps=30, metadata=dict(artist='Me'), bitrate=1800)



# First set up the figure, the axis, and the plot element we want to animate
fig4 = plt.figure()
ax = plt.axes(xlim=(-1, 2.5), ylim=(-1.5, 1.5))
plt.ylabel('z')
plt.xlabel('x')

line, = ax.plot([], [], lw=1, ls = '--')
line1, = ax.plot([],[], lw =5,color = '0.5',label = 'Right')
line2, = ax.plot([], [], lw=1, ls = '--')
line3, = ax.plot([],[], lw =5,color = 'red',label = 'Left')
line4, = ax.plot([], [], lw=1, ls = '--')
line5, = ax.plot([],[], lw =5,color = '0.5')
line6, = ax.plot([], [], lw=1, ls = '--')
line7, = ax.plot([],[], lw =5,color = 'red')

legend = ax.legend(loc='upper center', shadow=True)

time_template = 'time = %.1fs'
time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)

front_text = ax.text(0.23,0.5,'REAR',transform=ax.transAxes)
rear_text = ax.text(0.65,0.5,'FRONT',transform=ax.transAxes)

# initialization function: plot the background of each frame
def init():
    line.set_data([], [])
    time_text.set_text('')
    return line,

# animation function.  This is called sequentially
i = 0

x0,y0,x1,y1,x2,y2,x3,y3 = [],[],[],[],[],[],[],[]

def animate(i):
    time_text.set_text(time_template % (i*Ts))
    x0.append(u1[i][0])
    y0.append(u1[i][1])
    dotx0 = [u1[i-1][0],u1[i][0]]
    doty0 = [u1[i-1][1], u1[i][1]]
    x1.append(u2[i][0])
    y1.append(u2[i][1])
    dotx1 = [u2[i-1][0],u2[i][0]]
    doty1 = [u2[i-1][1], u2[i][1]]
    x2.append(1.5 + u3[i][0])
    y2.append(u3[i][1])
    dotx2 = [1.5 + u3[i-1][0],1.5 + u3[i][0]]
    doty2 = [u3[i-1][1], u3[i][1]]
    x3.append(1.5 + u4[i][0])
    y3.append(u4[i][1])
    dotx3 = [1.5 + u4[i-1][0],1.5 + u4[i][0]]
    doty3 = [u4[i-1][1], u4[i][1]]
    line.set_data(x0, y0)
    line1.set_data(dotx0,doty0)
    line2.set_data(x1, y1)
    line3.set_data(dotx1,doty1)
    line4.set_data(x2, y2)
    line5.set_data(dotx2,doty2)
    line6.set_data(x3, y3)
    line7.set_data(dotx3,doty3)
    return line,

# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig4, animate, np.arange(1, len(u)), init_func=init,
                               interval=Ts, blit=False)
                              
#==============================================================================
# anim.save('Legs.mp4', writer=writer)
#==============================================================================

