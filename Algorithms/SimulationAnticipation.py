 # -*- coding: utf-8 -*-
"""
Created on Tue Jan 17 16:54:42 2017

@author: user
"""
from scipy.integrate import ode
from ComputeEventsHyQ import ComputeEventsHyQ
from ComputeEventsHyQ2 import ComputeEventsHyQ2
from ReplaceEventsHyQ import ReplaceEventsHyQ
import matplotlib.pyplot as plt
import math
import numpy as np
import matplotlib.animation as animation
from matplotlib import rc

plt.clf()
plt.close('all')


numberOfLegs = 4  # Define the total number of legs of the robot
x_0 =  [[0] for i in range(0,numberOfLegs*2)] # Define initial state for max-plus algebra 
#==============================================================================
# x_0[0] = [10]
#==============================================================================
Ts = 0.01 # Sampling time for the simulation
#==============================================================================
# ts = 0.000005 # Sampling time for integration
#==============================================================================
ts = 0.0005  # Sampling time for integration
EventsList = [[0 for i in range(0,numberOfLegs*2)]] # List of lift-off and touchdown events for each of the legs
total_time = 30# Total simulation time
time_axis = np.arange(0,total_time,Ts) # Time vector
compare1 = [0 for i in range (0,numberOfLegs)] # Comparison to verify if an element from the events list needs to be created
compare = 0


h = 0# Counter for the EventsList
w = [[0 for i in range(numberOfLegs)] for j in range(len(time_axis))] # Angular frequency initialization for each of the legs

# Oscillator constant parameters
Df_desired_vector = [0 for i in range (0,len(time_axis))]
bp = 100

#==============================================================================
# alpha = 0.0005
# beta = 0.0005
#==============================================================================
alpha = 1000
beta = 1000
a = 0.6 #Squared oscillator
b = 0.4 

St = 3 # Reasonable period

Df = 0.8
Tf = St*(1 - Df)
Tg = St*Df
Gr = 4
Td = [0.15,0.15,0.15,0.15]
gait = [[1],[3],[2],[4]] 
#==============================================================================
# Gr = 2
# Td = [0.6,1.2] # MaxTd = 0.6
# gait = [[2,3],[1,4]]
#==============================================================================
MaxTD1 = St*(1 - Gr*(1 - Df))


# Oscillator function       
def f1(t, u, w):
    import math
    return [alpha*(1 - math.pow(u[0],4)/math.pow(a,4) -  # Squared oscillator
            math.pow(u[1],4)/math.pow(b,4))*u[0] + (w*a)*math.pow(u[1],3)/math.pow(b,3), #x_dot
            beta*(1 - math.pow(u[0],4)/math.pow(a,4) - math.pow(u[1],4)/math.pow(b,4)
            )*u[1] - (w*b)*math.pow(u[0],3)/math.pow(a,3)] #z_dot   
  
u_matrix = np.zeros((len(time_axis), 2, numberOfLegs))

x0_1 = 0.6
z0_1 = -0.0001
x0_2 = 0.6
z0_2 = -0.0001
x0_3 = 0.6
z0_3 = -0.0001
x0_4 = 0.6
z0_4 = -0.0001

u0_1 = np.array([x0_1,z0_1])
u0_2 = np.array([x0_2,z0_2])
u0_3 = np.array([x0_3,z0_3])
u0_4 = np.array([x0_4,z0_4])

u1 = np.array([x0_1,z0_1])
u2 = np.array([x0_2,z0_2])
u3 = np.array([x0_3,z0_3])
u4 = np.array([x0_4,z0_4])

old_sign_u1 = np.sign(u0_1[1])
old_sign_u2 = np.sign(u0_2[1])
old_sign_u3 = np.sign(u0_3[1])
old_sign_u4 = np.sign(u0_4[1])

w_index = [1,1,1,1]

for i in range(0,1):
    EventsList,x_0 = ComputeEventsHyQ(EventsList,gait,numberOfLegs,Tf,Tg,Td,x_0)

#%%
# Setting of the gait parameter variation and gait changes throughout time
for i in range(0,len(time_axis) - 1):    
    
    Df_desired_vector[i] = Df    
    
    
# Creating list of events according to time instant     
#==============================================================================
#     compare = min(EventsList[max(w_index)]) - time_axis[i]
#     if compare <= 0: 
#         EventsList,x_0 = ComputeEventsHyQ(EventsList,gait,numberOfLegs,Tf,Tg,Td,x_0) 
#==============================================================================


    compare = min(x_0)[0] - time_axis[i]
    if compare <= 0: 
        h += 1
        EventsList,x_0 = ComputeEventsHyQ(EventsList,gait,numberOfLegs,Tf,Tg,Td,x_0)           
            
# Creating vector of mean angular frequencies according to EventsList times
    for k in range(0,numberOfLegs):                                     
        if time_axis[i] >= EventsList[w_index[k]-1][k] and time_axis[i] < EventsList[w_index[k]][k + numberOfLegs]: #Stance phase, squared oscillator        
              w[i][k] = math.pi*1.18/(EventsList[w_index[k]][k + numberOfLegs] -
                                      EventsList[w_index[k]-1][k])                                          
        elif time_axis[i] >= EventsList[w_index[k]][k + numberOfLegs] and time_axis[i] <= EventsList[w_index[k]][k]: #Swing phase                
              w[i][k] = math.pi*1.18/(EventsList[w_index[k]][k] -
                                      EventsList[w_index[k]][k + numberOfLegs])


    # First oscillator solution
    r = ode(f1).set_integrator('vode', method='adams',
                              order=10, atol=1e-6,
                              with_jacobian=False)             
    r.set_initial_value(u0_1, time_axis[i]).set_f_params(w[i][0])
    u = [];  t = []; y =[];
    while r.successful() and r.t < time_axis[i+1]:
        r.integrate(r.t + ts)
        u.append(r.y);  t.append(r.t)
        #r.set_f_params(w[len(t) - 1][0])
    u1 = np.vstack((u1,u[len(u) - 1]))
    u0_1 = u1[len(u1) - 1]
    sign_u1 = np.sign(u0_1[1])
    SignCheck_u1 = old_sign_u1 - sign_u1
    if old_sign_u1 == 1:
        if SignCheck_u1 != 0:
            print('Leg 1 Touchdown',time_axis[i])
            x_0[0] = [time_axis[i]]
            EventsList[w_index[0]][0] = time_axis[i]
            EventsList, x_next = ReplaceEventsHyQ(EventsList,gait,numberOfLegs,Tf,Tg,Td,x_0)
            w_index[0] += 1
    elif old_sign_u1 == -1:
        if SignCheck_u1 != 0:
            print('Leg 1 lift-off',time_axis[i])
            x_0[4] = [time_axis[i]]
            EventsList[w_index[0]][4] = time_axis[i]
    old_sign_u1 = sign_u1    

    
    # Second oscillator solution
    r = ode(f1).set_integrator('vode', method='adams',
                              order=10, atol=1e-6,
                              with_jacobian=False)             
    r.set_initial_value(u0_2, time_axis[i]).set_f_params(w[i][1])
    u = [];  t = []; y =[];
    while r.successful() and r.t < time_axis[i+1]:
        r.integrate(r.t + ts)
        u.append(r.y);  t.append(r.t)
        #r.set_f_params(w[len(t) - 1][1])
    u2 = np.vstack((u2,u[len(u) - 1]))
    u0_2 = u2[len(u2) - 1]
    sign_u2 = np.sign(u0_2[1])
    SignCheck_u2 = old_sign_u2 - sign_u2
    if old_sign_u2 == 1:
        if SignCheck_u2 != 0:
            print('Leg 2 touchdown',time_axis[i])
            EventsList[w_index[1]][1] = time_axis[i]
            x_0[1] = [time_axis[i]]
            EventsList, x_next = ReplaceEventsHyQ(EventsList,gait,numberOfLegs,Tf,Tg,Td,x_0)
            w_index[1] += 1
    elif old_sign_u2 == -1:
        if SignCheck_u2 != 0:
            print('Leg 2 lift-off',time_axis[i])
            x_0[5] = [time_axis[i]]
            EventsList[w_index[1]][5] = time_axis[i]
    old_sign_u2 = sign_u2
    
    
    # Third oscillator solution
    r = ode(f1).set_integrator('vode', method='adams',
                              order=10, atol=1e-6,
                              with_jacobian=False)             
    r.set_initial_value(u0_3, time_axis[i]).set_f_params(w[i][2])
    u = [];  t = []; y =[];
    while r.successful() and r.t < time_axis[i+1]:
        r.integrate(r.t + ts)
        u.append(r.y);  t.append(r.t)
        #r.set_f_params(w[len(t) - 1][2])
    u3 = np.vstack((u3,u[len(u) - 1]))
    u0_3 = u3[len(u3) - 1]
    sign_u3 = np.sign(u0_3[1])
    SignCheck_u3 = old_sign_u3 - sign_u3
    if old_sign_u3 == 1:
        if SignCheck_u3 != 0:
            print('Leg 3 Touchdown',time_axis[i])
            x_0[2] = [time_axis[i]]
            EventsList[w_index[2]][2] = time_axis[i]
            EventsList, x_next = ReplaceEventsHyQ(EventsList,gait,numberOfLegs,Tf,Tg,Td,x_0)
            w_index[2] += 1
    elif old_sign_u3 == -1:
        if SignCheck_u3 != 0:
            print('Leg 3 lift-off',time_axis[i])
            x_0[6] = [time_axis[i]]
            EventsList[w_index[2]][6] = time_axis[i]
    old_sign_u3 = sign_u3
    
    
    # Fourth oscillator solution
    r = ode(f1).set_integrator('vode', method='adams',
                              order=10, atol=1e-6,
                              with_jacobian=False)             
    r.set_initial_value(u0_4, time_axis[i]).set_f_params(w[i][3])
    u = [];  t = []; y =[];
    while r.successful() and r.t < time_axis[i+1]:
        r.integrate(r.t + ts)
        u.append(r.y);  t.append(r.t)
        #r.set_f_params(w[len(t) - 1][3])
    u4 = np.vstack((u4,u[len(u) - 1]))
    u0_4 = u4[len(u4) - 1]
    sign_u4 = np.sign(u0_4[1])
    SignCheck_u4 = old_sign_u4 - sign_u4
    if old_sign_u4 == 1:
        if SignCheck_u4 != 0:
            print('Leg 4 Touchdown',time_axis[i])
            x_0[3] = [time_axis[i]]
            EventsList[w_index[3]][3] = time_axis[i]
            EventsList, x_next = ReplaceEventsHyQ(EventsList,gait,numberOfLegs,Tf,Tg,Td,x_0)
            w_index[3] += 1
    elif old_sign_u4 == -1:
        if SignCheck_u4 != 0:
            print('Leg 4 lift-off',time_axis[i])
            x_0[7] = [time_axis[i]]
            EventsList[w_index[3]][7] = time_axis[i]
    old_sign_u4 = sign_u4
    
    
#%%           
z_matrix = [[row[1] for row in u1],[row[1] for row in u2],[row[1] for row in u3],[row[1] for row in u4]]

l_matrix = [[0 for i in range(0,numberOfLegs)] for j in range(0,len(time_axis))]

# Get variables for synchronization plot
for i in range(0,len(time_axis)):
    for j in range(0,numberOfLegs):
        if z_matrix[j][i] < 0:
            l_matrix[i][j] = 0.5
            
        else:
            l_matrix[i][j] = 'nan'
 
# Compute duty factor
old_sign = [np.sign(z_matrix[0][0]),np.sign(z_matrix[1][0]),np.sign(z_matrix[2][0]),np.sign(z_matrix[3][0])]
current_sign = [0 for i in range(0,numberOfLegs)]
flag = [0 for i in range(0,numberOfLegs)]
init_time = [time_axis[0] for i in range(0,numberOfLegs)]
initial_time_index = [0 for i in range(0,numberOfLegs)]
Real_lift_off = [0 for i in range(0,numberOfLegs)]
Real_lift_off_index = [0 for i in range(0,numberOfLegs)]
Real_touchdown = [0 for i in range(0,numberOfLegs)]
Real_touchdown_index = [0 for i in range(0,numberOfLegs)]
Df_matrix = []
Df_matrix_time = []
Df_vector = []
Df_time =[]

for j in range(0,numberOfLegs):
    for i in range(0,len(z_matrix[j])):
        current_sign[j] = np.sign(z_matrix[j][i])
        if current_sign[j] != old_sign[j] and flag[j] == 0:
            Real_lift_off[j] = time_axis[i]
            Real_lift_off_index[j] = i
            old_sign[j] = current_sign[j]
            flag[j] += 1
        elif current_sign[j] != old_sign[j] and flag[j] == 1:
            Real_touchdown[j] = time_axis[i]
            Real_touchdown_index[j] = i
            Real_Df = (Real_lift_off[j] - init_time[j])/(Real_touchdown[j] - init_time[j])
            Df_vector.append(Real_Df)
            Df_time.append(Real_touchdown[j])
            old_sign[j] = current_sign[j]
            flag[j] = 0
            init_time[j] = Real_touchdown[j]  
            initial_time_index[j] = Real_touchdown_index[j]
    Df_matrix.append(Df_vector)
    Df_matrix_time.append(Df_time)
    Df_vector = []
    Df_time =[]
        
rc('text', usetex=True,)       
rc('font', family='sans-serif')    


 
# Duty factor plot
Duty_tick = [0,0.5,1]
fig6, axarr = plt.subplots(4, sharex=True)
axarr[0].tick_params(labelsize=30)
axarr[0].plot(time_axis,[row[0] for row in l_matrix],color = '0.85',linewidth=130,solid_capstyle="butt")
axarr[0].plot(time_axis,np.ones(len(time_axis))*-1,color= '0.85',linewidth=5,label="Leg in stance")
axarr[0].step(Df_matrix_time[0],Df_matrix[0],label='Real $D_f$')
axarr[0].plot(time_axis,Df_desired_vector,linestyle = "dashed",label='Desired $D_f$')
axarr[0].set_ylim([0, 1])
axarr[0].set_xlim([0, total_time])
axarr[0].set_yticks(Duty_tick)
axarr[0].set_ylabel(r"$D_f$ LF[-]",fontsize = 30)
axarr[0].tick_params(labelsize=30)
axarr[0].set_title("Duty factor plot",fontsize = 30)
axarr[0].legend(loc='lower right', shadow=False,fontsize = 15)


axarr[1].plot(time_axis,[row[1] for row in l_matrix],color = '0.85',linewidth=130,solid_capstyle="butt")
axarr[1].plot(time_axis,np.ones(len(time_axis))*-1,color= '0.85',linewidth=5,label="Leg in stance")
axarr[1].step(Df_matrix_time[1],Df_matrix[1],label='Real $D_f$')
axarr[1].plot(time_axis,Df_desired_vector,linestyle = "dashed",label='Desired $D_f$')
axarr[1].set_ylim([0, 1])
axarr[1].set_xlim([0, total_time])
axarr[1].set_yticks(Duty_tick)
axarr[1].set_ylabel(r"$D_f$ RF[-]",fontsize = 30)
axarr[1].tick_params(labelsize=30)
axarr[1].legend(loc='lower right', shadow=True,fontsize = 15)


axarr[2].plot(time_axis,[row[2] for row in l_matrix],color = '0.85',linewidth=130,solid_capstyle="butt")
axarr[2].plot(time_axis,np.ones(len(time_axis))*-1,color= '0.85',linewidth=5,label="Leg in stance")
axarr[2].step(Df_matrix_time[2],Df_matrix[2],label='Real $D_f$')
axarr[2].plot(time_axis,Df_desired_vector,linestyle = "dashed",label='Desired $D_f$')
axarr[2].set_ylim([0, 1])
axarr[2].set_xlim([0, total_time])
axarr[2].set_yticks(Duty_tick)
axarr[2].set_ylabel(r"$D_f$ LH[-]",fontsize = 30)
axarr[2].tick_params(labelsize=30)
axarr[2].legend(loc='lower right', shadow=True,fontsize = 15)


axarr[3].plot(time_axis,[row[3] for row in l_matrix],color = '0.85',linewidth=130,solid_capstyle="butt")
axarr[3].plot(time_axis,np.ones(len(time_axis))*-1,color= '0.85',linewidth=5,label="Leg in stance")
axarr[3].step(Df_matrix_time[3],Df_matrix[3],label='Real $D_f$')
axarr[3].plot(time_axis,Df_desired_vector,linestyle = "dashed",label='Desired $D_f$')
axarr[3].set_ylim([0, 1])
axarr[3].set_xlim([0, total_time])
axarr[3].set_yticks(Duty_tick)
axarr[3].set_ylabel(r"$D_f$ RH[-]",fontsize = 30)
axarr[3].tick_params(labelsize=30)
axarr[3].set_xlabel("Time [s]",fontsize = 30)
axarr[3].legend(loc='lower right', shadow=True,fontsize = 15)
         

# Synchronization plot           
for i in range(0,len(time_axis)):
    for j in range(0,numberOfLegs):
        if z_matrix[j][i] < 0:
            l_matrix[i][j] = j + 1
            
        else:
            l_matrix[i][j] = 'nan'



#%%

    
fig2, axarr = plt.subplots(2, sharex=True)
axarr[0].set_title('Variation of z coordinate with respect to time ',fontsize=30)
axarr[0].tick_params(labelsize=25)
axarr[0].plot(time_axis,[row[1] for row in u2],linewidth=1,label = "RF")
axarr[0].plot(time_axis,[row[1] for row in u1],linewidth=1,label = "LF")
axarr[0].plot(time_axis,[row[1] for row in u3],linewidth=1,label = "LH")
axarr[0].plot(time_axis,[row[1] for row in u4],linewidth=1,label = "RH")
axarr[0].axhline(y=0,color='0')
axarr[0].set_ylabel('$z[-]$',fontsize = 30)
axarr[0].legend(loc='lower right', shadow=True,fontsize = 15)
#==============================================================================
# for i in range(0, len(EventsList)):
#     axarr[0].axvline(x = EventsList[i][5], linestyle = "dashed",linewidth = 2)
#     axarr[0].axvline(x = EventsList[i][1], linestyle = "dotted")
# axarr[0].axvline(x = EventsList[1][5], linestyle = "dashed",linewidth = 2,label = "Lift-off times")
# axarr[0].axvline(x = EventsList[1][1], linestyle = "dotted", label = "Touchdown times")
# axarr[0].legend(loc='upper right', shadow=False,fontsize=20)
#==============================================================================

axarr[1].set_title('Angular frequency with respect to time',fontsize=30)
axarr[1].tick_params(labelsize=25)
axarr[1].plot(time_axis,[row[0] for row in w],label = "LF")
axarr[1].plot(time_axis,[row[1] for row in w],label = "RF")
axarr[1].plot(time_axis,[row[2] for row in w],label = "LH")
axarr[1].plot(time_axis,[row[3] for row in w],label = "RH")
axarr[1].axhline(y=0,color='0')
axarr[1].set_ylabel('$\omega [ rad/s ]$',fontsize = 30)
axarr[1].set_xlabel('Time [s]',fontsize = 30)
axarr[1].legend(loc='lower right', shadow=True,fontsize = 15)
#==============================================================================
# for i in range(0, len(EventsList)):
#     axarr[1].axvline(x = EventsList[i][5], linestyle = "dashed",linewidth = 2)
#     axarr[1].axvline(x = EventsList[i][1], linestyle = "dotted")
# axarr[1].axvline(x = EventsList[1][5], linestyle = "dashed",linewidth = 2,label = "Lift-off times")
# axarr[1].axvline(x = EventsList[1][1], linestyle = "dotted", label = "Touchdown times")
# axarr[1].legend(loc='upper right', shadow=False,fontsize=20)
#==============================================================================
    
    
# First set up the figure, the axis, and the plot element we want to animate
Writer = animation.writers['ffmpeg'] # Set to record
writer = Writer(fps=40, metadata=dict(artist='Me'), bitrate=900) # Set to record
fig4 = plt.figure()
ax = plt.axes(xlim=(-1, 2.5), ylim=(-1.5, 1.5))
plt.ylabel('z')
plt.xlabel('x')

line, = ax.plot([], [], lw=1, ls = '--')
line1, = ax.plot([],[], lw =7,color = '0.5',label = 'Left')
line2, = ax.plot([], [], lw=1, ls = '--')
line3, = ax.plot([],[], lw =7,color = 'red',label = 'Right')
line4, = ax.plot([], [], lw=1, ls = '--')
line5, = ax.plot([],[], lw =7,color = '0.5')
line6, = ax.plot([], [], lw=1, ls = '--')
line7, = ax.plot([],[], lw =7,color = 'red')

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

fig4.savefig('Legs.png')
# animation function.  This is called sequentially
i = 0

x0,y0,x1,y1,x2,y2,x3,y3 = [],[],[],[],[],[],[],[]

def animate(i):
    time_text.set_text(time_template % (i*Ts))
    x0.append(u1[i][0])
    y0.append(u1[i][1])
    dotx0 = [1.5 + u1[i-1][0],1.5 + u1[i][0]]
    doty0 = [u1[i-1][1], u1[i][1]]
    x1.append(u2[i][0])
    y1.append(u2[i][1])
    dotx1 = [1.5 + u2[i-1][0],1.5 + u2[i][0]]
    doty1 = [u2[i-1][1], u2[i][1]]
    x2.append(1.5 + u3[i][0])
    y2.append(u3[i][1])
    dotx2 = [u3[i-1][0],u3[i][0]]
    doty2 = [u3[i-1][1], u3[i][1]]
    x3.append(1.5 + u4[i][0])
    y3.append(u4[i][1])
    dotx3 = [u4[i-1][0],u4[i][0]]
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
anim = animation.FuncAnimation(fig4, animate, np.arange(1, len(u1)), init_func=init,
                               interval=Ts, blit=False)
                              
#==============================================================================
# anim.save('Legs.mp4', writer=writer)
#==============================================================================

