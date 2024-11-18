#%%
import random
import numpy as np
import csv
from scipy.io import savemat, loadmat
import matplotlib.pyplot as plt
import kinematics as kin

t = np.empty([10,4998]) #[trial,time step]
q = np.empty([10,4998,7]) #[trial,time step, joint]
qdot = np.empty_like(q)


for i in range(10):
    mat_input = loadmat(f'part3_trial0{i}.mat')
    t[i] = mat_input['t'][0]
    q[i] = mat_input['q']
    qdot[i] = mat_input['q_dot']


#Define robot parameters
pi = np.pi
dh = np.array([[pi,.27035,-.069,pi/2], #ADDED IN PI OFFSET FOR TH1 TO FLIP X AXIS TO MATCH BASE ORIENTATION
               [pi/2, 0, 0, -pi/2],
               [0,.36435,-.069,pi/2],
               [0,0,0,-pi/2],
               [0,.37429,-.01,pi/2],
               [0,0,0,-pi/2],
               [0,.229525,0,0]])

base = np.array([[.7071,.7071,0,.06353],
                 [-.7071,.7071,0,-.2597],
                 [0,0,1,.119],
                 [0,0,0,1]])

arm = kin.SerialArm(dh,base=base)

J = np.empty([10,4998,6,7])

for i in range(10):
    for j in range(len(t[0])):
        J[i,j] = arm.jacob(q[i,j]) #get J for each time step of q

vel = np.empty([10,4998,6])

for i in range(10):
    for j in range(len(t[0])):
        vel[i,j] = J[i,j] @ qdot[i,j] # get velocities for each time step of qdot

print(np.shape(vel))

# %%
avgvel = []
for i in range(4998):
    sum = 0
    for j in range(10):
        sum = sum + vel[j,i]
    avgvel.append(sum/10)

avgvel = np.array(avgvel)
print(np.shape(avgvel))


# %%
sdvel = []
for i in range(4998):
    sum = 0
    for j in range(10):
        sum = (vel[j,i]-avgvel[i])**2
    sdvel.append(np.sqrt(sum/9))

sdvel = np.array(sdvel).T #Transpose both matrices to make plotting easier
avgvel = avgvel.T
print(np.shape(sdvel))



# %%
print((avgvel[0]))
#%%
print(sdvel[0])
print(np.shape(t[0]))
#%%
fig, axs = plt.subplots(2,3, sharex='col', sharey='row')

k = 0
for i in range(2):
    for j in range(3):
        axs[i,j].plot(t[k],avgvel[k], color = 'k', linestyle = '-')
        axs[i,j].plot(t[k],sdvel[k], color = 'b', linestyle = '--')  #Add extra lines above and below to show SD
        # axs[i,j].plot(t[k],avgvel[k]+sdvel[k],color = 'b', linestyle = '--')
        k = k+1
        # if i == 0:
        #     axs[i,j].legend(['Average Linear Velocity', 'Linear Velocity Standard Deviaiton'])
        # else:
        #     axs[i,j].legend(['Average Angular Velocity', 'Angular Velocity Standard Deviaiton'])
axs[1,0].set_xlabel('Time (s) X Axis')
axs[0,0].set_ylabel('Tip Velocity (m/s)')
axs[1,1].set_xlabel('Time (s) Y Axis')
axs[1,2].set_xlabel('Time (s) Z Axis')
axs[1,0].set_ylabel('Tip Angular Velocity (rad/s)')
plt.show()


# %%
fig, axs = plt.subplots(2,3, sharex='col', sharey='row')

k = 0
for i in range(2):
    for j in range(3):
        # axs[i,j].plot(t[k],avgvel[k], color = 'k', linestyle = '-')
        axs[i,j].plot(t[k],sdvel[k], color = 'b', linestyle = '-')  #Add extra lines above and below to show SD
        # axs[i,j].plot(t[k],avgvel[k]+sdvel[k],color = 'b', linestyle = '--')
        k = k+1
        # if i == 0:
        #     axs[i,j].legend(['Average Linear Velocity', 'Linear Velocity Standard Deviaiton'])
        # else:
        #     axs[i,j].legend(['Average Angular Velocity', 'Angular Velocity Standard Deviaiton'])
axs[1,0].set_xlabel('Time (s) X Axis')
axs[0,0].set_ylabel('Tip Velocity SD (m/s)')
axs[1,1].set_xlabel('Time (s) Y Axis')
axs[1,2].set_xlabel('Time (s) Z Axis')
axs[1,0].set_ylabel('Tip Angular Velocity SD (rad/s)')
plt.show()
# %%
