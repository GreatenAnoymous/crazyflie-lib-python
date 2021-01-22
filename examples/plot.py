import matplotlib.pyplot as plt

import numpy as np

import pandas as pd

np_data=np.loadtxt('./data.txt')

t=np.arange(0,np_data.shape[0])
omega=[np_data[i,4] for i in t]
omega_des=[np_data[i,1] for i in t]
r=[np_data[i,3] for i in t]
y=[np_data[i,5] for i in t]
df = pd.DataFrame(data=np_data, columns=["rateRoll_des","ratePitch_des","rateYaw_des","rateRoll","ratePitch","rateYaw"])




l1,=plt.plot(t/100,omega)
l2,=plt.plot(t/100,omega_des)
l3,=plt.plot(t/100,r)
l4,=plt.plot(t/100,y)
plt.legend(handles=[l1,l2,l3,l4],labels=['pitchRate','desired pitchRate','rollRate','yawRate'])
plt.xlabel("t")
plt.ylabel("angular rate")
plt.xlim(0,2.0)
plt.show()


plt.xlabel("t")
'''
motor_data=np.loadtxt('./motor_data.txt')
t=np.arange(0,motor_data.shape[0])
motor1=[motor_data[i,0] for i in t]
motor2=[motor_data[i,1] for i in t]
motor3=[motor_data[i,2] for i in t]
motor4=[motor_data[i,3] for i in t]
l2=len(motor1)
l1=len(omega)
print(l1,l2)

if l1<l2:
    motor1.pop(0)
    motor2.pop(1)
    motor3.pop(2)
    motor4.pop(3)
else:
    if l1>l2:
        motor1.append(motor1[-1])
        motor2.append(motor2[-1])
        motor3.append(motor3[-1])
        motor4.append(motor4[-1])

t=np.arange(0,np_data.shape[0])
df['m1']=motor1
df['m2']=motor2
df['m3']=motor3
df['m4']=motor4

'''

rpy_data=np.loadtxt('./rpy_data.txt')
t=np.arange(0,rpy_data.shape[0])
roll=[rpy_data[i,0] for i in t]
pitch=[rpy_data[i,1] for i in t]
yaw=[rpy_data[i,2] for i in t]
roll_d=[rpy_data[i,3] for i in t]
pitch_d=[rpy_data[i,4] for i in t]
yaw_d=[rpy_data[i,5] for i in t]

l2=len(roll)
l1=len(omega)

if l1<l2:
    roll.pop(0)
    pitch.pop(0)
    yaw.pop(0)
    roll_d.pop(0)
    pitch_d.pop(0)
    yaw_d.pop(0)
else:
    if l1>l2:
        roll.append(roll[-1])
        pitch.append(pitch[-1])
        yaw.append(yaw[-1])
        roll_d.append(roll_d[-1])
        pitch_d.append(pitch_d[-1])
        yaw_d.append(yaw_d[-1])

df['roll']=roll
df['pitch']=pitch
df['yaw']=yaw
df['roll_d']=roll_d
df['pitch_d']=pitch_d
df['yaw_d']=yaw_d


cmd_data=np.loadtxt('./cmd_data.txt')
t=np.arange(0,cmd_data.shape[0])
cmd_thrust=[cmd_data[i,0] for i in t]
cmd_roll=[cmd_data[i,1] for i in t]
cmd_pitch=[cmd_data[i,2] for i in t]
cmd_yaw=[cmd_data[i,3] for i in t]
l2=len(cmd_thrust)
l1=len(omega)

if l1<l2:
    cmd_thrust.pop(0)
    cmd_roll.pop(1)
    cmd_pitch.pop(2)
    cmd_yaw.pop(3)
else:
    if l1>l2:
        cmd_thrust.append(cmd_thrust[-1])
        cmd_roll.append(cmd_roll[-1])
        cmd_pitch.append(cmd_pitch[-1])
        cmd_yaw.append(cmd_yaw[-1])

t=np.arange(0,np_data.shape[0])
df['T']=cmd_thrust
df['R']=cmd_roll
df['P']=cmd_pitch
df['Y']=cmd_yaw

'''
l1,=plt.plot(t/100,motor1)
l2,=plt.plot(t/100,motor2)
l3,=plt.plot(t/100,motor3)
l4,=plt.plot(t/100,motor4)
plt.legend(handles=[l1,l2,l3,l4],labels=['motor1','motor2','motor3','motor4'])
plt.xlabel("t")
plt.ylabel("motor thrust")
plt.show()
'''



vel_data=np.loadtxt('./vel_data.txt')
t=np.arange(0,vel_data.shape[0])
vx=[vel_data[i,0]*0.001 for i in t]
vy=[vel_data[i,1]*0.001 for i in t]
vz=[vel_data[i,2]*0.001 for i in t]
z=[vel_data[i,3]*0.001 for i in t]

l2=len(vx)
l1=len(omega)

if l1<l2:
    vx.pop(0)
    vy.pop(0)
    vz.pop(0)
    z.pop(0)
    #pitch_d.pop(0)
    #yaw_d.pop(0)
else:
    if l1>l2:
        vx.append(vx[-1])
        vy.append(vy[-1])
        vz.append(vz[-1])
        z.append(z[-1])
        #roll_d(roll_d[-1])
        #pitch_d(pitch_d[-1])
        #yaw_d(yaw_d[-1])

df['vx']=vx
df['vy']=vy
df['vz']=vz
df['z']=z


#df = pd.DataFrame(data=np_data, columns=["pitch","ratePitch_des","rateRoll","ratePitch","rateYaw"])
t=np.arange(0,len(vx))
l1,=plt.plot(t/100,vx)
l2,=plt.plot(t/100,vy)
l3,=plt.plot(t/100,vz)
l4,=plt.plot(t/100,z)
plt.legend(handles=[l1,l2,l3,l4],labels=['vx','vy','vz','z'])
plt.xlabel("t")
plt.ylabel("angular rate")
plt.show()
df.to_csv('data.csv')
