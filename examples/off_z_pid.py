import numpy as np

import threading
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.utils.callbacks import Caller
from cflib.crazyflie.log import LogConfig


URI = 'radio://0/80/250K'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Called when the link is established and the TOCs (that are not
# cached) have been downloaded
connected = Caller()

cflib.crtp.init_drivers(enable_debug_driver=False)

cf = Crazyflie()


cf.open_link(URI)
time.sleep(8.0)

cf.commander.send_setpoint(0, 0, 0, 0)



def poshold(cf, t, z):
    steps = t * 10

    for r in range(steps):
        cf.commander.send_hover_setpoint(0, 0, 0, z)
        time.sleep(0.1)

class Odometry(threading.Thread):
    def __init__(self, cf):
        self.cf=cf
        super(Odometry, self).__init__()
        self.lg_pose = LogConfig(name='Stabilizer1', period_in_ms=100)
        self.lg_pose.add_variable('stateEstimate.x', 'float')
        self.lg_pose.add_variable('stateEstimate.y', 'float')
        self.lg_pose.add_variable('stateEstimate.z', 'float')
        self.lg_pose.add_variable('stateEstimate.roll', 'float')
        self.lg_pose.add_variable('stateEstimate.pitch', 'float')
        self.lg_pose.add_variable('stateEstimate.yaw', 'float')

        self.lg_velociy = LogConfig(name='Stabilizer2', period_in_ms=100)
        self.lg_velociy.add_variable('stateEstimate.vx', 'float')
        self.lg_velociy.add_variable('stateEstimate.vy', 'float')
        self.lg_velociy.add_variable('stateEstimate.vz', 'float')
        self.lg_velociy.add_variable('stateEstimateZ.ratePitch', 'float')
        self.lg_velociy.add_variable('stateEstimateZ.rateRoll', 'float')
        self.lg_velociy.add_variable('stateEstimateZ.rateYaw', 'float')

        self.x=0;self.y=0;self.z=0;self.roll=0;self.pitch=0;self.yaw=0;self.vz=0;self.vx=0;self.vy=0
        self.ratePitch=0;self.rateRoll=0;self.rateYaw=0

    def run(self):
        try:
            self.cf.log.add_config(self.lg_pose)
            self.cf.log.add_config(self.lg_velociy)
            self.is_connected=True
            self.lg_pose.data_received_cb.add_callback(self._pose_log_data)
            self.lg_velociy.data_received_cb.add_callback(self._velocity_log_data)

            self.lg_pose.start()
            self.lg_velociy.start()
     
            
        except KeyError as e:
            print('Could not start log configuration,''{} not found in TOC'.format(str(e)))
        

    def _pose_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        self.x=data['stateEstimate.x']
        self.y=data['stateEstimate.y']
        self.z=data['stateEstimate.z']
        self.roll=data['stateEstimate.roll']
        self.pitch=data['stateEstimate.y']
        self.yaw=data['stateEstimate.y']
       # self.vx=data['stateEstimate.vx']
       # self.vy=data['stateEstimate.vy']
      #  self.vz=data['stateEstimate.vz']
      #  self.wx=data['stateEstimateZ.rateRoll']
     #   self.wy=data['stateEstimateZ.ratePitch']
     #   self.wz=data['stateEstimateZ.rateYaw']
      # print("odometry z=",self.z)

    def _velocity_log_data(self,timestamp,data,logconf):
        self.vx=data['stateEstimate.vx']
        self.vy=data['stateEstimate.vy']
        self.vz=data['stateEstimate.vz']
        self.wx=data['stateEstimateZ.rateRoll']
        self.wy=data['stateEstimateZ.ratePitch']
        self.wz=data['stateEstimateZ.rateYaw']



kz=1
kvz=0.5
mass=0.027
g=9.81
k_theta=0.2


def eul2rot(theta):
    R = np.array([[np.cos(theta[1])*np.cos(theta[2]),       np.sin(theta[0])*np.sin(theta[1])*np.cos(theta[2]) - np.sin(theta[2])*np.cos(theta[0]),      np.sin(theta[1])*np.cos(theta[0])*np.cos(theta[2]) + np.sin(theta[0])*np.sin(theta[2])],
                  [np.sin(theta[2])*np.cos(theta[1]),       np.sin(theta[0])*np.sin(theta[1])*np.sin(theta[2]) + np.cos(theta[0])*np.cos(theta[2]),      np.sin(theta[1])*np.sin(theta[2])*np.cos(theta[0]) - np.sin(theta[0])*np.cos(theta[2])],
                  [-np.sin(theta[1]),                        np.sin(theta[0])*np.cos(theta[1]),                                                           np.cos(theta[0])*np.cos(theta[1])]])

    return R



class PID_controller():
    def __init__(self,cf):
        self.cf=cf
        self.odometry=Odometry(self.cf)
        self.odometry.start()
        self.rate=0.01
        self.cf.param.set_value('commander.enHighLevel', '1')


    def goto(self,rd): 
        start=time.time()
        while True:
            duration=time.time()-start
            if duration>=10:
                break
            r=np.array([self.odometry.x,self.odometry.y,self.odometry.z])
            v=np.array([self.odometry.vx,self.odometry.vy,self.odometry.vz])
            e_r=r-np.array(rd)
            e_v=v
            e_3=np.array([0,0,1])
            R=eul2rot(np.array([self.odometry.roll,self.odometry.pitch,self.odometry.yaw])*np.pi/180)
            f=np.dot(np.dot(R,np.transpose(-kz*e_r-kvz*e_v+mass*g*e_3)),e_3)
            roll_cmd=-k_theta*self.odometry.roll
            pitch_cmd=-k_theta*self.odometry.pitch
            yaw_cmd=-k_theta*self.odometry.yaw
            
            a=2.13e-11
            b=1.032e-6
            c=5.485e-4
            if f>0:
                thrust=int(f/(0.057*g)*65535)
                if thrust>=35535:
                    thrust=35534
            else:
                thrust=0
            print(e_r,e_v,f,thrust)
            self.cf.commander.send_setpoint(roll_cmd,pitch_cmd,yaw_cmd,thrust)
            time.sleep(self.rate)
        self.cf.high_level_commander.land(0.0, 1.0)


pid=PID_controller(cf)
pid.goto([0,0,0.3])
cf.close_link()

