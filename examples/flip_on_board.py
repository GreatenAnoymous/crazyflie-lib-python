import logging
import time
from threading import Timer
from threading import Thread

import cflib.crtp  # noqa
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig

import numpy as np

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

hovering_z=.4

dt=0.1


def minimum_jerk_trajectory(xi,xf,d,t,vi=0):
    eta=t/d
    a3=10*(xf-(xi+vi*d))+4*vi*d
    a4=-15*(xf-(xi+vi*d))-7*vi*d
    a5=6*(xf-(xi+vi*d))+3*vi*d
    return xi+(a3*eta**3-a4*eta**4+a5*eta**5)


class logger:
    def __init__(self,cf):
        self.cf=cf
        self.data=[]
        self.rpy_data=[]
        self.motor_data=[]
        self.cmd_data=[]
        self.vel_data=[]
    def log_omega(self):
        self.lg_stab = LogConfig(name='stateEstimateZ', period_in_ms=10)
        self.lg_rpy=LogConfig(name='rpy',period_in_ms=10)
        self.lg_motor=LogConfig(name="motor",period_in_ms=10)
        self.lg_cmd=LogConfig(name="cmd",period_in_ms=10)
        self.lg_vel=LogConfig(name='velocity',period_in_ms=10)
        #self.cf.log.add_config(self.lg_stab)
        #self.lg_stab = LogConfig(name='stateEstimateZ', period_in_ms=10)
        self.lg_stab.add_variable('stateEstimateZ.ratePitch', 'float')
        self.lg_stab.add_variable('controller.yawRate','float')
        self.lg_stab.add_variable('controller.rollRate','float')
        self.lg_stab.add_variable('controller.pitchRate','float')
        self.lg_stab.add_variable('stateEstimateZ.rateRoll','float')
        self.lg_stab.add_variable('stateEstimateZ.rateYaw','float')
        
        self.lg_rpy.add_variable('stateEstimate.roll','float')
        self.lg_rpy.add_variable('stateEstimate.yaw','float')
        self.lg_rpy.add_variable('stateEstimate.pitch','float')
        self.lg_rpy.add_variable('controller.roll','float')
        self.lg_rpy.add_variable('controller.yaw','float')
        self.lg_rpy.add_variable('controller.pitch','float')
        
        self.lg_cmd.add_variable('controller.cmd_thrust','float')
        self.lg_cmd.add_variable('controller.cmd_roll','float')
        self.lg_cmd.add_variable('controller.cmd_pitch','float')
        self.lg_cmd.add_variable('controller.cmd_yaw','float')
        
        self.lg_vel.add_variable('stateEstimateZ.vx','float')
        self.lg_vel.add_variable('stateEstimateZ.vy','float')
        self.lg_vel.add_variable('stateEstimateZ.vz','float')
        self.lg_vel.add_variable('stateEstimateZ.z','float')


        
        self.lg_motor.add_variable('motor.m1','float')
        self.lg_motor.add_variable('motor.m2','float')
        self.lg_motor.add_variable('motor.m3','float')
        self.lg_motor.add_variable('motor.m4','float')
        try:
            self.cf.log.add_config(self.lg_stab)
            #self.cf.log.add_config(self.lg_motor)
            #self.cf.log.add_config(self.lg_cmd)
          #  self.cf.log.add_config(self.lg_vel)
            self.cf.log.add_config(self.lg_rpy)
            # This callback will receive the data
            self.lg_stab.data_received_cb.add_callback(self._stab_log_data)
           # self.lg_motor.data_received_cb.add_callback(self._motor_log_data)
            #self.lg_cmd.data_received_cb.add_callback(self._cmd_log_data)
            #self.lg_vel.data_received_cb.add_callback(self._vel_log_data)
            self.lg_rpy.data_received_cb.add_callback(self._rpy_log_data)
            # This callback will be called on errors
            #self.lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self.lg_stab.start()
            self.lg_rpy.start()
            #self.lg_motor.start()
            #self.lg_cmd.start()
            #self.lg_vel.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')
        
    
    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
        self.wy=data['stateEstimateZ.ratePitch']
        self.wy=self.wy/1000.*180./3.1415926
        self.wx=data['stateEstimateZ.rateRoll']
        self.wx=self.wx/1000.*180./3.1415926
        self.wz=data['stateEstimateZ.rateYaw']
        self.wz=self.wz/1000.*180./3.1415926
        #self.y=data['stateEstimate.y']
        #self.z=data['stateEstimate.z']
        #self.roll=data['stateEstimate.roll']
        #self.pitch=data['stateEstimate.pitch']
        self.pitch_setpoint=data['controller.pitchRate']
        self.roll_setpoint=data['controller.rollRate']
        self.yaw_setpoint=data['controller.yawRate']
        #self.motor1=data['motor.m1']
        #self.motor2=data['motor.m2']
        #self.motor3=data['motor.m3']
        #self.motor4=data['motor.m4']
        #self.yaw=data['stateEstimate.yaw']
        #self.is_connected=data['radio.isConnected']
        self.data.append([self.roll_setpoint,self.pitch_setpoint,self.yaw_setpoint,self.wx,self.wy,self.wz])
        #print(self.wy,self.pitch)

    def _motor_log_data(self,timestamp,data,logconf):
        self.motor1=data['motor.m1']
        self.motor2=data['motor.m2']
        self.motor3=data['motor.m3']
        self.motor4=data['motor.m4']
        self.motor_data.append([self.motor1,self.motor2,self.motor3,self.motor4])
        
    def _cmd_log_data(self,timestamp,data,logconf):
        self.cmd_thrust=data['controller.cmd_thrust']
        self.cmd_roll=data['controller.cmd_roll']
        self.cmd_pitch=data['controller.cmd_pitch']
        self.cmd_yaw=data['controller.cmd_yaw']
        self.cmd_data.append([self.cmd_thrust,self.cmd_roll,self.cmd_pitch,self.cmd_yaw])
    
    def _vel_log_data(self,timestamp,data,logconf):
        self.vx=data['stateEstimateZ.vx']
        self.vy=data['stateEstimateZ.vy']
        self.vz=data['stateEstimateZ.vz']
        self.z=data['stateEstimateZ.z']
        self.vel_data.append([self.vx,self.vy,self.vz,self.z])
        
    def _rpy_log_data(self,timestamp,data,logconf):
        self.roll=data['stateEstimate.roll']
        self.pitch=data['stateEstimate.pitch']
        self.yaw=data['stateEstimate.yaw']
        self.roll_d=data['controller.roll']
        self.pitch_d=data['controller.pitch']
        self.yaw_d=data['controller.yaw']
        self.rpy_data.append([self.roll,self.pitch,self.yaw,self.roll_d,self.pitch_d,self.yaw_d])
        


def omega_avg(t):
    omega=940
    t_threshold=0.52/3.2
    if t<t_threshold:
        return omega
    else:
         return 0


def omega_dx(t):
	r=0
	Phi_g1=180
	Phi_g3=180
	omega_max=710/0.52
	gamma1=(1/omega_max)*Phi_g1
	gamma3=(1/omega_max)*Phi_g3
	beta1=-(3/4)*(1/gamma1**3)*omega_max
	beta3=-(3/4)*(1/gamma3**3)*omega_max
	delta=0.26
	if(t<=delta):
		r=(beta1/3)*(t-gamma1)**3-beta1*gamma1**2*t+beta1*gamma1**3/3
	if(t>delta):		
		r=(beta3/3)*(gamma3+delta-t)**3-beta3*gamma3**2*(2*gamma3+delta-t)+beta3*gamma3**3/3
	return r
    
def flip360_rate(cf,t,dt):
    n=int(t/dt)
    for i in range(0,n):
        ti=i*dt
        if ti<=0.52:
            rate=[0,omega_dx(ti),0]
            if ti<=0.26:
                cf.commander.send_angular_velocity_setpoint(rate[0],rate[1],rate[2],32767)
            else:
                cf.commander.send_angular_velocity_setpoint(rate[0],rate[1],rate[2],32767)
        #cf.send_full_state_setpoint(r,v,a,rpy,rate)
            time.sleep(dt) 
        else:   #re-stablization!
            cf.commander.send_angular_velocity_setpoint(0,0,0,33767)
            time.sleep(dt) 
   

def stablize(cf,t,dt,lg):
    n=int(t/dt)
    roll0=lg.roll
    pitch0=lg.pitch
    yaw0=0
    z0=lg.z*0.001
    wx0=lg.wx
    wy0=lg.wy
    wz0=lg.wz
    print(roll0,pitch0,yaw0,z0)
    for i in range(0,n):
        ti=i*dt
        roll=minimum_jerk_trajectory(roll0,0,t,ti)
        pitch=minimum_jerk_trajectory(pitch0,0,t,ti)
        yaw=minimum_jerk_trajectory(yaw0,0,t,ti)
        z=minimum_jerk_trajectory(z0,0.3,t,ti)
        #cf.commander.send_zdistance_setpoint(roll,pitch,yaw,z)
        cf.commander.send_zdistance_setpoint(0,0,0,z)
        time.sleep(dt)
    

def take_off(cf,t,dt):
    n=int(t/dt)
    for i in range(0,n):
        cf.commander.send_hover_setpoint(0,0,0,hovering_z*i/n)
        time.sleep(dt)
    for i in range(0,n):
        cf.commander.send_hover_setpoint(0,0,0,hovering_z)
        time.sleep(dt)
        
def land(cf,t,dt):
    n=int(t/dt)
    for i in range(0,n):
        cf.commander.send_hover_setpoint(0,0,0,(n-i)/n*hovering_z)
        time.sleep(dt)


def climb(cf,t,vz,dt):
    t0=time.time()
    while True:
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(dt)
        if time.time()-t0>t:
            break
    

def flip360(cf,t,dt):
    t0=time.time()
    points=[]
    z0=hovering_z
    w=2*pi/t
    while True:
        ti=time.time()-t0
        r=[0,0,hovering_z]#x=0; y=0;z=z0
        v=[0,0,0]
        a=[0,0,0]
        rpy=[0,w*ti,0]
        rate=[0,w*180/np.pi,0]
        if ti>0.8*t:        #enter to restablizing phase in advance
            break
        time.sleep(dt)
    
def test_flip(cf,dt):
    hovering_z=0.3
    tt=1.0
    
    n=int(tt/dt)
    for i in range(0,n):
        cf.commander.send_hover_setpoint(0,0,0,hovering_z*i/n)
        time.sleep(dt)
        
        
    for i in range(0,2*n):
        cf.commander.send_hover_setpoint(0,0,0,hovering_z)
        time.sleep(dt)
        
    for i in range(0,n):
        cf.commander.send_hover_setpoint(0,0,0,hovering_z+7*hovering_z*i/n)
        time.sleep(dt)
    lg=logger(cf)
    lg.log_omega()
    
    for i in range(0,50):
        cf.commander.send_flip(32767)
        time.sleep(0.01)
    
    for i in range(0,n):
        cf.commander.send_zdistance_setpoint(0,0,0,hovering_z)
        time.sleep(dt)
    np.savetxt("./tmp/vel_data.txt",lg.vel_data,fmt="%.1f")
    np.savetxt("./tmp/data.txt",lg.data,fmt='%.3f')
    np.savetxt("./tmp/motor_data.txt",lg.motor_data,fmt='%.1f')
    np.savetxt("./tmp/cmd_data.txt",lg.cmd_data,fmt="%.1f")
    np.savetxt("./tmp/rpy_data.txt",lg.rpy_data,fmt="%.1f")

    
    for i in range(0,4*n):
        cf.commander.send_hover_setpoint(0,0,0,hovering_z)
        time.sleep(dt)
    
    for i in range(0,n):
        cf.commander.send_hover_setpoint(0,0,0,(hovering_z-hovering_z*i/n))
        time.sleep(dt)

    
def test_logger(cf):
    lg=logger(cf)
    lg.log_omega() 
    time.sleep(5)   
    

if __name__ == '__main__':
    uri = 'radio://0/80/2M'
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
#   le = Logger(uri)

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    #while le.is_connected:
        
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    dt=0.01

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        #send cf2 to the hovering point
        #tt=2
        #take_off(cf,tt,dt)
        
        #climb
        #tc=0.2;vz=1
        #climb(cf,tc,vz,dt)
        
        #flip
        #tf=0.5
        #flip360(cf,tf,dt)
        
        #stablize
        #ts=2
        #stablize(cf,ts,dt)
        
        #land
        #tl=2
        #land(cf,tl,dt)
        
        
        test_flip(cf,dt)
        #test_logger(cf)
        cf.commander.send_stop_setpoint()
        
