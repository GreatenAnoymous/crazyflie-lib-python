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


def test_yaw(cf,dt):
    ts=2.0
    hovering_z=0.3
    n=int(ts/dt)
    for i in range(0,n):
        cf.commander.send_hover_setpoint(0,0,0,hovering_z*i/n)
        time.sleep(dt)
        
    yaw_rate=20
    tc=10
    n=int(tc/dt)
    yaw=0
    for i in range(0,n):
        yaw=yaw+yaw_rate*dt
        cf.commander.send_hover_setpoint(0,0,yaw_rate,hovering_z)
       # cf.commander.send_position_setpoint(0,0,hovering_z,yaw)
        time.sleep(dt)
    n=int(ts/dt)
    for i in range(0,n):
        cf.commander.send_hover_setpoint(0,0,0,hovering_z)
        #cf.commander.send_position_setpoint(0,0,hovering_z,yaw)
        time.sleep(dt)
    
    for i in range(0,n):
        cf.commander.send_hover_setpoint(0,0,0,hovering_z-hovering_z*i/n)
        #cf.commander.send_position_setpoint(0,0,hovering_z-hovering_z*i/n,0)
        time.sleep(dt)
    

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
        
        
        test_yaw(cf,dt)
        #test_logger(cf)
        cf.commander.send_stop_setpoint()
        
