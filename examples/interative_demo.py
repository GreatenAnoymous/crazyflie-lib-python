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
        cf.param.set_value('commander.enHighLevel', '1')
        
        
        cf.high_level_commander.takeoff(0.3,1.0)
        
        time.sleep(2)
        
        for i in range(0,300):
            cf.commander.send_velocity_world_setpoint(0,0,0,0)
            time.sleep(0.1)
            
        cf.high_level_commander.land(0.0,2.0)
