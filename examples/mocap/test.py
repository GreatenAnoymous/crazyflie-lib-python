import rospy
import logging
import time
from geometry_msgs.msg import *

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
#from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/E7E7E7E7E8'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

cf=Crazyflie()

mocap_topic="/mocap_node/cf2/pose"
x=0
y=0
z=0
def mocap_callback(data):
    x=data.pose.position.x
    y=data.pose.position.y
    z=data.pose.position.z
    cf.extpos.send_extpos(x,y,z)
    print(x,y,z)
   
    
def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    rospy.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    rospy.sleep(0.1)
    
def optitrack():
    rospy.init_node('optitrack_node')
    rospy.Subscriber(mocap_topic,PoseStamped, mocap_callback)
    
if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf.open_link(URI)
    
    time.sleep(7) #wait for initilization
    optitrack()
    rospy.sleep(1)
    reset_estimator(cf)
    
    print("all good!")
    
    t=3
    dt=0.01
    n=int(t/dt)
    for i in range(0,n):
        cf.commander.send_position_setpoint(0,0,i*0.5/n,0)
        rospy.sleep(dt)
        
    for i in range(0,2*n):
        cf.commander.send_position_setpoint(0,0,0.5,0)
        rospy.sleep(dt)
        
    for i in range(0,2*n):
        cf.commander.send_position_setpoint(0.25*i/n,0,0.5,0)
        rospy.sleep(dt)
        
    for i in range(0,n):
        cf.commander.send_position_setpoint(0.5,0,0.5-0.5*i/n,0)
        rospy.sleep(dt)
    cf.commander.send_stop_setpoint()
    cf.close_link()
    
