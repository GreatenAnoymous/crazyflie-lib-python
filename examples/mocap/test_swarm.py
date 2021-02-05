import rospy
import logging
import time
from geometry_msgs.msg import *

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
#from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M/E7E7E7E7E8'
URI2= 'radio://0/80/2M/E7E7E7E7E7'
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

cf1=Crazyflie()
cf2=Crazyflie()

mocap_topic1="/mocap_node/cf1/pose"
mocap_topic2="/mocap_node/cf2/pose"

x1=0
y1=0
z1=0

x2=0
y2=0
z2=0

def mocap_callback(data):
    x1=data.pose.position.x
    y1=data.pose.position.y
    z1=data.pose.position.z
    cf1.extpos.send_extpos(x1,y1,z1)
    #print(x,y,z)
def mocap_callback(data):
    x2=data.pose.position.x
    y2=data.pose.position.y
    z2=data.pose.position.z
    cf2.extpos.send_extpos(x2,y2,z2)   
    
def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    rospy.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    rospy.sleep(0.1)
    
def optitrack():
    rospy.init_node('optitrack_node')
    rospy.Subscriber(mocap_topic1,PoseStamped, mocap_callback)
    rospy.Subscriber(mocap_topic2,PoseStamped, mocap_callback)
    
if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf1.open_link(URI)
    cf2.open_link(URI2)
    
    time.sleep(7) #wait for initilization
    optitrack()
    rospy.sleep(1)
    reset_estimator(cf1)
    reset_estimator(cf2)
    pose_init1=[x1,y1,z1]
    pose_init2=[x2,y2,z2]
    print("all good!")
    
    t=3
    dt=0.01
    n=int(t/dt)
    for i in range(0,n):
        cf1.commander.send_position_setpoint(0+pose_init1[0],pose_init1[1],pose_init1[2]+i*0.5/n,0)
        cf2.commander.send_position_setpoint(0+pose_init2[0],pose_init2[1],pose_init2[2]+i*0.5/n,0)
        rospy.sleep(dt)
        
    for i in range(0,2*n):
        cf1.commander.send_position_setpoint(0+pose_init1[0],0+pose_init1[1],0.5+pose_init1[2],0)
        cf2.commander.send_position_setpoint(0+pose_init2[0],0+pose_init2[1],0.5+pose_init2[2],0)
        rospy.sleep(dt)
        
    for i in range(0,2*n):
        cf1.commander.send_position_setpoint(0.25*i/n+pose_init1[0],0+pose_init1[1],0.5+pose_init1[2],0)
        cf2.commander.send_position_setpoint(0.25*i/n+pose_init2[0],0+pose_init2[1],0.5+pose_init2[2],0)
        rospy.sleep(dt)
        
    for i in range(0,n):
        cf1.commander.send_position_setpoint(0.5+pose_init1[0],0+pose_init1[1],0.5-0.5*i/n+pose_init1[2],0)
        cf2.commander.send_position_setpoint(0.5+pose_init2[0],0+pose_init2[1],0.5-0.5*i/n+pose_init2[2],0)
        rospy.sleep(dt)
    cf1.commander.send_stop_setpoint()
    cf1.close_link()
    cf2.commander.send_stop_setpoint()
    cf2.close_link()
    
