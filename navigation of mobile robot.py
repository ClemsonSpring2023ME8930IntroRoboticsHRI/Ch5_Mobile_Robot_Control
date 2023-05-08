# -*- coding: utf-8 -*-
"""
Created on Sun May  7 16:08:29 2023

@author: vvrav
"""
"""
Before Running the command it is expected from you to read through the
command syntax from python API for CoppeliaSim. 
"""
# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#import all the necessary libraries
import sim                  
import sys
import time                #used to keep track of time
import numpy as np         #array library
import math


sim.simxFinish(-1) # just in case, close all opened connections

clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:  #check if client connection successful
    print ('Connected to remote API server')
    
else:
    print ('Connection not successful')
    sys.exit('Could not connect')
    
"get all the handles for the objects required"   

errorCode, right_motor=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_blocking)
errorCode, left_motor=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_blocking)
errorCode, robot=sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_blocking)
errorCode, goal=sim.simxGetObjectHandle(clientID,'goal',sim.simx_opmode_blocking)
errorCode, obstacle=sim.simxGetObjectHandle(clientID,'obstacle',sim.simx_opmode_blocking)
t = time.time()


while (time.time()-t)<50:
    
    errorCode, theta1= sim.simxGetObjectOrientation(clientID,robot,-1,sim.simx_opmode_oneshot_wait)              # robot orientation wrt inertial frame"
    errorCode, robot_position= sim.simxGetObjectPosition(clientID,robot,-1,sim.simx_opmode_oneshot_wait)         # robot position
    errorCode, goal_position= sim.simxGetObjectPosition(clientID,goal,-1,sim.simx_opmode_oneshot_wait)           #goal position
    errorCode, obstacle_position= sim.simxGetObjectPosition(clientID,obstacle,-1,sim.simx_opmode_oneshot_wait)   #obstacle position(you can extract this information using proximity sensor also)
   
    yg=goal_position[1];xg=goal_position[0]
    yr=robot_position[1];xr=robot_position[0]
    xo=obstacle_position[0];yo=obstacle_position[1]
    
    
    k_ao=3                     #avoid obstacle gain
    a=(xr-xo)
    b=(yr-yo)
    u_ao=np.array([k_ao*a,k_ao*b])             # obstacle avoidance vector
            
    k_gtg=3
    u_gtg=np.array([k_gtg*(xg-xr),k_gtg*(yg-yr)]) 
        #go to goal vector
    k_bf=3                                     #k_bf is scaling factor alpha
    u_bf_cc=np.array([-k_bf*b,k_bf*a])     #counter clockwise vector for boundary following,
    u_bf_c=np.array([k_bf*b,-k_bf*a])      #clockwise boundary following vector
    
    u1=0
    u2=0
    
    d=math.sqrt(a**2+b**2)                  #distance from obstacle to robot
    dg=math.sqrt((xr-xg)**2+(yr-yg)**2)     #distance from goal to robot
  

    if d>9:
        u1=u_gtg[0]               #gaurd conditions required for go-to-goal  
        u2=u_gtg[1]
        
    if d<9 and d>6:
        if np.dot(u_bf_c,u_gtg)>0:
            u1=u_bf_c[0]
            u2=u_bf_c[1]
        else:
            u1=u_bf_cc[0]                            #gaurd conditions required for boundary following 
            u2=u_bf_cc[1]
                

    if d<6:
        u1=u_ao[0]
        u2=u_ao[1]                #gaurd conditions required for avoid obstacle  
        print("AO")
    
    
    
    if dg<0.5:
       vr=0
       vl=0
    else:
        theta_d=math.atan2(u2, u1)        #desired angle for robot to follow
        e=theta_d-theta1[2]               #error in orientation and desired angle
        vo=math.sqrt(u1**2+u2**2)         #velocity of robot
        k=10    
         
        w=k*math.atan2(math.sin(e),math.cos(e))   #angular velocity of robot
        
        r=0.098                                   #radius of pioneer_3dx wheel
        l=0.4                                     #track width of pioneer_3dx robot
        
        vr=(2*vo+w*l)/2*r                        #right wheel velocity 
        vl=(2*vo-w*l)/2*r                        #left wheel velocity
        
        errorCode=sim.simxSetJointTargetVelocity(clientID, left_motor,vl,sim.simx_opmode_streaming)
        errorCode=sim.simxSetJointTargetVelocity(clientID, right_motor,vr,sim.simx_opmode_streaming)
        time.sleep(0.01)     

errorCode=sim.simxSetJointTargetVelocity(clientID, left_motor,0,sim.simx_opmode_streaming)
errorCode=sim.simxSetJointTargetVelocity(clientID, right_motor,0,sim.simx_opmode_streaming)   
    
sim.simxFinish(clientID)    
        
    
    
    

    


   