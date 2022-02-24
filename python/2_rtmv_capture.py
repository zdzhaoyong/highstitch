#! /usr/bin/python3
import svar
import cv2
import numpy as np
import time
import os
import math
import argparse
from mission import *  

parser = argparse.ArgumentParser()
parser.add_argument("-height",default=300,help="The target mission height")
parser.add_argument("-h_min",default=100,help="The minimum height to start recording")
parser.add_argument("-v_max",default=15,help="The max flight velocity")
parser.add_argument("-cam",default="[1920,1440,3350.778698,3350.778698,960,720]",help="The camera parameters in json array")
parser.add_argument("-intra_num",default=6,help="how many images to take in a half cycle")
parser.add_argument("-airline_direction",default=0,help="the airline direction in degree")
parser.add_argument("-intra_overlap",default=0.6,help="the intra overlap rate")
parser.add_argument("-forward_overlap",default=0.6,help="the forward overlap rate")
parser.add_argument("-side_overlap",default=0.8,type=float,help="the side overlap rate")
  
args = parser.parse_args()

# 1. perform mission planning
camera    =json.loads(args.cam)
fov_width =math.atan2(camera[0]/2,camera[2])*360/3.1415926
fov_height=math.atan2(camera[1]/2,camera[3])*360/3.1415926
mission_settings={"mission_intra_number":args.intra_num,
                  "mission_height":args.height,
                  "fov_width":fov_width,"fov_height":fov_height,
                  "mission_intra_overlap":args.intra_overlap,
                  "mission_forward_overlap":args.forward_overlap,
                  "mission_side_overlap":args.side_overlap,
                  "mission_airline_direction":args.airline_direction/3.1415926*180
                  }

gimbal_planner=GimbalPlanner(mission_settings)

# 2. recording rtmv
height=0
rtmv=open(str(time.time())+".rtmv",'wb',buffering=0)

def callback_buf(buf):
  mem=memoryview(buf)
  if height > args.h_min:
    rtmv.write(mem.tobytes())# only record height above h_min

def rad2degree(r):
  return 180/3.1415926*r

def degree2rad(d):
  return 3.1415926/180.*d

osdk=svar.load('svar_osdk')
env=osdk.LinuxSetup(True)
vehicle = env.getVehicle()
broadcast=vehicle.broadcast
gimbalManager=vehicle.gimbalManager

options={}
options["camera_position"]=osdk.OSDK_CAMERA_POSITION_NO_1
options["playload_index"]=osdk.PAYLOAD_INDEX_0
options["h264_source"]=osdk.OSDK_CAMERA_SOURCE_H20T_ZOOM
options["enable_rtmv"]=True
node=osdk.VehicleNode(vehicle,options)
sub_rtmv=osdk.messenger.subscribe("rtmv",0,callback_buf)

# 3. control gimbal
while True:
  t_i= time.time()
  q=broadcast.getQuaternion()
  global_pos=broadcast.getGlobalPosition()
  height=global_pos["height"]
  uav_rotation=osdk.SO3(q[0],q[1],q[2],q[3])
  x_dir=uav_rotation.trans(osdk.Point3d(1,0,0))
  pitch,yaw,roll=gimbal_planner.get_gimbal_at_time(t_i,(x_dir.x,x_dir.y,x_dir.z))
  print("now:",broadcast.getGimbal())
  print("control: t:%f,pitch:%f,yaw:%f,roll:%f"%(t_i,pitch,yaw,roll))
  gimbalManager.rotateSync(osdk.PAYLOAD_INDEX_0,{"pitch":pitch,"yaw":yaw,"roll":roll,"time":0.1},1)
  time.sleep(0.1)
