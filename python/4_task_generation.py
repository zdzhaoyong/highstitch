#! /usr/bin/python3
import svar
import numpy as np
import cv2
import time
import json
import os
import argparse
from mission import * 

parser = argparse.ArgumentParser()
parser.add_argument("-rtmv",default="./data/sample.rtmv",help="The rtmv file captured")
parser.add_argument("-task",default="./data/sample",help="The rtmv file captured")
parser.add_argument("-skip",default=1,type=int,help="Skip video frame to reduce cpu usage")
parser.add_argument("-cam",default="[1920,1440,3350.778698,3350.778698,960,720]",help="The camera parameters in json array")
parser.add_argument("-yaw_offset",default=0,type=float,help="Fix yaw")
parser.add_argument("-area",default="area.json",help="The target mission area")
parser.add_argument("-height",default=300,type=float,help="The target mission height")
parser.add_argument("-v_max",default=15,help="The max flight velocity")
parser.add_argument("-kml",default="mission.kml",help="The target mission kml file, not supported yet")
parser.add_argument("-intra_num",default=6,type=int,help="how many images to take in a half cycle")
parser.add_argument("-airline_direction",default=0,help="the airline direction in degree")
parser.add_argument("-intra_overlap",default=0.6,help="the intra overlap rate")
parser.add_argument("-forward_overlap",default=0.6,help="the forward overlap rate")
parser.add_argument("-min_timestamp",default=-1,type=float,help="The minimum timestamp")
parser.add_argument("-max_timestamp",default=-1,type=float,help="The maximum timestamp")
  
args = parser.parse_args()

camera    =json.loads(args.cam)
fov_width =math.atan2(camera[0]/2,camera[2])*360/3.1415926
fov_height=math.atan2(camera[1]/2,camera[3])*360/3.1415926
mission_settings={"mission_intra_number":args.intra_num,
                  "mission_height":args.height,
                  "fov_width":fov_width,"fov_height":fov_height,
                  "mission_intra_overlap":args.intra_overlap,
                  "mission_forward_overlap":args.forward_overlap,
                  "mission_airline_direction":args.airline_direction/3.1415926*180
                  }

gimbal_planner=GimbalPlanner(mission_settings)

# 2. play rtmv
rtmv=svar.load('svar_rtmv')
rtmv.set_gps_rad(False)
dataset=rtmv.DatasetNetwork({"ip":args.rtmv,"port":1212,"skip":args.skip})
camera    =json.loads(args.cam)

taskjson={"task":"offline_planar","parameters":{},"dataset":{"type":"images","cameras":{"h20t_rtmv":camera},"images":[]}}
last_shot_time=-100

def callback_frame(fr):
  if fr == None:
    return
 
  gimage=fr.getImage(0,0)
  image= np.array(gimage)
  image.dtype='uint8'
  gps=fr.getGPSLLA()
  pyr=fr.getPitchYawRoll()
  height=fr.getHeight2Ground()

  global last_shot_time
  if gimbal_planner.should_take_photo(last_shot_time,fr.timestamp()) :
    last_shot_time=fr.timestamp()
    if last_shot_time < args.min_timestamp:
      return
      
    if args.max_timestamp > 0 and last_shot_time > args.max_timestamp:
      return
    # meta
    gps={"longitude":gps["lng"],"latitude":gps["lat"],"altitude":gps["alt"]}
    gpsSigma={"longitude":5,"latitude":5,"altitude":10}
    attitudeSigma={"pitch":1,"roll":1,"yaw":10}
    height={"sigma":height["sigma"],"value":height["height"]}
    frame_json={"timestamp":last_shot_time,"gps":gps,"gpsSigma":gpsSigma,"height":height,
                "attitude":{"pitch":pyr["pitch"], "yaw":pyr["yaw"]+args.yaw_offset, "roll":pyr["roll"]},
                "attitudeSigma":attitudeSigma,"camera":"h20t_rtmv"}

    image_path=os.path.abspath(os.path.join(args.task,str(fr.timestamp())+".jpg"))
    task_path=os.path.join(args.task,'task.json')
    frame_json["image"]=image_path
    print(frame_json)
    taskjson["dataset"]["images"].append(frame_json)
    open(task_path,'w').write(json.dumps(taskjson,indent=2))   

    cv2.imwrite(image_path,image)

sub_kf= rtmv.messenger.subscribe('UI.output.curframe',0,callback_frame)

while True:
  time.sleep(0.01)
