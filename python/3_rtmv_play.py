#! /usr/bin/python3
import svar
import numpy as np
import cv2
import time
import json
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-rtmv",default="./data/sample.rtmv",help="The rtmv file captured")
parser.add_argument("-skip",default=1,type=int,help="Skip video frame to reduce cpu usage")
parser.add_argument("-cam",default="[1920,1440,3350.778698,3350.778698,960,720]",help="The camera parameters in json array")
parser.add_argument("-yaw_offset",default=0,type=float,help="Fix yaw")
parser.add_argument("-not_follow",action="store_true",default=False,help="follow the uav or not")
parser.add_argument("-plot",default=False,type=bool,help="plot cv2 or not")
  
args = parser.parse_args()

# 1. show base satellite layer
qglopm=svar.load('svar_qglopm')
qapp=qglopm.QApplication()
vis2d= qglopm.Visualizer2D({"home":[108.,34.,0.]})
vis2d.show()
ui=qglopm.UIThread()
ui.translate('UI.output.curframe','ui_thread_frame')

def gaode_url(x,y,z):
  return "https://webst01.is.autonavi.com/appmaptile?style=6&x=%d&y=%d&z=%d" % (x,y,z)

base_des={"type":"livelayer","manager":gaode_url,"cache_folder":"/tmp"}
base_layer=vis2d.addLayer("base",base_des)

# 2. play rtmv
rtmv=svar.load('svar_rtmv')
rtmv.set_gps_rad(False)
dataset=rtmv.DatasetNetwork({"ip":args.rtmv,"port":1212,"skip":args.skip})
camera    =json.loads(args.cam)
uav_layer=vis2d.addLayer("uav",{"type":"imageoverlayer","image":"waypoint.png","coordinates":[[38,108]]})
is_first_frame=True

def callback_frame(fr):
  if fr == None:
    return
 
  image=fr.getImage(0,0)

  if args.plot:
    m= np.array(image)
    m.dtype='uint8'
    cv2.imshow('video', m)
    cv2.waitKey(5)
  gps=fr.getGPSLLA()
  pyr=fr.getPitchYawRoll()
  height=fr.getHeight2Ground()

  frjson={"id":fr.id(),"timestamp":fr.timestamp(),
          "lon":gps["lng"],"lat":gps["lat"],"alt":gps["alt"],
          "height":height["height"], "camera":camera,
          "pitch":pyr["pitch"], "yaw":pyr["yaw"]+args.yaw_offset, "roll":pyr["roll"]}
  print(frjson)
  fr=qglopm.frame_from_json(frjson)
  lt=rtmv.from_pixel_to_lla(fr,(0,0))
  rt=rtmv.from_pixel_to_lla(fr,(image.width(),0))
  rb=rtmv.from_pixel_to_lla(fr,(image.width(),image.height()))
  lb=rtmv.from_pixel_to_lla(fr,(0,image.height()))
  coordinates=[[lt.x,lt.y],[rt.x,rt.y],[rb.x,rb.y],[lb.x,lb.y]]
  print(coordinates)
  image_overlayerdes={"type":"imageoverlayer",
                      "image":image,"coordinates":coordinates}
  image_overlayer=vis2d.addLayer("imageover",image_overlayerdes)
  uav_layer.set({"coordinates":[[gps["lat"],gps["lng"]]]})
  if not args.not_follow or is_first_frame:
    vis2d.setHomePOS([gps["lng"],gps["lat"],0])
    vis2d.goHome()
    global is_first_frame
    is_first_frame=False

sub_kf= rtmv.messenger.subscribe('ui_thread_frame',0,callback_frame)

while True:
  qapp.processEvents()
  time.sleep(0.01)




