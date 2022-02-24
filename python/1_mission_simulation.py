#! /usr/bin/python3
import svar
import numpy as np
import time
import os
import math
import json
import argparse
from mission import *    

parser = argparse.ArgumentParser()
parser.add_argument("-area",default="area.json",help="The target mission area")
parser.add_argument("-height",default=300,type=float,help="The target mission height")
parser.add_argument("-v_max",default=15,type=float,help="The max flight velocity")
parser.add_argument("-kml",default="highstitch",help="The target mission kml file")
parser.add_argument("-cam",default="[1440,1920,3350.778698,3350.778698,960,720]",help="The camera parameters in json array")
parser.add_argument("-intra_num",default=4,type=int,help="how many images to take in a half cycle")
parser.add_argument("-airline_direction",default=0,type=float,help="the airline direction in degree")
parser.add_argument("-intra_overlap",default=0.6,type=float,help="the intra overlap rate")
parser.add_argument("-forward_overlap",default=0.6,type=float,help="the forward overlap rate")
parser.add_argument("-side_overlap",default=0.8,type=float,help="the side overlap rate")
parser.add_argument("-timespeed",default=10,type=float,help="the simulation time speed")
  
args = parser.parse_args()

# 1. load mission area from geojson
area_geojson=json.loads(open(args.area).read())
poly_feature=area_geojson
if area_geojson["type"] == "FeatureCollection":
  for feature in area_geojson["features"]:
     if feature["geometry"]["type"] == "Polygon":
       poly_feature=feature
       break
poly_coordinates=poly_feature["geometry"]["coordinates"][0]

# 2. perform mission planning
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

mission_planner=MissionPlanner(mission_settings)
gimbal_planner=GimbalPlanner(mission_settings)
lats,lons,lengths=mission_planner.get_flight_path(poly_coordinates)
mission_planner.export_kml(args.kml,lats,lons)

print("It need about %f minutes to process the mission."%(sum(lengths)/mission_planner.v_max/60))

# 3. visualize
qglopm=svar.load('svar_qglopm')

qapp=qglopm.QApplication()
vis2d= qglopm.Visualizer2D({"home":poly_coordinates[0]+[0.]})
vis2d.show()
vis2d.goHome()

# 3.1 base layer
def gaode_url(x,y,z):
  return "https://webst01.is.autonavi.com/appmaptile?style=6&x=%d&y=%d&z=%d" % (x,y,z)
base_des={"type":"livelayer","manager":gaode_url,"cache_folder":"/tmp/qglopm_cache"}
base_layer=vis2d.addLayer("base",base_des)
# 3.2 mission area layer
poly_layer=vis2d.addLayer("area",{"type":"geojson","content":area_geojson})
# 3.3 mission path layer
path_coordinates=[[lons[i],lats[i]] for i in range(len(lats))]
geometry={"coordinates":path_coordinates,"type":"LineString"}
path_layer=vis2d.addLayer("path",{"type":"geojson","content":{"type":"Feature","geometry":geometry,
                                  "properties":{"stroke":"#f00000"}}})

# 3.4 mission waypoints layer
for i in range(0,len(lats)):
  waypointdes={"type":"imageoverlayer","image":"waypoint.png","coordinates":[[lats[i],lons[i]]]}
  vis2d.addLayer("imageover"+str(i),waypointdes)

# 3.5 mission uav layer
uav_layer=vis2d.addLayer("uav",{"type":"imageoverlayer","image":"waypoint.png","coordinates":[[lats[0],lons[0]]]})

# 3.6 simulate uav photo capture
first_time=time.time()
last_shot_time=-100
frameid=0
is_first_frame=True
while True:
  qapp.processEvents()
  time.sleep(0.01)
  sim_time=(time.time()-first_time)*args.timespeed
  lat,lon,alt,v_direction=mission_planner.get_gps_at_time(lats,lons,lengths,sim_time)
  if lat is None:
    continue
  pitch,yaw,roll=gimbal_planner.get_gimbal_at_time(sim_time,v_direction)
  uav_layer.set({"coordinates":[[lat,lon]],"yaw":math.atan2(v_direction[1],v_direction[0])})

  if is_first_frame:
    vis2d.setHomePOS([lon,lat,0])
    vis2d.goHome()
    is_first_frame=False

  if gimbal_planner.should_take_photo(last_shot_time,sim_time) :
    print("sim_time:%f,lat:%f,lon:%f,alt:%f,pitch:%f,yaw:%f,roll:%f"%(sim_time,lat,lon,alt,pitch,yaw,roll))
    last_shot_time=sim_time
    # add photo here
    frjson={"timestamp":sim_time,"lon":lon,"lat":lat,"alt":alt,"height":gimbal_planner.h,
            "pitch":pitch,"yaw":yaw,"roll":roll,"camera":camera}
    fr=qglopm.frame_from_json(frjson)
    tl=qglopm.from_pixel_to_lla(fr,(0,0))
    tr=qglopm.from_pixel_to_lla(fr,(camera[0],0))
    br=qglopm.from_pixel_to_lla(fr,(camera[0],camera[1]))
    bl=qglopm.from_pixel_to_lla(fr,(0,camera[1]))
    
    fr_overlayerdes={"type":"imageoverlayer",
                     "image":"green"}
    fr_overlayerdes["coordinates"]=[[tl.x,tl.y],[tr.x,tr.y],[br.x,br.y],[bl.x,bl.y]]
    fr_overlayer=vis2d.addLayer("frame"+str(frameid),fr_overlayerdes)
    frameid+=1
