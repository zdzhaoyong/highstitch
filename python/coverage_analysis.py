import svar
import json
import argparse
import time
import numpy as np
from mission import * 

parser = argparse.ArgumentParser()
parser.add_argument("-area",default="area.json",help="The target mission area")
parser.add_argument("-height",default=300,type=float,help="The target mission height")
parser.add_argument("-v_max",default=15,type=float,help="The max flight velocity")
parser.add_argument("-v_realrate",default=1,type=float,help="The max flight velocity")
parser.add_argument("-cam",default="[1920,1440,3350.778698,3350.778698,960,720]",help="The camera parameters in json array")
parser.add_argument("-intra_num",default=6,type=int,help="how many images to take in a half cycle")
parser.add_argument("-airline_direction",default=0,type=float,help="the airline direction in degree")
parser.add_argument("-intra_overlap",default=0.6,type=float,help="the intra overlap rate")
parser.add_argument("-forward_overlap",default=0.6,type=float,help="the forward overlap rate")
parser.add_argument("-side_overlap",default=0.8,type=float,help="the side overlap rate")
  
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

# generate geojson from simulation
qglopm   =svar.load('svar_qglopm')
features =[]
camera   =json.loads(args.cam)
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

# obtain frames
frames=[]
for sim_time in np.arange(0,gimbal_planner.t+0.01,(gimbal_planner.t/2-gimbal_planner.tm)/(gimbal_planner.n-1)):
  lat,lon,alt,v_direction=mission_planner.get_gps_at_time(lats,lons,lengths,sim_time*args.v_realrate)
  if lat is None:
    continue
  pitch,yaw,roll=gimbal_planner.get_gimbal_at_time(sim_time,v_direction)

  if True:
    # add photo here
    frjson={"timestamp":sim_time,"lon":lon,"lat":lat,"alt":alt,"height":gimbal_planner.h,
            "pitch":pitch,"yaw":yaw,"roll":roll,"camera":camera}
    fr=qglopm.frame_from_json(frjson)
    frames.append(fr)

# plot frame viewports
for fr in frames:
  tl=qglopm.from_pixel_to_lla(fr,(0,0))
  tr=qglopm.from_pixel_to_lla(fr,(camera[0],0))
  br=qglopm.from_pixel_to_lla(fr,(camera[0],camera[1]))
  bl=qglopm.from_pixel_to_lla(fr,(0,camera[1]))
  coordinates=[[tl.y,tl.x],[tr.y,tr.x],[br.y,br.x],[bl.y,bl.x]]
  geometry={"type": "Polygon","coordinates":[coordinates]}
  feature={"type": "Feature","properties": {},"geometry":geometry}
  features.append(feature)

# plot matching graph
coordinates=[]
for fr in frames:
  child_center=qglopm.from_pixel_to_lla(fr,(camera[0]/2,camera[1]/2))
  coordinates.append([child_center.y,child_center.x])
geometry={"type": "LineString","coordinates":coordinates}
feature={"type": "Feature","properties": {'stroke':"#02f740"},"geometry":geometry}
features.append(feature)

# plot frame centers
for fr in frames:
  child_center=qglopm.from_pixel_to_lla(fr,(camera[0]/2,camera[1]/2))
  geometry={"type": "Point","coordinates":[child_center.y,child_center.x]}
  feature={"type": "Feature","properties": {"icon":"frame_center.png"},"geometry":geometry}
  features.append(feature)

geojson={"type":"FeatureCollection","features":features}

#open(args.geojson,'w').write(json.dumps(geojson,indent=2))

# vis through qglopm plugin
qapp=qglopm.QApplication()

vis2d= qglopm.Visualizer2D({})
vis2d.show()
graph_layer=vis2d.addLayer("graph",{"type":"geojson","content":geojson})

while True:
  qapp.processEvents()
  time.sleep(0.01)

