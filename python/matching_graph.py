#! /usr/bin/python3
import svar
import json
import argparse
import time

parser = argparse.ArgumentParser()
parser.add_argument("-gmap",default="./map.gmap",help="The input gmap file")
parser.add_argument("-geojson",default='matching_graph.json',help="The output json file representing matches")
  
args = parser.parse_args()

# generate geojson from gmap
svar_gmap=svar.load('svar_gmap')
qglopm   =svar.load('svar_qglopm')

gmap=svar_gmap.load(args.gmap)
frames=gmap.getFrames()

features=[]

# plot frame viewports
camera=frames[0].getCamera()
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
for fr in frames:
  child_center=qglopm.from_pixel_to_lla(fr,(camera[0]/2,camera[1]/2))
  parents=fr.getParents()
  for parent in parents:
    parent_center=qglopm.from_pixel_to_lla(gmap.getFrame(parent['id']),(camera[0]/2,camera[1]/2))
    coordinates=[[parent_center.y,parent_center.x],[child_center.y,child_center.x]]
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

open(args.geojson,'w').write(json.dumps(geojson,indent=2))

# vis through qglopm plugin
qapp=qglopm.QApplication()

vis2d= qglopm.Visualizer2D({})
vis2d.show()
graph_layer=vis2d.addLayer("graph",{"type":"geojson","content":geojson})
child_center=qglopm.from_pixel_to_lla(fr,(camera[0]/2,camera[1]/2))
vis2d.setHomePOS([child_center.y,child_center.x,0])
vis2d.goHome()

while True:
  qapp.processEvents()
  time.sleep(0.01)

