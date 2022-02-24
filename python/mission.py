import numpy as np
import time
import os
import math
import json
import xml.dom.minidom

def rad2degree(r):
    return 180/3.1415926*r

def degree2rad(d):
    return 3.1415926/180.*d

class MissionPlanner():
  def __init__(self,config):
    self.config=config
    self.f_w=config.get("fov_width",20.)
    self.f_h=config.get("fov_height",15.)
    self.h  =config.get("mission_height",300)
    self.v_max=config.get("mission_max_velocity",15.)
    
    self.o_f=config.get("mission_forward_overlap",0.6)
    self.o_s=config.get("mission_side_overlap",0.6)
    self.o_i=config.get("mission_intra_overlap",0.6)
    self.n  =config.get("mission_intra_number",4)
    self.airline_yaw=config.get("mission_airline_direction",0)

    self.w_i=2*self.h*math.tan(degree2rad(self.f_w)/2) # the image viewport in meters
    self.h_i=2*self.h*math.tan(degree2rad(self.f_h)/2) # the image height viewport in meters

    self.f_h_e=self.f_h*self.o_i+(1-self.o_i)*self.f_h*self.n # concated fake FOV height, for latex
    self.d_f=(1-self.o_f)*self.w_i*2           # flight distance for one cycle
    self.d_s=2*self.h*math.tan(degree2rad(self.f_h_e)/2)- self.o_s*self.h_i # distance between airlines
  
  @staticmethod
  def transform(xs,ys,theta,center):
    cosTheta=math.cos(theta)
    sinTheta=math.sin(theta)

    return cosTheta*(xs-center[0])+sinTheta*(ys-center[1]), cosTheta*(ys-center[1])-sinTheta*(xs-center[0])

  @staticmethod
  def latlon_to_xy(lats,lons):
    DEG2RAD=0.017453292519943
    a = 6378137.0
    f = 1.0 / 298.257223563
    e_2 = 2 * f - f * f
    xs,ys=([0],[0])

    phi_rad = lats[0] * DEG2RAD;
    lng_unit = DEG2RAD * a * math.cos(phi_rad) / math.sqrt(1 - e_2 * math.sin(phi_rad) * math.sin(phi_rad))
    lat_unit = DEG2RAD * a * (1 - e_2) / math.pow(1 - e_2 * math.sin(phi_rad) * math.sin(phi_rad), 1.5)

    return (lons - lons[0]) * lng_unit,(lats - lats[0]) * lat_unit

  @staticmethod
  def latlon_from_xy(xs,ys,ref):
    DEG2RAD=0.017453292519943
    a = 6378137.0
    f = 1.0 / 298.257223563
    e_2 = 2 * f - f * f

    lats,lons=([],[])
    phi_rad = ref[0] * DEG2RAD;
    lng_unit = DEG2RAD * a * math.cos(phi_rad) / math.sqrt(1 - e_2 * math.sin(phi_rad) * math.sin(phi_rad))
    lat_unit = DEG2RAD * a * (1 - e_2) / math.pow(1 - e_2 * math.sin(phi_rad) * math.sin(phi_rad), 1.5)

    return ys / lat_unit + ref[0],xs / lng_unit + ref[1]

  def get_flight_path(self,area_poly):
    ref   = (area_poly[0][1],area_poly[0][0])
    area_poly_array=np.array(area_poly)
    xs,ys = MissionPlanner.latlon_to_xy(area_poly_array[:,1],area_poly_array[:,0])
    center= (0,0)
    xs,ys = MissionPlanner.transform(xs,ys,self.airline_yaw,center)

    max_y,min_y=(max(ys)- self.d_s*0.5,min(ys) )
    lines=[]
    for i in range(1,len(xs)+1):
      i0,i1=(i-1,i%len(xs))
      a = ys[i0] - ys[i1]
      b = xs[i1] - xs[i0]
      c = xs[i0]*ys[i1] - xs[i1]*ys[i0]
      s = (xs[i0], ys[i0])
      e = (xs[i1], ys[i1])
      if a == 0:
        a=1e-9
      lines.append((a,b,c,s,e))

    forward=True
    path=[]
    for y in np.arange(max_y,min_y,-self.d_s):
      maxX,minX=-1e9,1e9
      for line in lines:
        a,b,c,s,e=line
        if y <= min(s[1],e[1]) or y >= max(s[1],e[1]):
          continue
        x=-(b*y+c)/a
        maxX=max(maxX,x)
        minX=min(minX,x)
      if forward:
        path+=((minX,y),(maxX,y))
      else:
        path+=((maxX,y),(minX,y))
      forward=not forward

    patharray=np.array(path)
     
    path_xs,path_ys=MissionPlanner.transform(patharray[:,0],patharray[:,1],-self.airline_yaw,(-center[0],-center[1]))

    d_x = path_xs[1:]-path_xs[0:len(path_xs)-1]
    d_y = path_ys[1:]-path_ys[0:len(path_ys)-1]
    path_lats,path_lons=MissionPlanner.latlon_from_xy(path_xs,path_ys,ref)

    return path_lats,path_lons,np.sqrt(d_x*d_x+d_y*d_y)
    
  def get_gps_at_time(self,lats,lons,lengths,tm):
    # get similated gps position
    i,s,traj_length=0,0,self.v_max*tm
    for i in range(0,len(lengths)):
      s+=lengths[i]
      if s >= traj_length:
        break
      i=i+1
    if i >= len(lengths):
      return None,None,None,None
      
    percent= (s-traj_length)/lengths[i]
    #print(i,percent,traj_length,s)
    xs,ys=MissionPlanner.latlon_to_xy(lats[i:i+2],lons[i:i+2])
    x,y=xs[0]*percent+xs[1]*(1-percent),ys[0]*percent+ys[1]*(1-percent)
    lat,lon=MissionPlanner.latlon_from_xy(x,y,(lats[i],lons[i]))
    v_direction=ys[1]-ys[0],xs[1]-xs[0]
    return lat,lon,self.h,v_direction

  def export_kml(self,name,lats,lons):
    doc=xml.dom.minidom.Document()
    kml=doc.createElement("kml")
    kml.setAttribute('xmlns','http://www.opengis.net/kml/2.2')
    doc.appendChild(kml)
    Document=doc.createElement('Document')
    Document.setAttribute('xmlns','')
    kml.appendChild(Document)
    nameNode=doc.createElement('name')
    nameNode.appendChild(doc.createTextNode(name))
    Document.appendChild(nameNode)
    openNode=doc.createElement('open')
    openNode.appendChild(doc.createTextNode('1'))
    Document.appendChild(openNode)

    ExtendedData=doc.createElement('ExtendedData')
    ExtendedData.setAttribute('xmlns:mis',"www.dji.com")    
    typeNode=doc.createElement('mis:type')
    typeNode.appendChild(doc.createTextNode('Waypoint'))
    ExtendedData.appendChild(typeNode)
    stationType=doc.createElement('mis:stationType')
    stationType.appendChild(doc.createTextNode('1'))
    ExtendedData.appendChild(stationType)
    Document.appendChild(ExtendedData)

    Style=doc.createElement('Style')
    LineStyle=doc.createElement('LineStyle')
    color=doc.createElement('color')
    width=doc.createElement('width')
    Style.setAttribute('id',"waylineGreenPoly")
    color.appendChild(doc.createTextNode('FF0AEE8B'))
    width.appendChild(doc.createTextNode('6'))
    Style.appendChild(LineStyle)
    LineStyle.appendChild(color)
    LineStyle.appendChild(width)
    Document.appendChild(Style)

    Style=doc.createElement('Style')
    waypointStyle=doc.createElement('waypointStyle')
    Icon=doc.createElement('Icon')
    href=doc.createElement('href')
    Style.setAttribute('id',"waypointStyle")
    href.appendChild(doc.createTextNode('https://cdnen.dji-flighthub.com/static/app/images/point.png'))
    Style.appendChild(waypointStyle)
    waypointStyle.appendChild(Icon)
    Icon.appendChild(href)    
    Document.appendChild(Style)

    Folder=doc.createElement('Folder')
    Document.appendChild(Folder)
    nameNode=doc.createElement('name')
    nameNode.appendChild(doc.createTextNode('Waypoints'))
    Folder.appendChild(nameNode)
    # <description>Waypoints in the Mission.</description>
    description=doc.createElement('description')
    description.appendChild(doc.createTextNode('Waypoints in the Mission.'))
    Folder.appendChild(description)

    for i in range(0,len(lats)):
      Placemark=doc.createElement('Placemark')
      nameNode=doc.createElement('name')
      nameNode.appendChild(doc.createTextNode('Waypoint%d'%i))
      Placemark.appendChild(nameNode)

      visibility=doc.createElement('visibility')
      visibility.appendChild(doc.createTextNode('1'))
      Placemark.appendChild(visibility)

      description=doc.createElement('description')
      description.appendChild(doc.createTextNode('Waypoint'))
      Placemark.appendChild(description)

      styleUrl=doc.createElement('styleUrl')
      styleUrl.appendChild(doc.createTextNode('#waypointStyle'))
      Placemark.appendChild(styleUrl)

      ExtendedData=doc.createElement('ExtendedData')
      ExtendedData.setAttribute('xmlns:mis',"www.dji.com")   
      useWaylineAltitude=doc.createElement('mis:useWaylineAltitude')
      heading=doc.createElement('mis:heading')
      turnMode=doc.createElement('mis:turnMode')
      gimbalPitch=doc.createElement('mis:gimbalPitch')
      useWaylineSpeed=doc.createElement('mis:useWaylineSpeed')
      speed=doc.createElement('mis:speed')
      useWaylineHeadingMode=doc.createElement('mis:useWaylineHeadingMode')
      useWaylinePointType=doc.createElement('mis:useWaylinePointType')
      pointType=doc.createElement('mis:pointType')
      cornerRadius=doc.createElement('mis:cornerRadius')

      useWaylineAltitude.appendChild(doc.createTextNode('true'))
      heading.appendChild(doc.createTextNode('0'))
      turnMode.appendChild(doc.createTextNode('Auto'))
      gimbalPitch.appendChild(doc.createTextNode('0.0'))
      useWaylineSpeed.appendChild(doc.createTextNode('useWaylineSpeed'))
      speed.appendChild(doc.createTextNode(str(self.v_max)))
      useWaylineHeadingMode.appendChild(doc.createTextNode('true'))
      useWaylinePointType.appendChild(doc.createTextNode('true'))
      pointType.appendChild(doc.createTextNode('LineStop'))
      cornerRadius.appendChild(doc.createTextNode('0.2'))

      ExtendedData.appendChild(useWaylineAltitude)
      ExtendedData.appendChild(heading)
      ExtendedData.appendChild(turnMode)
      ExtendedData.appendChild(gimbalPitch)
      ExtendedData.appendChild(useWaylineSpeed)
      ExtendedData.appendChild(speed)
      ExtendedData.appendChild(useWaylineHeadingMode)
      ExtendedData.appendChild(useWaylinePointType)
      ExtendedData.appendChild(pointType)
      ExtendedData.appendChild(cornerRadius)
      Placemark.appendChild(ExtendedData)

      Point=doc.createElement('Point')
      altitudeMode=doc.createElement('altitudeMode')
      coordinates=doc.createElement('coordinates')
      altitudeMode.appendChild(doc.createTextNode('relativeToGround'))
      coordinates.appendChild(doc.createTextNode('%.10f,%.10f,%.1f'%(lons[i],lats[i],self.h)))
      Point.appendChild(altitudeMode)
      Point.appendChild(coordinates)
      Placemark.appendChild(Point)
      Folder.appendChild(Placemark)

    Placemark=doc.createElement('Placemark')
    nameNode=doc.createElement('name')
    nameNode.appendChild(doc.createTextNode('Wayline'))
    Placemark.appendChild(nameNode)
    Document.appendChild(Placemark)

    description=doc.createElement('description')
    description.appendChild(doc.createTextNode('Wayline'))
    Placemark.appendChild(description)

    visibility=doc.createElement('visibility')
    visibility.appendChild(doc.createTextNode('1'))
    Placemark.appendChild(visibility)

    ExtendedData=doc.createElement('ExtendedData')
    ExtendedData.setAttribute('xmlns:mis',"www.dji.com")   
    altitude=doc.createElement('mis:altitude')
    autoFlightSpeed=doc.createElement('mis:autoFlightSpeed')
    actionOnFinish=doc.createElement('mis:actionOnFinish')
    headingMode=doc.createElement('mis:headingMode')
    gimbalPitchMode=doc.createElement('mis:gimbalPitchMode')
    powerSaveMode=doc.createElement('mis:powerSaveMode')
    waypointType=doc.createElement('mis:waypointType')
    droneInfo=doc.createElement('mis:droneInfo')
    droneType=doc.createElement('mis:droneType')
    advanceSettings=doc.createElement('mis:advanceSettings')
    droneCameras=doc.createElement('mis:droneCameras')
    camera=doc.createElement('mis:camera')
    droneHeight=doc.createElement('mis:droneHeight')
    cameraIndex=doc.createElement('mis:cameraIndex')
    cameraName=doc.createElement('mis:cameraName')
    cameraType=doc.createElement('mis:cameraType')
    useAbsolute=doc.createElement('mis:useAbsolute')
    hasTakeoffHeight=doc.createElement('mis:hasTakeoffHeight')
    takeoffHeight=doc.createElement('mis:takeoffHeight')

    altitude.appendChild(doc.createTextNode(str(self.h)))
    autoFlightSpeed.appendChild(doc.createTextNode(str(self.v_max)))
    actionOnFinish.appendChild(doc.createTextNode('GoHome'))
    headingMode.appendChild(doc.createTextNode('Auto'))
    gimbalPitchMode.appendChild(doc.createTextNode('ControlledByRC'))
    powerSaveMode.appendChild(doc.createTextNode('false'))
    waypointType.appendChild(doc.createTextNode('CoordinateTurning'))
    droneType.appendChild(doc.createTextNode('PM430'))
    advanceSettings.appendChild(doc.createTextNode('true'))
    cameraIndex.appendChild(doc.createTextNode('0'))
    cameraName.appendChild(doc.createTextNode('Zenmuse H20T'))
    cameraType.appendChild(doc.createTextNode('43'))
    useAbsolute.appendChild(doc.createTextNode('false'))
    hasTakeoffHeight.appendChild(doc.createTextNode('false'))
    takeoffHeight.appendChild(doc.createTextNode('0.0'))

    ExtendedData.appendChild(altitude)
    ExtendedData.appendChild(autoFlightSpeed)
    ExtendedData.appendChild(actionOnFinish)
    ExtendedData.appendChild(headingMode)
    ExtendedData.appendChild(gimbalPitchMode)
    ExtendedData.appendChild(powerSaveMode)
    ExtendedData.appendChild(waypointType)
    ExtendedData.appendChild(droneInfo)
    droneInfo.appendChild(droneType)
    droneInfo.appendChild(advanceSettings)
    droneInfo.appendChild(droneCameras)
    droneInfo.appendChild(droneHeight)
    droneCameras.appendChild(camera)
    camera.appendChild(cameraIndex)
    camera.appendChild(cameraName)
    camera.appendChild(cameraType)
    droneHeight.appendChild(useAbsolute)
    droneHeight.appendChild(hasTakeoffHeight)
    droneHeight.appendChild(takeoffHeight)
    Placemark.appendChild(ExtendedData)

    styleUrl=doc.createElement('styleUrl')
    styleUrl.appendChild(doc.createTextNode('#waylineGreenPoly'))
    Placemark.appendChild(styleUrl)

    LineString=doc.createElement('LineString')
    Placemark.appendChild(LineString)

    tessellate=doc.createElement('tessellate')
    tessellate.appendChild(doc.createTextNode('1'))
    LineString.appendChild(tessellate)

    altitudeMode=doc.createElement('altitudeMode')
    altitudeMode.appendChild(doc.createTextNode('relativeToGround'))
    LineString.appendChild(altitudeMode)

    coordinates=doc.createElement('coordinates')
    coorstr=''
    for i in range(0,len(lats)):
      coorstr+='%.10f,%.10f,%f '%(lons[i],lats[i],self.h)
    coordinates.appendChild(doc.createTextNode(coorstr))
    LineString.appendChild(coordinates)
        
    doc.writexml(open(name+".kml",'w'),indent=' ',addindent='\t',newl='\n',encoding='utf-8')
    

class GimbalPlanner():
  def __init__(self,config):
    self.config=config
    self.f_w=config.get("fov_width",20.)
    self.f_h=config.get("fov_height",15.)
    self.h  =config.get("mission_height",300)
    self.v_max=config.get("mission_max_velocity",15.)
    
    self.o_f=config.get("mission_forward_overlap",0.6)
    self.o_s=config.get("mission_side_overlap",0.6)
    self.o_i=config.get("mission_intra_overlap",0.6)
    self.n  =config.get("mission_intra_number",4)

    self.w_i=2*self.h*math.tan(degree2rad(self.f_w)/2) # the image viewport in meters
    self.h_i=2*self.h*math.tan(degree2rad(self.f_h)/2) # the image height viewport in meters

    self.f_h_e=self.f_h*self.o_i+(1-self.o_i)*self.f_h*self.n # concated fake FOV height, for latex
    self.d_f=(1-self.o_f)*self.w_i*2           # flight distance for one cycle
    self.d_s=2*self.h*math.tan(degree2rad(self.f_h_e)/2)- self.o_s*self.h_i # distance between airlines
    self.t =self.d_f/self.v_max           # estimated cycle time
    self.tm=config.get("mission_gimbal_roll_time",self.t/2/self.n) # time for fast roll move, default is t/2/n
 
    #compute control points t-pitch-roll
    self.te1=(self.t/2-self.tm)/2
    self.te2=self.te1+self.t/2
    self.v_pitch= (1-self.o_i)*self.f_h*(self.n-1)/(self.te1*2)
    self.v_roll = -2*math.atan(self.te1*self.v_max/self.h)/self.tm

  def get_gimbal_at_time(self,t_i,velocity_direction):
    # compute pitch,yaw,roll for gimbal
    t_i=t_i%self.t
    yaw=rad2degree(math.atan2(velocity_direction[1],velocity_direction[0])) - 90
    if t_i <= self.t/2-self.tm:
        droll = math.atan((t_i-self.te1)*self.v_max/self.h)
        dpitch= self.v_pitch*(t_i-self.te1)
    elif t_i<=self.t/2:
        droll = self.v_roll*(t_i-self.t/2+self.tm/2)
        dpitch= self.v_pitch*(self.t/2-self.tm-self.te1)
    elif t_i <= self.t-self.tm:
        droll = math.atan((t_i-self.te2)*self.v_max/self.h)
        dpitch= -self.v_pitch*(t_i-self.te2)
    else:
        droll = self.v_roll*(t_i-self.t+self.tm/2)
        dpitch= -self.v_pitch*(self.t-self.tm-self.te2)
    pitch=dpitch-90
    roll =rad2degree(droll)
    return pitch,yaw,roll

  def should_take_photo(self,last_photo_time,now_time):
    if now_time - last_photo_time > (self.t/2-self.tm)/(self.n-1) :
      return True
    return False

  def self_test(self):
    while True:
      tm=time.time()
      pitch,yaw,roll=self.get_gimbal_at_time(tm, (0,1))
      print('time:%f,pitch:%f,yaw:%f,roll:%f'%(tm,pitch,yaw,roll))

