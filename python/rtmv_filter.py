#! /usr/bin/python3
import svar
import numpy as np
import cv2
import time
import json
import os
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("-input",default="./data/sample.rtmv",help="The rtmv file captured")
parser.add_argument("-out",default="out.rtmv",help="The output rtmv")
parser.add_argument("-min_timestamp",default=-1,type=float,help="The minimum timestamp")
parser.add_argument("-max_timestamp",default=-1,type=float,help="The maximum timestamp")
parser.add_argument("-min_time",default=0,type=float,help="The minimum time")
parser.add_argument("-min_height",default=100,type=float,help="the minimum height")
#parser.add_argument("-min_height",default=100,help="the minimum height")

args = parser.parse_args()

rtmv=svar.load('svar_rtmv')

reader=rtmv.RTMVReader(args.input)
out=open(args.out,'wb',buffering=0)

first_time=None

while True:
  package=reader.grab()
  if package is None:
    break
  print(package)
  timestamp=package["timestamp"]
  if first_time is None:
    first_time=timestamp
  t=timestamp-first_time
  h=package["H"]

  if timestamp < args.min_timestamp or t < args.min_time or h < args.min_height:
   continue

  if args.max_timestamp > 0 and timestamp > args.max_timestamp:
   continue
  
  header=memoryview(package["header"])
  payload=memoryview(package["payload"])
  out.write(header.tobytes())
  out.write(payload.tobytes())
  





