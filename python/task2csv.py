#! /usr/bin/python3
import json
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("-task",default="task.json",help="The task file")
parser.add_argument("-csv",default="pos.csv",help="the csv output")

args = parser.parse_args()

task=json.loads(open(args.task).read())

images=task["dataset"]["images"]

csv=open(args.csv,'w')

for img in images:
  gps=img["gps"]
  path=os.path.basename(img["image"])
  csv.write('%s,%.10f,%.10f,%.2f\n'%(path,gps["longitude"],gps["latitude"],gps["altitude"]))

csv.close()
