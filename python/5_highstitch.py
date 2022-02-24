#! /usr/bin/python3
import svar
import json
import argparse
import os

parser = argparse.ArgumentParser()
parser.add_argument("-task",default="./data/sample/task.json",help="The input task json file")
parser.add_argument("-out",default='dom.tif',help="The output ortho mosaic image file")
parser.add_argument("-arg", nargs='+', action='append',default=[],help="the processing parameters")
parser.add_argument("-with_pose", action="store_true", help="Use pose stitching instead of reconstruct")
  
args = parser.parse_args()

sibitu=svar.load('svar_highstitch')

task=json.loads(open(args.task).read())

for it_args in args.arg:
  for arg in it_args:
    key,value= arg.split('=')
    task["parameters"][key]=value

task["parameters"]["topdir"]= os.path.dirname(args.task)

os.system('rm area.txt *.tile -f')

if args.with_pose:
    success=sibitu.stitch_with_pose(task,args.out)
else:
    success=sibitu.stitch_task(task,args.out)
os.system('rm area.txt *.tile -f')


