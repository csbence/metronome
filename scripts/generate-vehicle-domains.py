#! /usr/bin/env python3

import os.path
import argparse 
import random

parser = argparse.ArgumentParser()

parser.add_argument("height", help="the height of the Vehicle world")
parser.add_argument("width", help="the width of the Vehicle world")
parser.add_argument("-p", "--path", help="path to save the worlds")
parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
parser.add_argument("-n", "--number", help="number of worlds to generate")

args = parser.parse_args()
height = int(args.height) + 1
width = int(args.height) + 1
number = int(args.number)

if args.verbose:
  print(args.height)
  print(args.width)

obstaclePercentage = 0.30
bunkerPercentage = 0.30
startX = 1
startY = 1

endX = width
endY = height

path = args.path

if type(path) == type(None):
  path = "../resources/input/vehicle"

for iteration in range(0,number):

  newDomain = "vehicle"+str(iteration)

  completeFile = os.path.join(path, newDomain+".v")

  aFile = open(completeFile, "w")

  preamble = args.height+"\n"+args.width+"\n"
  world = ""

  for y in range(1,height):
    for x in range(1,width):
      flipObstacle = random.random()
      flipBunker = random.random() 
      if ((x == startX) and (y == startY)):
        world += "@"
      elif flipObstacle < obstaclePercentage and x != width-1:
        world += "#"
      elif flipBunker < bunkerPercentage and x != width-1:
        world += "$"
      elif (y == height-1 and x == width-1):
        world += "*"
      else:
        world += "_"
    world += "\n"

  if args.verbose:
    print(world)
    
  aFile.write(preamble + world)

  aFile.close()
