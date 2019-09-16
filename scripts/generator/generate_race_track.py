#! /usr/bin/env python3

import os.path
import argparse
import random
import numpy as np


def generate_racetracks():
    parser = argparse.ArgumentParser()
    parser.add_argument("height", help="the height of the Vehicle world",
                        type=int)

    parser.add_argument("width", help="the width of the Vehicle world",
                        type=int)

    parser.add_argument("n", help="number of worlds to generate", type=int)

    parser.add_argument("-p", "--path", help="path to save the worlds")
    parser.add_argument("-v", "--verbose", help="increase output verbosity",
                        action="store_true")
    args = parser.parse_args()
    height = args.height
    width = args.width
    domain_count = args.n
    verbose = args.verbose

    if args.verbose:
        print(args.height)
        print(args.width)

    obstaclePercentage = 0.01
    startX = 0
    startY = 0
    endX = width
    endY = height
    path = args.path
    if path is None:
        path = "./generated_tracks/"

    if not os.path.exists(path):
        os.makedirs(path)

    random.seed(1)
    
    for i in range(0, domain_count):
        target_path = os.path.join(path, str(i) + ".track")

        target_file = open(target_path, "w")

        preamble = str(width) + "\n" + str(height) + "\n"
        world = ""
        lines = []

        for y in range(0, height):
            line = ""
            
            # if y == height - 3:
            #     line = ("#" * 5 + "_" * 5)
            #     line *= int((width / len(line)))
            #     lines.append(line)
            #     continue
                
            for x in range(0, width):
                obstacle = random.random()

                if (x == 0) and (y == height - 1):
                    line += "@"

                elif x > (width - height - 2):
                    line += "*"

                elif obstacle < obstaclePercentage and y < height - 2:
                    line += "#"
                    
                else:
                    line += "_"
            
            lines.append(line)

        lines.reverse()
        world += "\n".join(lines)

        if args.verbose:
            print(world)

        target_file.write(preamble + world)
        target_file.close()


if __name__ == '__main__':
    generate_racetracks()
