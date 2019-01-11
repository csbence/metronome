#! /usr/bin/env python3

import os
import sys
import argparse
import random
import copy
from subprocess import run, PIPE, Popen
import json
from shutil import copyfile

__author__ = 'Kevin C. Gall'

# Good configs:
# python3 generate_grid_world.py 1500 1500 1000 -o 0.0006 -s corridors -c 0.07 -e 2
#   around 10% solvable


class SingleObstacleStrategy:
    def add_obstacle(self, obstacleTracker, start):
        obstacleTracker.add(start[0], start[1])

    def reset(self):
        pass


class EnclosureObstacleStrategy:
    def __init__(self, sizeBound, widthFactor = 1.0, exits=0, equalLength = False, alignDirections = False):
        self.sizeBound = sizeBound
        self.widthSizeBound = int(max(sizeBound * widthFactor, 1))
        self.exits = exits
        self.equalLength = equalLength
        self.alignDirections = alignDirections
        self.directions = self.get_directions() if alignDirections else None


    def get_directions(self):
        firstVector = dict()
        midVector = dict()
        lastVector = dict()
        direction = random.randint(0, 3)

        if (direction == 0):
            firstVector['x'] = -1
            firstVector['y'] = 0
        elif (direction == 1):
            firstVector['x'] = 1
            firstVector['y'] = 0
        elif (direction == 2):
            firstVector['x'] = 0
            firstVector['y'] = -1
        elif (direction == 3):
            firstVector['x'] = 0
            firstVector['y'] = 1

        lastVector['x'] = -firstVector['x']
        lastVector['y'] = -firstVector['y']

        nextDirection = random.randint(0, 1)
        if nextDirection == 0:
            nextDirection = -1

        if firstVector['x'] == 0:
            midVector['x'] = nextDirection
            midVector['y'] = 0
        else:
            midVector['y'] = nextDirection
            midVector['x'] = 0

        return (firstVector, midVector, lastVector)


    def add_obstacle(self, obstacleTracker, start):
        # Get random size for the enclosure
        firstWallSize = random.randint(1, self.sizeBound)
        midWallSize = random.randint(1, self.widthSizeBound)
        lastWallSize = firstWallSize + 1 if self.equalLength else random.randint(1, self.sizeBound)

        # get the directions - which way does the wall extend from its starting position
        obstacleDirections = self.directions if self.directions != None else self.get_directions()

        firstVector = obstacleDirections[0]
        midVector = obstacleDirections[1]
        lastVector = obstacleDirections[2]

        # get random exits - "holes" in the walls where an agent can pass through
        exitSet = set()
        while len(exitSet) < self.exits:
            exitWall = random.randint(0, 1)
            # only the length walls have exits
            if exitWall == 1:
                exitWall = 2

            wallSize = firstWallSize if exitWall == 0 else lastWallSize
            exitLoc = random.randint(0, wallSize)

            exitSet.add((exitWall, exitLoc))

        # Add wall spaces to the obstacle tracker
        current = (start[0], start[1])
        for wallIndex, wall in enumerate([(firstVector, firstWallSize), (midVector, midWallSize), (lastVector, lastWallSize)]):
            for i in range(wall[1]):
                if (wallIndex, i) not in exitSet:
                    obstacleTracker.add(current)
                current = (current[0] + wall[0]['x'], current[1] + wall[0]['y'])

    def reset(self):
        if self.alignDirections:
            self.directions = self.get_directions()


def generate_goals(goals, width, height):
    goalSet = set()
    while len(goalSet) < goals:
        goalSet.add((random.randint(0, width), random.randint(0, height)))

    return goalSet


def generate_filter_configs(domains, useVacuum):
    config_list = []

    domainName = 'GRID_WORLD'
    if useVacuum:
        domainName = 'VACUUM_WORLD'

    for domain in domains:
        config = dict()
        config['algorithmName'] = 'A_STAR'
        config['actionDuration'] = 1
        config['domainName'] = domainName
        config['terminationType'] = 'EXPANSION'
        config['lookaheadType'] = 'DYNAMIC'
        config['commitmentStrategy'] = 'SINGLE'
        config['heuristicMultiplier'] = 1.0
        # remove leading . char in a relative path
        config['domainPath'] = domain[1:]

#        if not useVacuum:
#            config2 = config.copy()
#            config2['domainName'] = 'VACUUM_WORLD'
#            config_list.append(config2)

        config_list.append(config)

    return config_list


def main(args):
    height = args.height
    width = args.width
    total = args.total
    goals = args.goals
    obstaclePercentage = args.obstacle_probability

    # Size bound for enclosures calculated using power
    sizeBound = max(int(height ** 0.7), 1)

    if args.verbose:
        print(args.height)
        print(args.width)
        print(f'Percent chance a cell will start an obstacle: {obstaclePercentage}')
        print(f'{goals} goal(s) will be generated')

    strategy = args.strategy
    obstacleBuilder = None
    if strategy == 'single':
        obstacleBuilder = SingleObstacleStrategy()
        if args.verbose:
            print('Single Cell obstacle strategy')

    elif strategy == 'minima':
        obstacleBuilder = EnclosureObstacleStrategy(sizeBound)
        if args.verbose:
            print(f'Minima obstacle strategy. Size bound {sizeBound}')

    elif strategy == 'corridors':
        obstacleBuilder = EnclosureObstacleStrategy(sizeBound,
                                                    widthFactor=args.corridor_width_factor,
                                                    exits=args.corridor_exits,
                                                    equalLength=True)
        if args.verbose:
            print(f'Corridors obstacle strategy. Size bound {sizeBound}')

    elif strategy == 'corridors-aligned':
        obstacleBuilder = EnclosureObstacleStrategy(sizeBound,
                                                    widthFactor=args.corridor_width_factor,
                                                    exits=args.corridor_exits,
                                                    equalLength=True,
                                                    alignDirections=True)
        if args.verbose:
            print(f'Aligned Corridors obstacle strategy. Size bound {sizeBound}')

    endX = width
    endY = height
    # hard code starting position. Consider making this configurable
    startX = 1
    startY = 1

    outPath = args.path

    config_type = strategy
    if config_type == 'single':
        config_type = 'uniform'

    if args.verbose:
        print(f'Generating {total} worlds of type {config_type}')

    if not os.path.exists(outPath):
        os.makedirs(outPath)

    generated_domains = []
    base_domain_name = config_type + str(height) + '_' + str(width) + '-'
    for iteration in range(total):
        obstacleBuilder.reset()

        newDomain = base_domain_name + str(iteration)
        completeFile = os.path.join(outPath, newDomain+'.vw')
        generated_domains.append(completeFile)

        aFile = open(completeFile, 'w')

        preamble = str(width)+'\n'+str(height)+'\n'
        world = ''

        goal_set = generate_goals(goals, width, height)

        obstacleLocations = set()
        for y in range(0, height):
            for x in range(0, width):
                if random.random() < obstaclePercentage:
                    obstacleBuilder.add_obstacle(obstacleLocations, (x, y))

        for y in range(0, height):
            for x in range(0, width):
                if (x == startX) and (y == startY):
                    world += '@'
                elif (x, y) in goal_set:
                    world += '*'
                elif (x, y) in obstacleLocations:
                    world += '#'
                else:
                    world += '_'
            world += '\n'

        if args.verbose:
            print(world)

        aFile.write(preamble + world)

        aFile.close()

    if args.filter:
        this_cwd = os.getcwd()

        if args.verbose:
            print(f'Metronome path: {args.filter}')

        success_index = 0
        
        filtered_dir = os.path.join(outPath, 'filtered')
        if not os.path.exists(filtered_dir):
            os.makedirs(filtered_dir)
        configs = generate_filter_configs(generated_domains, goals > 1)

        sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
        from Metronome import distributed_execution

        if args.verbose:
            print('Begin filtering of generated domains')
        
        os.chdir('../..')
        results = distributed_execution(configs, this_cwd)
        os.chdir(this_cwd)

        for result in results:
            if (result['success']):
                print(f'Domain {result["configuration"]["domainPath"]} is solvable')

                copyfile('.' + result['configuration']['domainPath'], os.path.join(filtered_dir, base_domain_name + str(success_index)))
                success_index += 1
            else:
                print(f'Domain {result["configuration"]["domainPath"]} was not successfully solved')




if __name__ == '__main__':
    # Begin Argument Definition

    parser = argparse.ArgumentParser()

    parser.add_argument('height', help='the height of the Vehicle world', type=int)
    parser.add_argument('width', help='the width of the Vehicle world', type=int)
    parser.add_argument('total', help='total number of worlds to generate', type=int)
    parser.add_argument('-g', '--goals', help='total number of goals to generate', type=int, default=1)
    parser.add_argument('-p', '--path', help='directory path to save the worlds. MUST BE RELATIVE PATH to cwd. That is a known issue, but no time to fix', default='./gridworld')
    parser.add_argument('-v', '--verbose', help='increase output verbosity', action='store_true')
    parser.add_argument('-o', '--obstacle-probability', default=0.0, type=float,
                        help='probability of obstacle to begin in any given grid cell')
    parser.add_argument('-s', '--strategy', choices=['single', 'minima', 'corridors', 'corridors-aligned'], default='single',
                        help='obstacle structure strategy for the generated worlds. Defaults to "single". If "corridors-aligned", all corridors will be facing one direction')
    parser.add_argument('-c','--corridor-width-factor', default=0.05, type=float,
                        help='Factor of the width of a corridor. Only used in the corridors and corridors-aligned strategies')
    parser.add_argument('-e', '--corridor-exits', default=1, type=int,
                        help='If a corridor strategy, defines how many "exits" from the corridor will be generated. Exits appear on either of the length walls')
    parser.add_argument('-f', '--filter', default=None, action='store_true',
                        help='Filter generated domains to only solvable. Assumes a previous build of Metronome. Executes A_STAR on each domain.')

    # End argument definition

    main(args=parser.parse_args())

