#!/usr/bin/env python3

import random 
import sys

__author__ = "Bence Cserna (bence@cserna.net)"

def parse_grid():
    width = int(sys.stdin.readline().strip())
    height = int(sys.stdin.readline().strip())

    grid = []
    for x in range(0, height):
        grid.append(sys.stdin.readline().strip())

    return width, height, grid

def remove_start_goal(grid):
    grid = [line.replace('@','_') for line in grid]     
    grid = [line.replace('*','_') for line in grid]     
    return grid

def add_random_start_goal(grid, width, height):

    while True:
        start_x = random.randrange(0, width)    
        end_x = random.randrange(0, width)    
        start_y = random.randrange(0, height)    
        end_y = random.randrange(0, height)    
    
        if grid[start_y][start_x] == '_' and grid[end_y][end_x] == '_' and (start_x  != end_x or start_y != end_y):
            break
    
    
    charList = list(grid[start_y])
    charList[start_x] = '@'

    grid[start_y] = "".join(charList) 

    charList = list(grid[end_y])

    charList[end_x] = '*'

    grid[end_y] = "".join(charList) 
    return grid

def main():
    width, height, grid = parse_grid();
    grid = remove_start_goal(grid)
    grtd = add_random_start_goal(grid, width, height)

    print(width)
    print(height)
    for line in grid:
        print(line)
    

if __name__ == "__main__":
    main()
