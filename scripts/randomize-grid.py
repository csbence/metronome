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
    for y, line in enumerate(grid):
        x = line.find('@')
        if x != -1:
            start_x = x
            start_y = y

        x = line.find('*')
        if x != -1:
            end_x = x
            end_y = y

    grid = [line.replace('@', '_') for line in grid]
    grid = [line.replace('*', '_') for line in grid]
    return grid, start_x, start_y, end_x, end_y


def add_random_start_goal(grid, width, height, old_start_x, old_start_y, old_end_x, old_end_y):
    while True:
        start_x = random.randint(-2, 2) + old_start_x
        end_x = random.randint(-2, 2) + old_end_x
        start_y = random.randint(-2, 2) + old_start_y
        end_y = random.randint(-2, 2) + old_end_y

        if start_x < 0 or start_y < 0 or end_x < 0 or end_y < 0 or start_x >= width or start_y >= height or end_x >= width or end_y >= height:
            continue

        if grid[start_y][start_x] == '_' and grid[end_y][end_x] == '_' and (start_x != end_x or start_y != end_y):
            break

    char_list = list(grid[start_y])
    char_list[start_x] = '@'

    grid[start_y] = "".join(char_list)

    char_list = list(grid[end_y])

    char_list[end_x] = '*'

    grid[end_y] = "".join(char_list)
    return grid


def main():
    width, height, grid = parse_grid();
    grid, start_x, start_y, end_x, end_y = remove_start_goal(grid)
    grid = add_random_start_goal(grid, width, height, start_x, start_y, end_x, end_y)

    print(width)
    print(height)
    for line in grid:
        print(line)


if __name__ == "__main__":
    main()
