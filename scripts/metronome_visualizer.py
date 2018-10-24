#!/usr/bin/env python3

import pygame
import sys

from pygame import gfxdraw

__author__ = 'Bence Cserna'

# Maximum window size
MAX_X = 1024
MAX_Y = 1024

# Basic colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)

# Univerisy of New Hampshire brand colors
BRAND_DARK = (37, 55, 70)
BRAND_LIGHT1 = (162, 170, 173)
BRAND_LIGHT2 = (214, 210, 196)
BRAND_ORANGE = (247, 122, 5)


class Agent:
    def __init__(self, world):
        self.world = world
        self.x = world.start[0]
        self.y = world.start[1]

    def draw(self, screen, color):
        pygame.gfxdraw.aacircle(screen,
                                int(self.x * self.world.cell_size + self.world.cell_size / 2),
                                int(self.y * self.world.cell_size - self.world.cell_size / 2),
                                int(self.world.cell_size * 0.5),
                                color,
                                )

        pygame.gfxdraw.filled_circle(screen,
                                     int(self.x * self.world.cell_size + self.world.cell_size / 2),
                                     int(self.y * self.world.cell_size - self.world.cell_size / 2),
                                     int(self.world.cell_size * 0.5),
                                     color,
                                     )

    def move(self, dx, dy):
        if not self.world.in_collision((self.x + dx, self.y + dy)):
            self.x += dx
            self.y += dy

    def move_north(self):
        self.move(0, 1)

    def move_south(self):
        self.move(0, -1)

    def move_west(self):
        self.move(-1, 0)

    def move_east(self):
        self.move(1, 0)


class World:
    def __init__(self, raw_world):
        self.obstacles = []
        self.obstacle_lookup = set()
        self.width = 0
        self.height = 0
        self.cell_size = 0
        self.start = None
        self.goal = None
        self.step_size = 0
        self.parse_world(raw_world)

    def __hash_cell(self, x, y):
        return int(y * self.width + x)

    def parse_world(self, raw_world):

        filtered_input = raw_world

        self.width = int(filtered_input.pop(0).strip())
        self.height = int(filtered_input.pop(0).strip())

        self.cell_size = int(min(MAX_X / self.width, MAX_Y / self.height))

        for y in range(0, self.height):
            line = filtered_input[y]

            for x in range(0, self.width):
                if line[x] is '#':
                    self.obstacles.append([x, y])
                    self.obstacle_lookup.add(self.__hash_cell(x, self.height - 1 - y))
                elif line[x] is '*':
                    self.goal = [x, y]
                elif line[x] is '@':
                    self.start = [x, y]

    def get_obstacles(self):
        return self.obstacles

    def draw(self, screen):
        if not screen:
            return

        if self.goal is not None:
            pygame.draw.rect(screen, BRAND_ORANGE, (
                self.goal[0] * self.cell_size,
                self.goal[1] * self.cell_size,
                self.cell_size,
                self.cell_size),
                             0)

        for obstacle in self.obstacles:
            pygame.draw.rect(screen, BRAND_DARK, (
                obstacle[0] * self.cell_size,
                obstacle[1] * self.cell_size,
                self.cell_size,
                self.cell_size),
                             0)

        pygame.draw.rect(screen, WHITE, (
            0,
            0,
            self.width * self.cell_size,
            self.height * self.cell_size),
                         1)

    def in_collision(self, location):
        hash_value = self.__hash_cell(pygame.math.floor(location[0]), math.floor(location[1]))
        return self.__out_of_range(location) or self.obstacle_lookup.__contains__(hash_value)

    def __out_of_range(self, location):
        return location[0] < 0 or location[1] < 0 or location[0] >= self.width or location[1] >= self.height


def initialize_world():
    world = World()
    filtered_input = world.parse_world()
    return world, filtered_input


def initialize_screen(world):
    pygame.init()
    pygame.display.set_caption('Metronome')

    print(world.height)
    print(world.width)
    print(world.cell_size)
    screen = pygame.display.set_mode((world.width * world.cell_size,
                                      world.height * world.cell_size))

    return screen


def handle_input(screen):
    for event in pygame.event.get():
        if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN and (
                                event.key == pygame.K_LEFT or event.key == pygame.K_RIGHT or event.key == pygame.K_UP or event.key == pygame.K_DOWN):
            return event.key


def main():
    raw_map = sys.stdin.readlines()

    world = World(raw_map)
    agent = Agent(world)

    screen = initialize_screen(world)

    while True:
        control = handle_input(screen)
        # if control is not None:
        #     observation = process_observation(control, agent, world, sigma)
        #     x, y, control = observation
        #     message = "%f %f %s\n" % (x, y, control)
        #     print("OUT:: " + message)
        #     process.stdin.write(message)
        #     particles = read_particles(args_particles, process)

        screen.fill(WHITE)
        world.draw(screen)
        # world.draw_particles(screen, particles, BRAND_ORANGE)
        agent.draw(screen, BRAND_LIGHT1)
        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
