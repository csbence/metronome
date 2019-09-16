import pygame
import sys
import time
import random
import numpy as np

from pygame.locals import *

FPS = 5
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 360
GRID_SIZE = 40
GRID_WIDTH = SCREEN_WIDTH / GRID_SIZE
GRID_HEIGHT = SCREEN_HEIGHT / GRID_SIZE

BUFFER = 20

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

cell_to_number = {
    '_': 0,  # CELL
    '#': 1,  # OBSTACLE
    '?': 2,  # FALSE_DYNAMIC_OBSTACLE
    '!': 3,  # TRUE_DYNAMIC_OBSTACLE
    '@': 4,  # START
    '*': 5,  # GOAl
}

number_to_color = {
    0: WHITE,
    1: BRAND_DARK,
    2: BRAND_LIGHT1,
    3: BRAND_LIGHT2,
    4: RED,
    5: GREEN,
}


def init_pygame():
    pygame.init()

    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
    surface = pygame.Surface(screen.get_size())
    surface = surface.convert()
    surface.fill((255, 255, 255))
    clock = pygame.time.Clock()
    screen.blit(surface, (0, 0))

    pygame.key.set_repeat(1, 50)

    return surface, screen, clock


def draw_box(surf, color, pos):
    r = pygame.Rect((pos[0] * GRID_SIZE, pos[1] * GRID_SIZE),
                    (GRID_SIZE, GRID_SIZE))
    pygame.draw.rect(surf, color, r)


class Obstacles(object):
    def __init__(self, path):
        self.obstacles = None
        self.color = (0, 0, 0)
        self.width = None
        self.height = None

        self.load_from_file(path)

    def load_from_file(self, path):
        lines = None
        with open(path, 'r') as file:
            lines = file.readlines()

        self.height = int(lines[1])
        self.width = int(lines[0])

        self.obstacles = np.zeros((self.width, self.height))

        for x in range(self.width):
            for y in range(self.height):
                symbol = lines[y + 2][x]
                cell = cell_to_number[symbol]
                self.obstacles[x][y] = cell

    def draw(self, surf, offset):
        for y in range(8):
            for x in range(offset, offset + BUFFER):
                if x > self.width:
                    x %= self.width
                cell = self.obstacles[x][y]
                if cell:
                    color = number_to_color[cell]
                    draw_box(surf, color, (x - offset, y))


class Agent(object):
    def __init__(self):
        self.position = (0, 0)
        self.color = (255, 0, 0)
        self.randomize()

    def randomize(self):
        self.position = (random.randint(0, GRID_WIDTH - 1) * GRID_SIZE,
                         random.randint(0, GRID_HEIGHT - 1) * GRID_SIZE)

    def draw(self, surf):
        draw_box(surf, self.color, self.position)


def draw_counter(surface, screen, number):
    font = pygame.font.Font(None, 36)
    text = font.render(str(number), 1, (10, 10, 10))
    textpos = text.get_rect()
    textpos.centerx = 20
    surface.blit(text, textpos)
    screen.blit(surface, (0, 0))


if __name__ == '__main__':
    surface, screen, clock = init_pygame()

    obstacles = Obstacles('../generator/generated_tracks/0.track')
    agent = Agent()

    position = 0

    while True:
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == KEYDOWN:
                if event.key == K_UP:
                    position += 10
                elif event.key == K_DOWN:
                    position -= 10
                elif event.key == K_LEFT:
                    position -= 1
                elif event.key == K_RIGHT:
                    position += 1

        surface.fill((255, 255, 255))
        obstacles.draw(surface, position)
        agent.draw(surface)
        draw_counter(surface, screen, position)

        pygame.display.flip()
        pygame.display.update()
        # clock.tick(FPS + obstacles.length / 3)
