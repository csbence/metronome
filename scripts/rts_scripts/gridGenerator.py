#!/usr/bin/env python3

import argparse
import random
import sys
from math import sqrt

__author__ = 'Bence Cserna'

MIN_RADIUS = 7
MIN_LENGTH = 30


def create_image(height, width):
    return [["_" for _ in range(height)] for _ in range(width)]


def add_pixel(image, x, y):
    image[x][y] = "#"


def add_line(image, x1, y1, x2, y2):
    # Setup initial conditions
    # x1, y1 = start
    # x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        add_pixel(image, coord[0], coord[1])

        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points


def valid_points(x0, y0, x1, y1):
    # Exclude vertical and horizontal lines
    if x0 == x1 or y0 == y1:
        return False

    distance = sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    if distance < MIN_LENGTH:
        return False

    return True


def add_random_line(image, height, width):
    # Generate endpoints

    while True:
        x = [random.randint(0, width - 1), random.randint(0, width - 1)]
        y = [random.randint(0, height - 1), random.randint(0, height - 1)]

        x0 = min(x)
        x1 = max(x)
        y1 = y[0]
        y0 = y[1]

        if valid_points(x0, y0, x1, y1):
            break

    add_line(image, x0, y0, x1, y1)
    return x0, y0, x1, y1


def add_lines(image, numlines, height, width):
    lines = []
    for _ in range(numlines):
        lines.append(add_random_line(image, height, width))

    return lines


def add_circle(image, x0, y0, radius):
    x = radius
    y = 0
    decision_over = 1 - x  # Decision criterion divided by 2 evaluated at x=r, y=0

    while y <= x:
        add_pixel(image, x + x0, y + y0)  # Octant 1
        add_pixel(image, y + x0, x + y0)  # Octant 2
        add_pixel(image, -x + x0, y + y0)  # Octant 4
        add_pixel(image, -y + x0, x + y0)  # Octant 3
        add_pixel(image, -x + x0, -y + y0)  # Octant 5
        add_pixel(image, -y + x0, -x + y0)  # Octant 6
        add_pixel(image, x + x0, -y + y0)  # Octant 8
        add_pixel(image, y + x0, -x + y0)  # Octant 7
        y += 1
        if decision_over <= 0:
            decision_over += 2 * y + 1  # Change in decision criterion for y -> y+1
        else:
            x -= 1
            decision_over += 2 * (y - x) + 1  # Change for y -> y+1, x -> x-1


def add_random_circle(image, height, width):
    max_radius = 0
    while max_radius < MIN_RADIUS:
        x = random.randint(0, width - 1)
        y = random.randint(0, height - 1)

        max_x_radius = min(x, (width - 1 - x))
        max_y_radius = min(y, (height - 1 - y))
        max_radius = min(max_x_radius, max_y_radius)

    radius = random.randint(MIN_RADIUS, max_radius)
    add_circle(image, x, y, radius)

    return x, y, radius


def add_random_circles(image, numcircles, height, width):
    circles = []
    for _ in range(numcircles):
        circles.append(add_random_circle(image, height, width))

    return circles


def add_noisy_pixel(image, noise, x, y, height, width):
    random_probability = random.uniform(0, 100)

    if random_probability > noise:
        # Use the original point
        add_pixel(image, x, y)
    elif random_probability > noise / 3 * 2:
        # Move it to somewhere else
        add_pixel(image, random.randint(0, width - 1), random.randint(0, height - 1))
    elif random_probability > noise / 3:
        pass  # Remove point
    else:
        noisy_x = min(max(x + random.randint(-2, 2), 0), width - 1)
        noisy_y = min(max(y + random.randint(-2, 2), 0), height - 1)
        add_pixel(image, noisy_x, noisy_y)


def add_noise(image, noise, height, width):
    noisy_image = create_image(height, width)

    for y in range(height):
        for x in range(width):
            if image[x][y] in "#":
                add_noisy_pixel(noisy_image, noise, x, y, height, width)

    return noisy_image


def print_image(image, height, width):
    print(width)
    print(height)
    for y in range(height):
        for x in range(width):
            sys.stdout.write('%s' % image[x][y])
            sys.stdout.flush()
        print("")


def print_meta_info(lines, circles):
    print("/ Meta information: (Do not parse it!) ")
    print("// number of circles: %d" % len(circles))
    for circle in circles:
        print("// " + ' '.join(map(str, circle)))

    print("// number of lines: %d" % len(lines))
    for line in lines:
        print("// " + ' '.join(map(str, line)))


def main():
    circle_count, height, line_count, noise, width = parse_arguments()

    image = create_image(height, width)
    lines = add_lines(image, line_count, height, width)
    circles = add_random_circles(image, circle_count, height, width)
    noisy_image = add_noise(image, noise, height, width)
    print_meta_info(lines, circles)
    print_image(noisy_image, height, width)


def parse_arguments():
    parser = argparse.ArgumentParser(description="CS730/830 Assignment #11 - Image Generator")
    parser.add_argument("-he", "--height", required=True)
    parser.add_argument("-wi", "--width", required=True)
    parser.add_argument("-l", "--numlines", required=True)
    parser.add_argument("-c", "--numcircles", required=True)
    parser.add_argument("-n", "--noise", required=True)
    args = parser.parse_args()
    height = int(args.height)
    width = int(args.width)
    line_count = int(args.numlines)
    circle_count = int(args.numcircles)
    noise = float(args.noise)
    return circle_count, height, line_count, noise, width


if __name__ == "__main__":
    main()
