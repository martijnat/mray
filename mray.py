#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (C) 2017  Martijn Terpstra

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import png
from sys import stdout, argv, stderr
from random import random
from math import sqrt, sin, cos, pi

esc_draw_rgb_bg = "\x1b[48;2;%i;%i;%im"
esc_draw_rgb_fg = "\x1b[38;2;%i;%i;%im"
half_block = "▄"
EPSILON = 0.000001
MAX_RAY_DEPTH = 8
MAX_RAY_LENGTH = 10**3
TERM_WIDTH = 80
TERM_HEIGHT = 24


if len(argv) > 3:
    WIDTH, HEIGHT = map(int, [argv[1], argv[2]])
    png_file_name = argv[3]
    TERM_HEIGHT_RATIO = 1 + HEIGHT // TERM_HEIGHT
    TERM_WIDTH_RATIO = 1 + WIDTH // TERM_WIDTH
else:
    stdout.write("Usage %s WIDTH HEIGHT FILENAME" % argv[0])
    quit(1)


def Moller_Trumbore(V1, V2, V3, O, D):
    "Möller–Trumbore intersection algorithm"
    # https://en.wikipedia.org/wiki/Möller–Trumbore_intersection_algorithm

    # Find vectors for two edges sharing V1
    e1 = V2 - V1
    e2 = V3 - V1
    # Begin calculating determinant - also used to calculate u parameter
    P = D.cross_product(e2)
    # if determinant is near zero, ray lies in plane of triangle or ray is
    # parallel to plane of triangle
    det = e1.dot(P)
    # NOT CULLING
    if (det > -EPSILON and det < EPSILON):
        return False

    inv_det = 1.0 / det

    # calculate distance from V1 to ray origin
    T = O - V1

    # Calculate u parameter and test bound
    u = T.dot(P) * inv_det
    # The intersection lies outside of the triangle
    if (u < 0.0 or u > 1.0):
        return False

    # Prepare to test v parameter
    Q = T.cross_product(e1)

    # Calculate V parameter and test bound
    v = D.dot(Q) * inv_det

    # The intersection lies outside of the triangle
    if(v < 0.0 or u + v > 1.0):
        return False

    t = e2.dot(Q) * inv_det

    if(t > EPSILON):
        return True

    # No hit, no win
    return False


class vec3():

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __mul__(self, other):
        return vec3(self.x * other, self.y * other, self.z * other)

    def __add__(self, other):
        return vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        return vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def dot(self, other):
        return (self.x * other.x) + (self.y * other.y) + (self.z * other.z)

    def __abs__(self):
        return self.dot(self)

    def __repr__(self):
        return "vec3(%f %f %f)" % (self.x, self.y, self.z)

    def length(self):
        return sqrt(abs(self))

    def cross_product(self, other):
        return vec3(self.y * other.z - self.z * other.y,
                    self.z * other.x - self.x * other.z,
                    self.x * other.y - self.y * other.x)

    def components(self):
        return self.x, self.y, self.z

    def normalize(self):
        size = sqrt(self.dot(self))
        if size > 0:
            return vec3(self.x / size, self.y / size, self.z / size)
        return vec3(random(), random(), random()).normalize()

    def transpose_mul(self, other):
        return vec3(self.x * other.x, self.y * other.y, self.z * other.z)


class Light():

    def __init__(self, position, color):
        self.position = position
        self.color = color

    def direction_to_pos(self, pos):
        return (self.position - pos).normalize()

    def clear_path(self, pos, scene, lights, ray_depth):
        max_dist = (self.position - pos).length()
        if ray_depth > MAX_RAY_DEPTH:
            return True
        for obj in scene:
            does_intersect, dist, obj_color = obj.intersection(
                pos, self.direction_to_pos(pos), scene, lights, ray_depth + 1)
            if does_intersect:
                if dist < max_dist:
                    return False
        return True

RED = vec3(1, 0, 0)
GREEN = vec3(0, 1, 0)
BLUE = vec3(0, 0, 1)
YELLOW = vec3(1, 1, 0)
MAGENTA = vec3(1, 0, 1)
CYAN = vec3(0, 1, 1)
WHITE = vec3(1, 1, 1)
BLACK = vec3(0, 0, 0)
GREY = vec3(0.5, 0.5, 0.5)
COLOR_BG = vec3(1 / 1.5, 1 / 1.2, 1 / 1.1)
COLOR_SKY = vec3(0.7, 0.6, 0.5)
ORANGE = vec3(1, 0.5, 0)
UNINTIALIZED = vec3(10, 0, 10)
EYE_POSITION = vec3(0.0, 0.0, -1.0)


class Sphere():

    def __init__(self, x, y, z, radius, color=WHITE, reflectivity=0):
        self.x = x
        self.y = y
        self.z = z
        self.radius = radius
        self.color = color
        self.reflectivity = reflectivity

    def get_color(self, pos):
        return self.color

    def intersection(self, ray_origin, ray_direction, scene, lights, ray_depth):
        NO_INTERSECTION = False, MAX_RAY_LENGTH, COLOR_BG
        # L: distance from ray_origin to sphere center
        L = vec3(self.x, self.y, self.z) - ray_origin
        # Project L onto the ray direction, this gives us tc
        tc = L.dot(ray_direction)
        if tc < 0:
            return NO_INTERSECTION
        # calculate tc's distance from center of sphere
        try:
            d = sqrt(L.length()**2 - tc**2)
        except ValueError:            # rounding error because of floats
            d = 0
        if d > self.radius:
            return NO_INTERSECTION

        # calculate the distance from tc to the edge of the sphere along the
        # ray
        t1c = sqrt(self.radius**2 - d**2)

        # we now have interections at the following distance
        t1 = tc - t1c           # front side intersection
        # t2 = tc + t1c           # back side intersection, unused

        intersection = ray_origin + (ray_direction * t1)
        normal = (intersection - vec3(self.x, self.y, self.z)).normalize()

        color = vec3(0, 0, 0)
        for light in lights:
            if light.clear_path(intersection, scene, [], ray_depth + 1):
                fullbright_color = self.get_color(
                    intersection).transpose_mul(light.color)
                angle = normal.dot(light.direction_to_pos(intersection))
                if angle > 0:
                    color += fullbright_color * angle

        if self.reflectivity > 0:
            bounce_start, bounce_direction = bounce_ray(
                intersection, ray_direction, normal)
            reflected_color = raytrace(
                bounce_start, bounce_direction, scene, lights, ray_depth + 1)
            color = color * (1 - self.reflectivity) + \
                (reflected_color * self.reflectivity)

        return True, t1, color


def bounce_ray(ray_origin, ray_direction, normal):
    bounce_direction = (ray_direction - normal * 2 *
                        ray_direction.dot(normal)).normalize()
    bounce_start = ray_origin + bounce_direction * EPSILON
    return bounce_start, bounce_direction


class Plane(Sphere):
    "A plane can be seen as a sphere with an infinite radius"

    def __init__(self, origin, normal, color=UNINTIALIZED, reflectivity=0):
        self.x, self.y, self.z = (
            (normal * -MAX_RAY_LENGTH) + origin).components()
        self.radius = MAX_RAY_LENGTH
        self.color = color
        self.reflectivity = reflectivity


class Triangle():

    def __init__(self, p1, p2, p3, color=UNINTIALIZED, reflectivity=0):
        self.p1, self.p2, self.p3 = p1, p2, p3
        self.center = (p1 * (1 / 3.0)) + (p2 * (1 / 3.0)) + (p3 * (1 / 3.0))
        self.color = color
        self.reflectivity = reflectivity
        self.normal = normal_from_triangle(p1, p2, p3)
        self.triangle_plane = Plane(self.center, self.normal)
        self.size = 200 * max(map(abs, [p1 - p2, p2 - p3, p1 - p3]))

    def get_color(self, pos):
        return self.color

    def intersection(self, ray_origin, ray_direction, scene, lights, ray_depth):
        NO_INTERSECTION = False, MAX_RAY_LENGTH, COLOR_BG

        hit = Moller_Trumbore(self.p1, self.p2, self.p3,
                              ray_origin, ray_direction)
        if not hit:
            return NO_INTERSECTION

        NO_INTERSECTION = False, MAX_RAY_LENGTH, COLOR_BG

        if self.normal.dot(ray_direction) > 0:
            # back face culling
            return NO_INTERSECTION

        # for edge in [self.edge12]:
        #     hit,dist,color = edge.intersection(ray_origin,ray_direction,[],[],ray_depth+1)
        #     if hit:
        #         return NO_INTERSECTION

        hit, dist, color = self.triangle_plane.intersection(
            ray_origin, ray_direction, [], [], ray_depth + 1)

        if not hit:
            return NO_INTERSECTION

        x, y, z = (self.center - (self.normal * self.size)).components()
        surface = Sphere(x, y, z, self.size, self.color, self.reflectivity)
        return surface.intersection(ray_origin, ray_direction, scene, lights, ray_depth + 1)


def normal_from_triangle(p1, p2, p3):
    U = p2 - p1
    V = p3 - p1

    Nx = U.y * V.z - U.z * V.y
    Ny = U.z * V.x - U.x * V.z
    Nz = U.x * V.y - U.y * V.x

    return vec3(Nx, Ny, Nz).normalize()


class CheckeredSphere(Sphere):

    def get_color(self, pos):
        density = 1 / 3
        x, y, z = map(lambda x: int(x * density) if x >=
                      0 else int((density * -x + 1)), pos.components())
        if (x + y + z) % 2 == 0:
            return self.color
        else:
            return (self.color * 0.5)


class CheckeredPlane(Plane):

    def get_color(self, pos):
        density = 10
        x, y, z = map(lambda x: int(x * density) if x >=
                      0 else int((density * -x + 1)), pos.components())
        if (x + y + z) % 2 == 0:
            return self.color
        else:
            return (self.color * 0.5)


def raytrace(origin, direction, scene, lights, ray_depth=1):
    if ray_depth > MAX_RAY_DEPTH:
        return COLOR_BG
    else:
        nearest_color = COLOR_BG
        nearest_dist = MAX_RAY_LENGTH
        for obj in scene:
            does_intersect, dist, obj_color = obj.intersection(
                origin, direction, scene, lights, ray_depth)
            if does_intersect:
                if dist < nearest_dist:
                    dist_ratio = dist / MAX_RAY_LENGTH
                    nearest_color = (obj_color * (1 - dist_ratio)
                                     ) + (COLOR_BG * (dist_ratio))
                    nearest_dist = dist
        return nearest_color


def raytrace_screen(y, x, scene, lights):
    screen_ratio = HEIGHT / WIDTH
    screen_intersect = vec3((2 * x / WIDTH - 1),
                            (-screen_ratio * (2 * y / HEIGHT - 1)), 0)
    screen_ray = (screen_intersect - EYE_POSITION).normalize()
    return raytrace(EYE_POSITION, screen_ray, scene, lights)


def load_obj(filename):
    "Parse an .obj file and return an array of Triangles"
    global draw_dist_min, draw_dist_max, zoomfactor
    vertices, normals, faces = [], [], []
    # each line represents 1 thing, we care only about
    # vertices(points) and faces(triangles)
    for linenumber, line in enumerate(open(filename).readlines()):
        c = line[0]
        if c == "v":            # vertices information
            if line[1] in "t":  # We ignore textures
                pass
            elif line[1] == "n":  # normals
                pass
            else:
                coords = list(map(float, line[1:-1].split()))
                # vertices.append(vec3(coords[0],coords[1],coords[2])) # wrong
                # direction
                vertices.append(vec3(coords[0], coords[1], -coords[2]))
        elif c == "f":          # face information
            normali = None
            if "/" in line:  # check for a/b/c syntax
                if "//" in line:  # check for a//b b//c c//d sumtax
                    indexes = [list(map(lambda x:int(x.split("//")[0]),
                                        miniline.split(" ")))[0] - 1
                               for miniline in line[2:-1].split()]
                else:
                    indexes = [list(map(int, miniline.split("/")))[0] - 1
                               for miniline in line[2:-1].split()]
            else:
                indexes = list(map(lambda x: (int(x) - 1), line[1:-1].split()))

            for i in range(0, len(indexes) - 2):
                p1, p2, p3 = vertices[indexes[0]], vertices[
                    indexes[i + 1]], vertices[indexes[i + 2]]
                yield p1, p3, p2  # mirror
        else:
            pass                # ignore all other information


lights_sky = [
    Light(vec3(MAX_RAY_LENGTH // 2, MAX_RAY_LENGTH, -MAX_RAY_LENGTH // 2), COLOR_SKY),
    Light(EYE_POSITION, COLOR_SKY),

]


carrot_point1 = vec3(-0.1, 1.3, 2.6)
carrot_point2 = vec3(0.1, 1.3, 2.6)
carrot_point3 = vec3(0.0, 1.4, 2.6)
carrot_point4 = vec3(-0.0, 1.0, 2.0)

scene_snowman = [
    # floor
    CheckeredSphere(0, -2 - MAX_RAY_LENGTH, 0, MAX_RAY_LENGTH, WHITE, 0.5),
    # body
    Sphere(0, -1, 3, 1, WHITE, 0.5),
    Sphere(0, 0.4, 3, 0.75, WHITE, 0.5),
    Sphere(0, 1.5, 3, 0.5, WHITE, 0.5),
    # arms
    Triangle(vec3(0.0, 0.25, 3.0), vec3(1.5, 1.4, 2.0),
             vec3(1.5, 1.0, 2.0), BLACK, 0.5),
    Triangle(vec3(-0.0, 0.25, 3.0), vec3(-1.5, 1.0, 2.0),
             vec3(-1.5, 1.4, 2.0), BLACK, 0.5),
    # eyes
    Sphere(-0.2, 1.5, 2.5, 0.1, GREY, 0.5),
    Sphere(0.2, 1.5, 2.5, 0.1, GREY, 0.5),
    # buttons
    Sphere(0.0, 0.6, 2.3, 0.1, BLACK, 0.5),
    Sphere(0.0, 0.2, 2.3, 0.1, BLACK, 0.5),
    Sphere(0.0, -0.4, 2.2, 0.1, BLACK, 0.5),
    # carrot nose
    Triangle(carrot_point2, carrot_point1, carrot_point4, ORANGE, 0.5),
    Triangle(carrot_point1, carrot_point3, carrot_point4, ORANGE, 0.5),
    Triangle(carrot_point3, carrot_point2, carrot_point4, ORANGE, 0.5),
    # colored spheres
    Sphere(2, -1.0, 5.0, 1.0, RED, 0.5),
    Sphere(-2, -1.0, 5.0, 1.0, GREEN, 0.5),
    Sphere(3, -1.0, 3.0, 1.0, YELLOW, 0.5),
    Sphere(-3, -1.0, 3.0, 1.0, BLUE, 0.5),
]

scene_test = [CheckeredSphere(
    0, -2 - MAX_RAY_LENGTH, 0, MAX_RAY_LENGTH, WHITE, 0.5)]

pixelmap = [[(255, 0, 255) if (x // 8 + y // 8) % 2 else (0, 0, 0)
             for x in range(WIDTH)] for y in range(HEIGHT)]

for y in range(0, HEIGHT, 1):
    for x in range(0, WIDTH, 1):
        r, g, b = raytrace_screen(y, x, scene_snowman, lights_sky).components()
        R, G, B = map(lambda x: min(255, max(0, int((x**2) * 255))), [r, g, b])
        if (x % TERM_WIDTH_RATIO) == 0 and (y % TERM_HEIGHT_RATIO) == 0:
            stdout.write(esc_draw_rgb_bg % (R, G, B))
            stdout.write(" ")
            # stdout.write(esc_draw_rgb_fg%(R2,G2,B2))
            stdout.write(esc_draw_rgb_bg % (0, 0, 0))
            stdout.write(esc_draw_rgb_fg % (255, 0, 0))
            stdout.flush()
        pixelmap[y][x] = R, G, B
    if (y % TERM_HEIGHT_RATIO) == 0:
        stdout.write("\n")


pngmap = [[subpixel for x in range(WIDTH)
           for subpixel in pixelmap[y][x]]
          for y in range(HEIGHT)]


output_file = open(png_file_name, 'wb')
w = png.Writer(WIDTH, HEIGHT, alpha=False)
w.write(output_file, pngmap)
output_file.close()
