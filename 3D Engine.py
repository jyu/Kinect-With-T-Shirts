import pygame

# 3D engine

# Based on the tutorial's pseudocode
# https://gamedevelopment.tutsplus.com/tutorials/lets-build-a-3d-graphics-engine-points-vectors-and-basic-concepts--gamedev-8143
# Expanded to include custom features

class Point(object):

    def __init__(self, x, y, z, surface):
        self.coords = (x,y,z)
        self.surface = surface


    def drawPoint():
        pygame.draw.circle(
            self.surface,
            (255,0,0),
            self.coords,
            radius = 20
                        )





