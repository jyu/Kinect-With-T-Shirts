import pygame

# 3D engine

# Based on the tutorial's pseudocode
# https://gamedevelopment.tutsplus.com/tutorials/lets-build-a-3d-graphics-engine-points-vectors-and-basic-concepts--gamedev-8143
# Expanded to include custom features

class Point(object):

    def __init__(self, x, y, z, surface):

        self.x = x
        self.y = y
        self.z = z
        self.surface = surface
        self.screenHeight = surface.get_height()

        # have camera the so (0,0) is the middle
        self.camX = int(surface.get_width() / 2)
        self.camY = int(surface.get_height() / 2)

    def drawPoint(self):
        # Adding Distortion for z coordinate to be drawn
        # Larger z towards center
        x = int(self.x * ((self.screenHeight / 2) / self.z)) + self.camX
        y = int(self.y * ((self.screenHeight / 2) / self.z)) + self.camY

        pygame.draw.circle(
            self.surface,
            (255,0,0),
            (x,y),
            20
        )








