import pygame

# 3D engine

# Based on the tutorial's pseudocode
# https://gamedevelopment.tutsplus.com/tutorials/lets-build-a-3d-graphics-engine-points-vectors-and-basic-concepts--gamedev-8143
# Expanded to include custom features

class Point(object):

    def __init__(self, x, y, z, surface):

        self.x = x + surface.get_width() / 4
        self.y = y + surface.get_height() / 4
        self.z = z
        self.surface = surface


    def drawPoint(self):
        # Adding Distortion for z coordinate to be drawn
        # Larger z towards center
        x = int(self.x * (200 / self.z))
        y = int(self.y * (200 / self.z))

        pygame.draw.circle(
            self.surface,
            (255,0,0),
            (x,y),
            20
        )






