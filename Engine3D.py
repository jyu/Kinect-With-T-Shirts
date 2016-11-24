import pygame

# 3D engine

# Guided by tutorial's pseudocode
# https://gamedevelopment.tutsplus.com/tutorials/lets-build-a-3d-graphics-engine-points-vectors-and-basic-concepts--gamedev-8143
# Watched this video for more info about 3D engines
# https://www.youtube.com/watch?v=g4E9iq0BixA
# Customized 3D engine for my project

minZ = 250

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

        # Adding Distortion for z coordinate to be drawn
        # Larger z towards center
        # z best works from range 200 to 300
        self.drawX = int(self.x * ((self.screenHeight / 2) / self.z) +
                    self.camX)
        self.drawY = int(self.y * ((self.screenHeight / 2) / self.z) +
                    self.camY)


    def drawPoint(self):

        pygame.draw.circle(
            self.surface,
            (255,0,0),
            (self.drawX,self.drawY),
            20
        )

class Cube(object):
    def __init__(self,pos,length,surface):
        centerX = pos[0]
        centerY = pos[1]
        centerZ = pos[2]
        side = length/2
        #z dimension is half of the x and y dimensions
        zSide = length/4
        self.points = [
          Point(centerX - side, centerY - side, centerZ - zSide + minZ, surface),
          Point(centerX + side, centerY - side, centerZ - zSide + minZ, surface),
          Point(centerX + side, centerY + side, centerZ - zSide + minZ, surface),
          Point(centerX - side, centerY + side, centerZ - zSide + minZ, surface),
          Point(centerX - side, centerY - side, centerZ + zSide + minZ, surface),
          Point(centerX + side, centerY - side, centerZ + zSide + minZ, surface),
          Point(centerX + side, centerY + side, centerZ + zSide + minZ, surface),
          Point(centerX - side, centerY + side, centerZ + zSide + minZ, surface)
          ]


    def draw(self, surface):
        for point in self.points:
            point.drawPoint()










