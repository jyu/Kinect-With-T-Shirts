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

        # have (0,0) point be in the middle
        self.cX = int(surface.get_width() / 2)
        self.cY = int(surface.get_height() / 2)

        # Adding Distortion for z coordinate to be drawn
        # Larger z towards center
        # z best works from range 200 to 300
        self.drawX = int(self.x * ((self.screenHeight / 2) / self.z) +
                    self.cX)
        self.drawY = int(self.y * ((self.screenHeight / 2) / self.z) +
                    self.cY)


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
        # z dimension is half of the x and y dimensions
        zSide = length/4
        # points of the cube
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
        # edges of the cube (point1, point2)
        self.edges = [
                (0,1),(1,2),(2,3),(3,0),
                (4,5),(5,6),(6,7),(7,4),
                (0,4),(1,5),(2,6),(3,7)
                ]


    def draw(self, surface):
        #draws points
        for point in self.points:
            point.drawPoint()
        #draws edges
        for edge in self.edges:
            point1Index, point2Index = edge[0], edge[1]
            point1 = self.points[point1Index]
            point2 = self.points[point2Index]

            pygame.draw.line(
                surface,
                (255,0,0),
                (point1.drawX, point1.drawY),
                (point2.drawX, point2.drawY),
                20
                )











