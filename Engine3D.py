import pygame
import numpy as np
import math

# 3D engine

# Guided by tutorial's pseudocode
# https://gamedevelopment.tutsplus.com/tutorials/lets-build-a-3d-graphics-engine-points-vectors-and-basic-concepts--gamedev-8143
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

        self.updateDrawCoord()

    def convertTo3D(self,coord,center):
        # Adding Distortion for z coordinate to be drawn
        # Larger z towards center
        # z best works from range 100 to 300
        try: return int(coord * ((self.screenHeight / 2) / self.z) + center)
        except: return int(coord * ((self.screenHeight / 2) / (self.z+1)) + center)

    def updateDrawCoord(self):
        self.drawX = self.convertTo3D(self.x,self.cX)
        self.drawY = self.convertTo3D(self.y,self.cY)

    def drawPoint(self):
        pygame.draw.circle(
            self.surface,
            (255,0,0),
            (self.drawX,self.drawY),
            20
        )

class Cube(object):
    def __init__(self,x,y,z,length,surface):
        centerX = x
        centerY = y
        centerZ = z
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
        # draws points
        for point in self.points:
            point.drawPoint()
        # draws edges
        for edge in self.edges:
            point1Index, point2Index = edge[0], edge[1]
            point1, point2 = self.points[point1Index], self.points[point2Index]

            pygame.draw.line(
                surface,
                (255,0,0),
                (point1.drawX, point1.drawY),
                (point2.drawX, point2.drawY),
                20
                )

class Model(object):
    def __init__(self, surface):
        self.shapes = [Cube(0, 0, 0, 200, surface),
                      Cube(300, 150, 0, 100, surface)]
        self.surface = surface
        self.cam = Camera(self.shapes)

    def draw(self):
        for shape in self.shapes:
            shape.draw(self.surface)

class Camera(object):
    def __init__(self,shapes):
        self.shapes = shapes

    def keyPressed(self, key, model):
        # moves camera and changes view of the objects
        self.dX, self.dY, self.dZ = 0, 0, 0
        self.rotXY, self.rotXZ = 0,0
        step = 5
        radStep = .05
        # process keypresses
        if key[pygame.K_w]: self.dY -= step
        if key[pygame.K_s]: self.dY += step
        if key[pygame.K_a]: self.dX -= step
        if key[pygame.K_d]: self.dX += step
        if key[pygame.K_z]: self.dZ += step
        if key[pygame.K_x]: self.dZ -= step


        if key[pygame.K_o]: self.rotXY += radStep
        if key[pygame.K_p]: self.rotXY -= radStep

        if key[pygame.K_k]: self.rotXZ += radStep
        if key[pygame.K_l]: self.rotXZ -= radStep


        for shape in self.shapes:
           for point in shape.points:
                point.x += self.dX
                point.y += self.dY
                point.z += self.dZ
                (point.x, point.y, point.z) = self.rotate(
                    point.x, point.y, point.z,
                    self.rotXY,
                    "XY"
                    )
                (point.x, point.y, point.z) = self.rotate(
                    point.x, point.y, point.z,
                    self.rotXZ,
                    "XZ"
                    )
                point.updateDrawCoord()


    def rotate(self, x, y, z, radians, plane):
        orig = [x, y, z]
        c = math.cos(radians)
        s = math.sin(radians)
        if plane == "XY":
            # Matrix for transformation in XY plane
            rotMatrix = [
                [c, -s, 0],
                [s,  c, 0],
                [0,  0, 1]
                ]
        elif plane == "XZ":
            # Matrix for transformation in XZ plane
            rotMatrix = [
                [c,  0, s],
                [0,  1, 0],
                [-s, 0, c]
                ]
        return np.dot(orig,rotMatrix)
