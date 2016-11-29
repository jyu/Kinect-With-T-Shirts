import pygame
import numpy as np
import math
import bisect

# 3D engine

# Guided by pseudocode from this tutorial:
# https://gamedevelopment.tutsplus.com/tutorials/lets-build-a-3d-graphics-engine-points-vectors-and-basic-concepts--gamedev-8143
# Additional info about 3D engines from:
# https://www.youtube.com/watch?v=g4E9iq0BixA
# Customized 3D engine for my project

class Point(object):

    def __init__(self, x, y, z, surface, view=None):
        if view==None: view = [0,0,600]
        # Position coords
        self.x = x
        self.y = y
        self.z = z
        self.surface = surface
        self.screenHeight = surface.get_height()

        # View coords
        self.xView = view[0]
        self.yView = view[1]
        self.zView = view[2]

        # Have (0,0) point be in the middle
        self.cX = int(surface.get_width() / 2)
        self.cY = int(surface.get_height() / 2)

        # Intialize the coords where point is going to be drawn
        self.drawX, self.drawY, self.drawZ = 0,0,0
        self.updateDrawCoord()

    def updateViewCoord(self, newXView, newYView, newZView):
        self.xView = newXView
        self.yView = newYView
        self.zView = newZView

    def convertTo3D(self,coord,center):
        # Adding Distortion for z coordinate to be drawn
        # Larger z towards center
        if not self.drawZ == 0:
            return int(coord * ((self.screenHeight / 2) / self.drawZ) + center)
        else:
            return 1


    def updateDrawCoord(self):
        # Updates draw positions based on changes in position of the point
        self.drawZ = self.z + self.zView
        self.drawX = self.convertTo3D(self.x,self.cX) + self.xView
        self.drawY = self.convertTo3D(self.y,self.cY) + self.yView

    def __repr__(self):
        return "X:%d Y:%d Z:%d" % (self.drawX,self.drawY,self.drawZ)

    def draw(self):
        # Draws Circle at the point
        pygame.draw.circle(
            self.surface,
            (255,0,0),
            (self.drawX,self.drawY),
            20
        )


class Cube(object):
    def __init__(self,x,y,z,length,surface):
        # Position of cube
        centerX = x
        centerY = y
        centerZ = z
        # Z to start shape at
        minZ = 250
        side = length/2
        self.surface = surface
        self.initPoints(centerX,centerY,centerZ,minZ,side)
        self.initEdges()
        self.initFaces()

    def initPoints(self,centerX,centerY,centerZ,minZ,side):
        # Points of the cube
        self.points = [
         Point(centerX - side, centerY - side, centerZ - side, self.surface),
         Point(centerX + side, centerY - side, centerZ - side, self.surface),
         Point(centerX + side, centerY + side, centerZ - side, self.surface),
         Point(centerX - side, centerY + side, centerZ - side, self.surface),
         Point(centerX - side, centerY - side, centerZ + side, self.surface),
         Point(centerX + side, centerY - side, centerZ + side, self.surface),
         Point(centerX + side, centerY + side, centerZ + side, self.surface),
         Point(centerX - side, centerY + side, centerZ + side, self.surface)
         ]

    def initEdges(self):
        # Edges of the cube in the form (point1 index, point2 index)
        self.edges = [
                (0,1),(1,2),(2,3),(3,0),
                (4,5),(5,6),(6,7),(7,4),
                (0,4),(1,5),(2,6),(3,7)
                ]

    def initFaces(self):
        # Faces of the cube in the form (point1 index, point2 index,
        # point3 index, point4 index)
        self.faces = [
                (0,1,2,3),
                (4,5,6,7),
                (2,3,7,6),
                (0,1,5,4),
                (1,2,6,5),
                (0,3,7,4)
                ]

    def draw(self):
        # Draw points
        for point in self.points:
            point.draw()
        # Draw edges
        for edge in self.edges:
            point1Index, point2Index = edge[0], edge[1]
            point1, point2 = self.points[point1Index], self.points[point2Index]
            pygame.draw.line(
                self.surface,
                (255,0,0),
                (point1.drawX, point1.drawY),
                (point2.drawX, point2.drawY),
                20
                )
        self.drawFaces()

    def drawFaces(self):
        # Draw faces
        # Sort faces first so only visible faces are shown
        indicies = self.sortFacesByZ()
        for faceIndex in indicies:
            face = self.faces[faceIndex]
            pointList = []
            for pointIndex in face:
                point = self.points[pointIndex]
                pointList.append((point.drawX, point.drawY))
            if faceIndex in [0,1]: color = (200,200,0)
            elif faceIndex in [2,3]: color = (0,200,0)
            else: color = (0,0,200)
            pygame.draw.polygon(
            self.surface,
            color,
            pointList
            )

    def sortFacesByZ(self):
        # Sort the faces by their average Z value
        # High Z values are at the front, low z values at the back
        zValues = []
        zSortedIndices = []
        for faceIndex in range(len(self.faces)):
            face = self.faces[faceIndex]
            point1,point2 = self.points[face[0]],self.points[face[1]]
            point3,point4 = self.points[face[2]],self.points[face[3]]
            faceZ = (point1.z + point2.z + point3.z + point4.z) / 4
            # Use bisect to determine where to place the new value of Z
            index = bisect.bisect(zValues,faceZ)
            # Place value and index to corresponding list
            zValues.insert(index, faceZ)
            zSortedIndices.insert(index, faceIndex)
        # Bisect sorts in low Z to high Z, so reverse
        zSortedIndices.reverse()
        zValues.reverse()
        return zSortedIndices

class Model(object):
    # Model is the 3D model that holds all the different 3D shapes and camera
    # Overall "runner" of 3D engine
    def __init__(self, surface, shapes):
        self.shapes = shapes
        self.surface = surface
        self.cam = Camera(self.shapes)

    def draw(self):
        # Draws all shapes
        for shape in self.shapes:
            shape.draw()

class Camera(object):
    # Camera controls our view of all the objects
    def __init__(self,shapes):
        self.shapes = shapes

    def keyPressed(self, key, model):
        # Moves camera and changes view by keys
        self.dX, self.dY, self.dZ = 0, 0, 0
        self.rotXY, self.rotXZ, self.rotYZ = 0,0,0
        step = 5
        radStep = .05
        # Process keypresses for the x,y,z direction
        if key[pygame.K_w]: self.dY -= step
        if key[pygame.K_s]: self.dY += step
        if key[pygame.K_a]: self.dX -= step
        if key[pygame.K_d]: self.dX += step
        if key[pygame.K_z]: self.dZ += step
        if key[pygame.K_x]: self.dZ -= step

        # Process keypresses for xy, xz, rotation
        if key[pygame.K_o]: self.rotXY += radStep
        if key[pygame.K_p]: self.rotXY -= radStep
        if key[pygame.K_k]: self.rotXZ += radStep
        if key[pygame.K_l]: self.rotXZ -= radStep
        if key[pygame.K_n]: self.rotYZ += radStep
        if key[pygame.K_m]: self.rotYZ -= radStep

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
                (point.x, point.y, point.z) = self.rotate(
                    point.x, point.y, point.z,
                    self.rotYZ,
                    "YZ"
                    )
                point.updateDrawCoord()


    def rotate(self, x, y, z, radians, plane):
        orig = [x, y, z]
        c = math.cos(radians)
        s = math.sin(radians)

        # Rotation Matrices info is from:
        # https://gamedevelopment.tutsplus.com/tutorials/lets-build-a-3d-graphics-engine-linear-transformations--gamedev-7716
        # https://en.wikipedia.org/wiki/Rotation_matrix

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
        elif plane == "YZ":
            # Matrix for transformation in YZ plane
            rotMatrix = [
                [1, 0,  0],
                [0, c, -s],
                [0, s,  c]
                ]
        return np.dot(orig,rotMatrix)

class shirt(object):
    def __init__(self,x,y,z,surface):
        self.surface = surface
        # Position of cube
        centerX = x
        centerY = y
        centerZ = z

        xSide = 100
        ySide = 200
        self.zSide = 100
        self.initPoints(xSide,ySide,centerX,centerY,centerZ)
        self.initEdges()
        self.initFaces()

    def initPoints(self,xSide,ySide,centerX,centerY,centerZ):
        # Points of the cube
        cX,cY = centerX,centerY
        sur = self.surface
        # Body
        self.points = ([
            Point(cX - xSide, cY - ySide, centerZ - self.zSide, sur),
            Point(cX + xSide, cY - ySide, centerZ - self.zSide, sur),
            Point(cX + xSide, cY + ySide, centerZ - self.zSide, sur),
            Point(cX - xSide, cY + ySide, centerZ - self.zSide, sur),
            Point(cX - xSide, cY - ySide, centerZ + self.zSide, sur),
            Point(cX + xSide, cY - ySide, centerZ + self.zSide, sur),
            Point(cX + xSide, cY + ySide, centerZ + self.zSide, sur),
            Point(cX - xSide, cY + ySide, centerZ + self.zSide, sur)
            ]
         + self.initLeftSleevePoints(xSide,ySide,centerX,centerY,centerZ)
         + self.initRightSleevePoints(xSide,ySide,centerX,centerY,centerZ))

    def initLeftSleevePoints(self,xSide,ySide,centerX,centerY,centerZ):
        cX,cY,cZ = centerX,centerY,centerZ
        sur = self.surface
        zSid = self.zSide
        # Left Sleeve
        return [
             Point(cX - 2.5 * xSide, cY - 0.6 * ySide, cZ - .5 * zSid, sur),
             Point(cX - xSide      , cY - 0.6 * ySide, cZ - .5 * zSid, sur),
             Point(cX - xSide      , cY - ySide      , cZ - .5 * zSid, sur),
             Point(cX - 2.5 * xSide, cY - ySide      , cZ - .5 * zSid, sur),
             Point(cX - 2.5 * xSide, cY - 0.6 * ySide, cZ + .5 * zSid, sur),
             Point(cX - xSide      , cY - 0.6 * ySide, cZ + .5 * zSid, sur),
             Point(cX - xSide      , cY - ySide      , cZ + .5 * zSid, sur),
             Point(cX - 2.5 * xSide, cY - ySide      , cZ + .5 * zSid, sur)
             ]

    def initRightSleevePoints(self,xSide,ySide,centerX,centerY,centerZ):
        cX,cY,cZ = centerX,centerY,centerZ
        sur = self.surface
        zSid = self.zSide
        # Right Sleeve
        return [
             Point(cX + 2.5 * xSide, cY - 0.6 * ySide, cZ - .5 * zSid, sur),
             Point(cX + xSide      , cY - 0.6 * ySide, cZ - .5 * zSid, sur),
             Point(cX + xSide      , cY - ySide      , cZ - .5 * zSid, sur),
             Point(cX + 2.5 * xSide, cY - ySide      , cZ - .5 * zSid, sur),
             Point(cX + 2.5 * xSide, cY - 0.6 * ySide, cZ + .5 * zSid, sur),
             Point(cX + xSide      , cY - 0.6 * ySide, cZ + .5 * zSid, sur),
             Point(cX + xSide      , cY - ySide      , cZ + .5 * zSid, sur),
             Point(cX + 2.5 * xSide, cY - ySide      , cZ + .5 * zSid, sur)
             ]
    def initEdges(self):
        # Edges of the cube in the form (point1 index, point2 index)
        self.edges = ([
                # Body
                (0,1),(1,2),(2,3),(3,0),
                (4,5),(5,6),(6,7),(7,4),
                (0,4),(1,5),(2,6),(3,7)]
                + self.initLeftSleeveEdges()
                + self.initRightSleeveEdges())

    def initLeftSleeveEdges(self):
                # Left Sleeve
        return [
                (8,9),(9,10),(10,11),(11,8),
                (12,13),(13,14),(14,15),(15,12),
                (8,12),(9,14),(10,14),(11,15)
                ]
    def initRightSleeveEdges(self):
                # Right Sleeve
        return [
                (16,17),(17,18),(18,19),(19,16),
                (20,21),(21,22),(22,23),(23,20),
                (16,20),(17,22),(18,22),(19,23)
                ]

    def initFaces(self):
        # Faces of the cube in the form (point1 index, point2 index,
        # point3 index, point4 index)
        self.faces = ([
                    # Body
                    (0,1,2,3),
                    (4,5,6,7),
                    (2,3,7,6),
                    (0,1,5,4),
                    (1,2,6,5),
                    (0,3,7,4)
                    ]
                    + self.initLeftSleeveFaces()
                    + self.initRightSleeveFaces())

    def initLeftSleeveFaces(self):
                # Left Sleeve
        return [
                (8,9,10,11),
                (12,13,14,15),
                (10,11,15,14),
                (8,9,13,12),
                (9,10,14,13),
                (8,11,15,12)
                ]
    def initRightSleeveFaces(self):
                # Right Sleeve
        return [
                (16,17,18,19),
                (20,21,22,23),
                (18,19,23,22),
                (16,17,21,20),
                (17,18,23,22),
                (16,19,23,21)
                ]

        # Base Format of points for body
        # self.points = [
        #  Point(x - xSide, y - ySide, z - self.zSide, self.surface, view),
        #  Point(x + xSide, y - ySide, z - self.zSide, self.surface, view),
        #  Point(x + xSide, y + ySide, z - self.zSide, self.surface, view),
        #  Point(x - xSide, y + ySide, z - self.zSide, self.surface, view),
        #  Point(x - xSide, y - ySide, z + self.zSide, self.surface, view),
        #  Point(x + xSide, y - ySide, z + self.zSide, self.surface, view),
        #  Point(x + xSide, y + ySide, z + self.zSide, self.surface, view),
        #  Point(x - xSide, y + ySide, z + self.zSide, self.surface, view)
        #  ]

        # Body
        XYOperations = [(-1,-1),(1,-1),(1,1),(-1,1)]

        self.points = []
         # Goes through all operations for points
        for zOp in [0,2]:
            for xyOp in XYOperations:
                x = xyOp[0] * xSide
                y = xyOp[1] * ySide
                z = zOp * self.zSide
                x, y, z = self.rotate(
                              x, y, z,
                              angleXZ,
                              "XZ"
                              )
                self.points.append(Point(x, y, z, self.surface, view))

        # Left Sleeve

        XOperations = [2.5,1,1,2.5]
        YOperations = [-0.6,-0.6,-1,-1]

        for zOp in [.6,1.6]:
            for i in range(len(XOperations)):
                xOp = XOperations[i]
                yOp = YOperations[i]

                x = xOp * xSide
                y = yOp * ySide
                z = zOp * self.zSide
                x = x - xSide
                y = y + ySide - 20
                x, y, z = self.rotate(
                              x, y, z,
                              leftArmAngle,
                              "XY"
                              )
                x = x + xSide - 50
                y = y - ySide
                x, y, z = self.rotate(
                              x, y, z,
                              angleXZ,
                              "XZ"
                              )
                self.points.append(Point(x, y, z, self.surface, view))

    def rotate(self, x, y, z, radians, plane):
        orig = [x, y, z]
        c = math.cos(radians)
        s = math.sin(radians)

        # Rotation Matrices info is from:
        # https://gamedevelopment.tutsplus.com/tutorials/lets-build-a-3d-graphics-engine-linear-transformations--gamedev-7716
        # https://en.wikipedia.org/wiki/Rotation_matrix

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
        elif plane == "YZ":
            # Matrix for transformation in YZ plane
            rotMatrix = [
                [1, 0,  0],
                [0, c, -s],
                [0, s,  c]
                ]
        return np.dot(orig,rotMatrix)

    def draw(self):
        # Draw points
        for point in self.points:
            point.draw()


        # Draw edges
        for edge in self.edges:
            point1Index, point2Index = edge[0], edge[1]
            point1, point2 = self.points[point1Index], self.points[point2Index]
            pygame.draw.line(
                self.surface,
                (200,0,0),
                (point1.drawX, point1.drawY),
                (point2.drawX, point2.drawY),
                20
                )
        self.drawFaces()

    def drawFaces(self):
        # Draw faces

        # Sort faces so only visible faces are shown
        indicies = self.sortFacesByZ()
        for faceIndex in indicies:
            face = self.faces[faceIndex]
            pointList = []
            for pointIndex in face:
                point = self.points[pointIndex]
                pointList.append((point.drawX, point.drawY))
            color = (43, 156, 54)
            if faceIndex == 0: color = (200,0,0)
            if faceIndex >= 6:
                color = (0, 0, 200)
            pygame.draw.polygon(
            self.surface,
            color,
            pointList
            )

    def sortFacesByZ(self):
        # Sort the faces by their average Z value
        # High Z values are at the front, low z values at the back
        zValues = []
        zSortedIndices = []
        for faceIndex in range(len(self.faces)):
            face = self.faces[faceIndex]
            point1,point2 = self.points[face[0]],self.points[face[1]]
            point3,point4 = self.points[face[2]],self.points[face[3]]
            faceZ = (point1.z + point2.z + point3.z + point4.z) / 4
            # Use bisect to determine where to place the new value of Z
            index = bisect.bisect(zValues, faceZ)
            # Place value and index to corresponding list
            zValues.insert(index, faceZ)
            zSortedIndices.insert(index, faceIndex)
        # Bisect sorts in low Z to high Z, so reverse
        zSortedIndices.reverse()
        zValues.reverse()
        return zSortedIndices

class leftSleeve(object):
    def __init__(self,x,y,z,surface):
        self.surface = surface
        # Position of cube
        centerX = x
        centerY = y
        centerZ = z

        self.zSide = 100
        self.initPoints(centerX,centerY,centerZ)
        self.initEdges()
        self.initFaces()

    def initPoints(self,centerX,centerY,centerZ):
        # Points of the cube
        self.points = [
         Point(centerX, centerY, centerZ - self.zSide, self.surface),
         Point(centerX, centerY, centerZ - self.zSide, self.surface),
         Point(centerX, centerY, centerZ - self.zSide, self.surface),
         Point(centerX, centerY, centerZ - self.zSide, self.surface),
         Point(centerX, centerY, centerZ + self.zSide, self.surface),
         Point(centerX, centerY, centerZ + self.zSide, self.surface),
         Point(centerX, centerY, centerZ + self.zSide, self.surface),
         Point(centerX, centerY, centerZ + self.zSide, self.surface)
         ]

    def initEdges(self):
        # Edges of the cube in the form (point1 index, point2 index)
        self.edges = [
                (0,1),(1,2),(2,3),(3,0),
                (4,5),(5,6),(6,7),(7,4),
                (0,4),(1,5),(2,6),(3,7)
                ]

    def initFaces(self):
        # Faces of the cube in the form (point1 index, point2 index,
        # point3 index, point4 index)
        self.faces = [
                (0,1,2,3),
                (4,5,6,7),
                (2,3,7,6),
                (0,1,5,4),
                (1,2,6,5),
                (0,3,7,4)
                ]

    def update(self, centerX, centerY, endX, endY, theta, thetaPrime, sleeveLength, angleXZ):
        # Process rotation
        angleXZ *= math.pi/180.0
        # Process view, XYZ movement
        view = (int(centerX), int(centerY), 600)

        self.points = []
         # Goes through all operations for points
        for zOp in [-1,1]:
            z = zOp * self.zSide/2  + self.zSide
            sign = 1 if abs(theta) > math.pi/8 and abs(theta) < math.pi*3/8 else -1
            # Lower left arm
            leftY1 = endY + sleeveLength * math.sin(thetaPrime) * sign
            leftX1 = endX + sleeveLength * math.cos(thetaPrime)
            leftY2 = endY - sleeveLength * math.sin(thetaPrime) * sign
            leftX2 = endX - sleeveLength * math.cos(thetaPrime)
            # Upper left arm
            leftY3 = centerY - sleeveLength * math.cos(theta) * sign
            leftX3 = centerX - sleeveLength * math.sin(theta)
            leftY4 = centerY + sleeveLength * math.cos(theta) * sign
            leftX4 = centerX + sleeveLength * math.sin(theta)
            # Rotations
            leftX1,leftY1,leftZ1 = self.rotate(leftX1,leftY1,z,angleXZ,"XZ")
            leftX2,leftY2,leftZ2 = self.rotate(leftX2,leftY2,z,angleXZ,"XZ")
            leftX3,leftY3,leftZ3 = self.rotate(leftX3,leftY3,z,angleXZ,"XZ")
            leftX4,leftY4,leftZ4 = self.rotate(leftX4,leftY4,z,angleXZ,"XZ")

            self.points.append(Point(leftX1, leftY1, leftZ1, self.surface, view))
            self.points.append(Point(leftX4, leftY4, leftZ4, self.surface, view))
            self.points.append(Point(leftX3, leftY3, leftZ3, self.surface, view))
            self.points.append(Point(leftX2, leftY2, leftZ2, self.surface, view))

    def rotate(self, x, y, z, radians, plane):
        orig = [x, y, z]
        c = float(math.cos(radians))
        s = float(math.sin(radians))

        # Rotation Matrices info is from:
        # https://gamedevelopment.tutsplus.com/tutorials/lets-build-a-3d-graphics-engine-linear-transformations--gamedev-7716
        # https://en.wikipedia.org/wiki/Rotation_matrix

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
        elif plane == "YZ":
            # Matrix for transformation in YZ plane
            rotMatrix = [
                [1, 0,  0],
                [0, c, -s],
                [0, s,  c]
                ]
        return np.dot(orig,rotMatrix)

    def draw(self):
        # Draw points
        for point in self.points:
            point.draw()
        # Draw edges
        for edge in self.edges:
            point1Index, point2Index = edge[0], edge[1]
            point1, point2 = self.points[point1Index], self.points[point2Index]
            pygame.draw.line(
                self.surface,
                (255,0,0),
                (point1.drawX, point1.drawY),
                (point2.drawX, point2.drawY),
                20
                )
        self.drawFaces()

    def drawFaces(self):
        # Draw faces

        # Sort faces so only visible faces are shown
        indicies = self.sortFacesByZ()
        for faceIndex in indicies:
            face = self.faces[faceIndex]
            pointList = []
            for pointIndex in face:
                point = self.points[pointIndex]
                pointList.append((point.drawX, point.drawY))
            color = (61, 133, 198)
            pygame.draw.polygon(
            self.surface,
            color,
            pointList
            )

    def sortFacesByZ(self):
        # Sort the faces by their average Z value
        # High Z values are at the front, low z values at the back
        zValues = []
        zSortedIndices = []
        for faceIndex in range(len(self.faces)):
            face = self.faces[faceIndex]
            point1,point2 = self.points[face[0]],self.points[face[1]]
            point3,point4 = self.points[face[2]],self.points[face[3]]
            faceZ = (point1.z + point2.z + point3.z + point4.z) / 4
            # Use bisect to determine where to place the new value of Z
            index = bisect.bisect(zValues, faceZ)
            # Place value and index to corresponding list
            zValues.insert(index, faceZ)
            zSortedIndices.insert(index, faceIndex)
        # Bisect sorts in low Z to high Z, so reverse
        zSortedIndices.reverse()
        zValues.reverse()
        return zSortedIndices
