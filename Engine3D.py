import pygame
import numpy as np
import math
import bisect
import cv2
import os

# 3D engine
# Jerry Yu
# AndrewID: jerryy
# Section I

# Guided by pseudocode from this tutorial:
# https://gamedevelopment.tutsplus.com/tutorials/lets-build-a-3d-graphics-engine-points-vectors-and-basic-concepts--gamedev-8143
# Additional info about 3D engines from:
# https://www.youtube.com/watch?v=g4E9iq0BixA
# Customized 3D engine for my project
# Leaned homography from tutorial:
# http://www.learnopencv.com/homography-examples-using-opencv-python-c/

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

    def set(self,newX,newY,newZ,newView):
        x, y, z, view = newX, newY, newZ, newView

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

    def update(self, *args):
        pass

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
        for i in range(len(self.shapes)):
            self.shapes[i].draw()
        # for shape in self.shapes:
        #     shape.draw()

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
    def __init__(self,x,y,z,surface,colors,xSide=100,ySide=200,zSide=50):
        self.surface = surface
        # Position of cube
        centerX = x
        centerY = y
        centerZ = z
        self.initX, self.initY = x,y
        xSide = xSide
        ySide = ySide
        self.zSide = zSide
        self.initPoints(xSide,ySide,centerX,centerY,centerZ)
        self.initEdges()
        self.initFaces()
        self.colors = colors
        self.initImages()

    def initImages(self):
        mainPath = os.getcwd()
        os.chdir("sourcePictures")
        self.front = cv2.imread("ironManFront.png")
        self.frontPts = np.array([[0,0],[152,0],[152,255],[0,255]])
        os.chdir(mainPath)

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
         + self.initRightSleevePoints(xSide,ySide,centerX,centerY,centerZ)
         + self.initTopPoints(xSide,ySide,centerX,centerY,centerZ))

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

    def initTopPoints(self,xSide,ySide,centerX,centerY,centerZ):
        cX,cY = centerX,centerY
        sur = self.surface
        # Body
        return [
            Point(cX - xSide, cY - ySide, centerZ - self.zSide, sur),
            Point(cX + xSide, cY - ySide, centerZ - self.zSide, sur),
            Point(cX + 0.4 * xSide, cY - 1.2 * ySide, centerZ - self.zSide, sur),
            Point(cX - 0.4 * xSide, cY - 1.2 * ySide, centerZ - self.zSide, sur),
            Point(cX - xSide, cY - ySide, centerZ + self.zSide, sur),
            Point(cX + xSide, cY - ySide, centerZ + self.zSide, sur),
            Point(cX + 0.4 * xSide, cY - 1.2 * ySide, centerZ + self.zSide, sur),
            Point(cX - 0.4 * xSide, cY - 1.2 * ySide, centerZ + self.zSide, sur)
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
                    + self.initRightSleeveFaces()
                    + self.initTopFaces())

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

    def initTopFaces(self):
        return [
                (24,25,26,27),
                (28,29,30,31),
                (26,27,31,30),
                (24,25,29,28),
                (25,26,31,30),
                (24,27,31,29)
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

    def update(self, cX, cY, width, height, angleXZ, leftAng, rightAng, bodyZ):
        # Process rotation
        angleXZ *= math.pi/180.0
        # XYZ movement and sizing
        view = (int(cX), int(cY), bodyZ)
        xSide, ySide = width/2, height/2
        self.updateBody(xSide,ySide,angleXZ,view)
        self.updateLeftSleeve(xSide,ySide,leftAng,angleXZ,view)
        self.updateRightSleeve(xSide,ySide,rightAng,angleXZ,view)
        self.updateTop(xSide,ySide,angleXZ,view)

    def updateBody(self, xSide, ySide, angleXZ, view):
        # Body
        index = 0
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
                index += 1

    def updateLeftSleeve(self, xSide, ySide, leftAng, angleXZ, view):
        # Left Sleeve
        XOperations, YOperations = [2.1,.4,.4,2.1], [-0.6,-0.6,-1.1,-1.1]
        index = 6
        for zOp in [0.2,2]:
            for i in range(len(XOperations)):
                if leftAng >= 0.3: XOperations = [2.1,.5,.5,2.1]
                xOp, yOp = XOperations[i], YOperations[i]
                if leftAng > 0.8 and leftAng < 1.5: leftAng -= -1*(leftAng-1.5)*.07
                x, y, z = xOp * xSide, yOp * ySide, zOp * self.zSide
                # Translate to shoulder to rotate and translate back
                if leftAng >= 0.3: preX, preY = -30, 20
                elif leftAng <= -.6: preX, preY = 0, 100*leftAng + 40
                else: preX,preY = 0, 0
                x, y = x - xSide + preX, y + ySide + preY
                x, y, z = self.rotate(
                              x, y, z,
                              leftAng,
                              "XY"
                              )
                x, y = x + xSide - 30, y - ySide
                x, y, z = self.rotate(
                              x, y, z,
                              angleXZ,
                              "XZ"
                              )
                self.points.append(Point(x, y, z, self.surface, view))

    def updateRightSleeve(self, xSide, ySide, rightAng, angleXZ, view):
        # Right Sleeve
        index = 0
        XOperations, YOperations = [-2.1,-.4,-.4,-2.1], [-0.6,-0.6,-1.1,-1.1]
        for zOp in [.2,2]:
            for i in range(len(XOperations)):
                if rightAng >= 0.3: XOperations = [-2.1,-.5,-.5,-2.1]
                xOp, yOp = XOperations[i], YOperations[i]
                x, y, z = xOp * xSide, yOp * ySide, zOp * self.zSide
                # Translate to shoulder to rotate and translate back
                if rightAng >= 0.3: preX, preY = 30, 20
                elif rightAng <= -.6: preX, preY = 0, 100*rightAng + 40
                else: preX,preY = 0, 0
                x, y = x + xSide + preX, y + ySide + preY
                x, y, z = self.rotate(
                              x, y, z,
                               -rightAng,
                              "XY"
                              )
                # Lift torso side of sleeve
                if index in [1,2,5,6] and rightAng > 0:
                    yLift = -30
                else: yLift = -30
                x, y = x - xSide + 50, y - ySide + 30 + yLift
                x, y, z = self.rotate(
                              x, y, z,
                              angleXZ,
                              "XZ"
                              )
                self.points.append(Point(x, y, z, self.surface, view))
                index += 1

    def updateTop(self, xSide, ySide, angleXZ, view):
        # Right Sleeve
        index = 18
        XOperations, YOperations = [-1,1,0.4,-0.4], [-1,-1,-1.2,-1.2]
        for zOp in [0,2]:
            for i in range(len(XOperations)):
                xOp, yOp = XOperations[i], YOperations[i]
                x, y, z = xOp * xSide, yOp * ySide, zOp * self.zSide
                x, y, z = self.rotate(
                              x, y, z,
                              angleXZ,
                              "XZ"
                              )
                self.points.append(Point(x, y, z, self.surface, view))
                index += 1

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
        # Draw Faces
        self.drawFaces()

    def drawPoints(self):
        for point in self.points:
             point.draw()

    def drawEdges(self):
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
            color = self.colors[0]
            if faceIndex == 0 or faceIndex == 18: color = self.colors[1]
            if faceIndex >= 6 and faceIndex <= 17:
                color = self.colors[2]
            pygame.draw.polygon(
            self.surface,
            color,
            pointList
            )
        face = self.faces[0]
        pointList = []
        for pointIndex in face:
            point = self.points[pointIndex]
            pointList.append((point.drawX, point.drawY))
        color = self.colors[1]
        pygame.draw.polygon(
        self.surface,
        color,
        pointList
        )
        face = self.faces[18]
        pointList = []
        for pointIndex in face:
            point = self.points[pointIndex]
            pointList.append((point.drawX, point.drawY))
        color = self.colors[1]
        pygame.draw.polygon(
        self.surface,
        color,
        pointList
        )


    def getFrontFace(self):
        pointList = []
        face = self.faces[0]
        for pointIndex in face:
                point = self.points[pointIndex]
                pointList.append((point.drawX, point.drawY))
        return pointList

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
