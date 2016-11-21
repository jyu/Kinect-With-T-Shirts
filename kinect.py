from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *

import ctypes
import _ctypes
import pygame
import sys
import math

class Game(object):
    def __init__(self):
        pygame.init()
        self.initScreenVar()
        # screen updates
        self.clock = pygame.time.Clock()
        # set the width and height of the window half of width.height
        self.screen = pygame.display.set_mode(
            (960, 540),
            pygame.HWSURFACE | pygame.DOUBLEBUF,
            32
        )
        # exit game
        self.done = False
        # color and body frames from kinect runtime object
        self.kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body
        )

        self.bodies = None
        self.frameSurface = pygame.Surface(
            (
                self.kinect.color_frame_desc.Width,
                self.kinect.color_frame_desc.Height
            ),
            0,
            32
        )
        self.initBodyVar()

    def initScreenVar(self):
        # screen variables
        self.screenWidth = 1920
        self.screenHeight = 1080
        self.sensorScreenHeight = 1.2
        self.sensorScreenWidth = 3
        self.cornerToMiddleConstant = 1000
        self.shirtCompensationHeight = 30
        self.shirtCompensationWidth = 20

    def initBodyVar(self):
        # body variables
        self.yLeftShoulder = 0
        self.yRightShoulder = 0
        self.yLeftHip = 0
        self.yRightHip = 0
        self.xLeftShoulder = 0
        self.xRightShoulder = 0
        self.xLeftHip = 0
        self.xRightHip = 0
        self.xLeftElbow = 1
        self.yLeftElbow = 1
        self.xRightElbow = 1
        self.yRightElbow = 1

    def drawColorFrame(self, frame, target_surface):
        target_surface.lock()
        address = self.kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def sensorToScreenX(self, sensorPosX):
        screenX = (
            sensorPosX * (self.screenWidth / 3) +
            self.cornerToMiddleConstant +
            self.shirtCompensationWidth
        )
        return screenX

    def sensorToScreenY(self, sensorPosY):
        screenY = (
            -1 *
            (sensorPosY - self.sensorScreenHeight / 2) *
            (self.screenHeight / self.sensorScreenHeight)
        )
        return screenY

    def data(self, joints, type):
        ret = joints[getattr(PyKinectV2, "JointType_" + type)].Position
        return (ret.x, ret.y)

    def updateBody(self, joints):
        # update body trackers
        self.xLeftHip, self.yLeftHip = self.data(joints, "HipLeft")
        self.xRightHip, self.yRightHip = self.data(joints, "HipRight")
        (self.xLeftShoulder,
        self.yLeftShoulder) = self.data(joints, "ShoulderLeft")
        (self.xRightShoulder,
        self.yRightShoulder) = self.data(joints, "ShoulderRight")

    def updateArms(self, joints):
        # updates arms
        self.xLeftElbow, self.yLeftElbow = self.data(joints, "ElbowLeft")
        self.xRightElbow, self.yRightElbow = self.data(joints, "ElbowRight")

    def drawLeftArm(self):
        #left arm
        xShould = self.sensorToScreenX(self.xLeftShoulder)
        yShould = self.sensorToScreenY(self.yLeftShoulder) + 40
        xElb = self.sensorToScreenX(self.xLeftElbow)
        yElb = self.sensorToScreenY(self.yLeftElbow)
        theta = math.atan((yShould - yElb)/(xElb - xShould))
        thetaPrime = math.pi - math.pi/2 - theta
        if theta < 0: yElb += 80
        sleeveLength = 35
        if abs(theta) > math.pi/8 and abs(theta) < math.pi*3/8:
            #lower left arm
            leftY1 = yElb + sleeveLength * math.sin(thetaPrime)
            leftX1 = xElb + sleeveLength * math.cos(thetaPrime)
            leftY2 = yElb - sleeveLength * math.sin(thetaPrime)
            leftX2 = xElb - sleeveLength * math.cos(thetaPrime)
            #upper left arm
            leftY3 = yShould - sleeveLength * math.cos(theta)
            leftX3 = xShould - sleeveLength * math.sin(theta)
            leftY4 = yShould + sleeveLength * math.cos(theta)
            leftX4 = xShould + sleeveLength * math.sin(theta)
        else:
            #lower left arm
            leftY1 = yElb - sleeveLength * math.sin(thetaPrime)
            leftX1 = xElb + sleeveLength * math.cos(thetaPrime)
            leftY2 = yElb + sleeveLength * math.sin(thetaPrime)
            leftX2 = xElb - sleeveLength * math.cos(thetaPrime)
            #upper left arm
            leftY3 = yShould + sleeveLength * math.cos(theta)
            leftX3 = xShould - sleeveLength * math.sin(theta)
            leftY4 = yShould - sleeveLength * math.cos(theta)
            leftX4 = xShould + sleeveLength * math.sin(theta)

        pygame.draw.polygon(
            self.frameSurface,
            (0, 0, 200),
            [(leftX1, leftY1),(leftX2, leftY2),(leftX3,leftY3),(leftX4,leftY4)]
        )

        pygame.draw.rect(
            self.frameSurface,
            (200, 200, 0),
            (leftX1, leftY1, 20, 20)
        )
        pygame.draw.rect(
            self.frameSurface,
            (200, 200, 0),
            (leftX2, leftY2, 20, 20)
        )
        pygame.draw.rect(
            self.frameSurface,
            (200, 200, 0),
            (leftX3, leftY3, 20, 20)
        )
        pygame.draw.rect(
            self.frameSurface,
            (200, 200, 0),
            (leftX4, leftY4, 20, 20)
        )
    def drawRightArm(self):
        #left arm
        xShould = self.sensorToScreenX(self.xRightShoulder)
        yShould = self.sensorToScreenY(self.yRightShoulder) + 40
        xElb = self.sensorToScreenX(self.xRightElbow)
        yElb = self.sensorToScreenY(self.yRightElbow)
        theta = -1 * math.atan((yShould - yElb)/(xElb - xShould))
        thetaPrime = math.pi - math.pi/2 - theta
        if theta < 0: yElb += 80
        sleeveLength = 35
        if abs(theta) < math.pi/8 or abs(theta) > math.pi*3/8:
            #lower right arm
            leftY1 = yElb + sleeveLength * math.sin(thetaPrime)
            leftX1 = xElb + sleeveLength * math.cos(thetaPrime)
            leftY2 = yElb - sleeveLength * math.sin(thetaPrime)
            leftX2 = xElb - sleeveLength * math.cos(thetaPrime)
            #upper right arm
            leftY3 = yShould - sleeveLength * math.cos(theta)
            leftX3 = xShould - sleeveLength * math.sin(theta)
            leftY4 = yShould + sleeveLength * math.cos(theta)
            leftX4 = xShould + sleeveLength * math.sin(theta)
        else:
            #lower right arm
            leftY1 = yElb - sleeveLength * math.sin(thetaPrime)
            leftX1 = xElb + sleeveLength * math.cos(thetaPrime)
            leftY2 = yElb + sleeveLength * math.sin(thetaPrime)
            leftX2 = xElb - sleeveLength * math.cos(thetaPrime)
            #upper right arm
            leftY3 = yShould + sleeveLength * math.cos(theta)
            leftX3 = xShould - sleeveLength * math.sin(theta)
            leftY4 = yShould - sleeveLength * math.cos(theta)
            leftX4 = xShould + sleeveLength * math.sin(theta)

        pygame.draw.polygon(
            self.frameSurface,
            (0, 0, 200),
            [(leftX1, leftY1),(leftX2, leftY2),(leftX3,leftY3),(leftX4,leftY4)]
        )

        pygame.draw.rect(
            self.frameSurface,
            (200, 200, 0),
            (leftX1, leftY1, 20, 20)
        )
        pygame.draw.rect(
            self.frameSurface,
            (200, 200, 0),
            (leftX2, leftY2, 20, 20)
        )
        pygame.draw.rect(
            self.frameSurface,
            (200, 200, 0),
            (leftX3, leftY3, 20, 20)
        )
        pygame.draw.rect(
            self.frameSurface,
            (200, 200, 0),
            (leftX4, leftY4, 20, 20)
        )
    def drawBody(self):
        rightPart = (self.xRightShoulder + self.xRightHip) / 2
        leftPart = (self.xLeftShoulder + self.xLeftHip) / 2
        upPart = (self.yRightShoulder + self.yLeftShoulder) / 2
        downPart = (self.yRightHip + self.yLeftHip) / 2
        # converts sensor coords to pygame screen coords
        bodyX1 = self.sensorToScreenX(rightPart) + 20
        bodyY1 = self.sensorToScreenY(upPart)
        bodyX2 = self.sensorToScreenX(leftPart) - 20
        bodyY2 = self.sensorToScreenY(downPart)

        bodyWidth = bodyX2 - bodyX1
        bodyHeight = -1 * (bodyY1 - bodyY2) - self.shirtCompensationHeight

        pygame.draw.rect(
            self.frameSurface,
            (0, 200, 0),
            (bodyX1, bodyY1, bodyWidth, bodyHeight)
        )
        pygame.draw.rect(
            self.frameSurface,
            (200, 0, 0),
            (bodyX2, bodyY2, 20, 20)
        )
        pygame.draw.rect(
            self.frameSurface,
            (200, 0, 0),
            (bodyX1, bodyY1, 20, 20)
        )
        pygame.draw.rect(
            self.frameSurface,
            (200, 0, 0),
            (bodyX1, bodyY2, 20, 20)
        )
        pygame.draw.rect(
            self.frameSurface,
            (200, 0, 0),
            (bodyX2, bodyY1, 20, 20)
        )

    def runLoop(self):
        # pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False

        # reads and processes body frame from kinect
        if self.kinect.has_new_body_frame():
            self.bodies = self.kinect.get_last_body_frame()

            for i in range(self.kinect.max_body_count):
                body = self.bodies.bodies[i]
                if body.is_tracked:
                    joints = body.joints
                    self.updateBody(joints)
                    self.updateArms(joints)

        # reads color images from kinect
        if self.kinect.has_new_color_frame():
            frame = self.kinect.get_last_color_frame()
            self.drawColorFrame(frame, self.frameSurface)
            frame = None

        self.drawBody()
        self.drawLeftArm()
        self.drawRightArm()

        # changes ratio of image to output to window
        h_to_w = float(
            self.frameSurface.get_height() /
            self.frameSurface.get_width()
        )
        target_height = int(h_to_w * self.screen.get_width())
        surface_to_draw = pygame.transform.scale(
            self.frameSurface,
            (self.screen.get_width(), target_height)
        )
        self.screen.blit(surface_to_draw, (0,0))
        surface_to_draw = None
        pygame.display.update()

        self.clock.tick(60)
        return True

game = Game()
while game.runLoop():
    pass
game.kinect.close()
pygame.quit()
