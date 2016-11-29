from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *

import ctypes
import _ctypes
import pygame
import sys
import math

from Engine3D import *

# Kinect Runner

# Guided by Kinect Workshop and FlapPyKinect
# https://onedrive.live.com/?authkey=%21AMWDgqPgtkPzsAM&id=ED75CBDC5E4AB0FE%211096749&cid=ED75CBDC5E4AB0FE

class Game(object):
    def __init__(self):
        pygame.init()
        self.initScreenVar()
        # screen updates
        self.clock = pygame.time.Clock()
        # set the width and height of the window to fit in screen
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

        self.model = Model(
                        self.frameSurface,
                        [
                        shirt(0, 0, 0, self.frameSurface)
                        ])

    def initScreenVar(self):
        # screen variables
        self.screenWidth = 1920
        self.screenHeight = 1080
        self.sensorScreenHeight = 1.2
        self.sensorScreenWidth = 3
        self.cornerToMiddleConstant = 1000
        self.shirtCompensationHeight = 50
        self.shirtCompensationWidth = 20

    def initBodyVar(self):
        # body variables
        self.yLeftShoulder = 0
        self.yRightShoulder = 0
        self.yLeftHip = 0
        self.yRightHip = 0
        self.xLeftShoulder = 0
        self.xRightShoulder = 0
        self.zRightShoulder = 0
        self.zLeftShoulder = 0
        self.xLeftHip = 0
        self.xRightHip = 0
        self.xLeftElbow = 1
        self.yLeftElbow = 1
        self.xRightElbow = 1
        self.yRightElbow = 1
        self.leftArmAngle = 0
        self.rightArmAngle = 0

    # Function from Kinect Workshop
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

    def data(self, joints, type, z=False):
        ret = joints[getattr(PyKinectV2, "JointType_" + type)].Position
        if z: return (ret.x,ret.y,ret.z)
        return (ret.x, ret.y)

    def updateBody(self, joints):
        # Update body trackers
        self.xLeftHip, self.yLeftHip = self.data(joints, "HipLeft")
        self.xRightHip, self.yRightHip = self.data(joints, "HipRight")
        (self.xLeftShoulder,
        self.yLeftShoulder,
        self.zLeftShoulder) = self.data(joints, "ShoulderLeft", True)
        (self.xRightShoulder,
        self.yRightShoulder,
        self.zRightShoulder) = self.data(joints, "ShoulderRight", True)

        self.updateBodyVars()

    def updateBodyVars(self):
        # XYZ movement calculations
        rightPart = (self.xRightShoulder + self.xRightHip) / 2
        leftPart = (self.xLeftShoulder + self.xLeftHip) / 2
        upPart = (self.yRightShoulder + self.yLeftShoulder) / 2
        downPart = (self.yRightHip + self.yLeftHip) / 2
        # Cnverts sensor coords to pygame screen coords
        bodyX1 = self.sensorToScreenX(rightPart) + 20
        bodyY1 = self.sensorToScreenY(upPart)
        bodyX2 = self.sensorToScreenX(leftPart) - 20
        bodyY2 = self.sensorToScreenY(downPart) - self.shirtCompensationHeight

        bodyCenterX = ((bodyX1 + bodyX2) / 2) - 960
        bodyCenterY = ((bodyY1 + bodyY2) / 2) - 540
        bodyWidth = bodyX2 - bodyX1
        bodyHeight = -1 * (bodyY1 - bodyY2)
        # Rotation calculations
        angleXZ = self.getAngleXZ()
        # Update body shape in model
        self.model.shapes[0].update(bodyCenterX,bodyCenterY,
                                    bodyWidth,bodyHeight,
                                    angleXZ,
                                    self.leftArmAngle,
                                    self.rightArmAngle)

    def getAngleXZ(self):
        # Compares shoulder width difference and depth differences to get angle
        # Convert to degrees for debugging and testing readibility
        return (math.atan2(self.zRightShoulder - self.zLeftShoulder,
                          self.xRightShoulder - self.xLeftShoulder)
                           * 180.0/math.pi)

    def updateArms(self, joints):
        # Updates arm variables
        self.xLeftElbow, self.yLeftElbow = self.data(joints, "ElbowLeft")
        self.xRightElbow, self.yRightElbow = self.data(joints, "ElbowRight")
        self.updateLeftArm()
        self.updateRightArm()

    def updateLeftArm(self):
        # left arm
        xShould = self.sensorToScreenX(self.xLeftShoulder)
        yShould = self.sensorToScreenY(self.yLeftShoulder) + 40
        xElb = self.sensorToScreenX(self.xLeftElbow)
        yElb = self.sensorToScreenY(self.yLeftElbow)

        try:
            theta = math.atan((yShould - yElb)/(xElb - xShould))
        except:
            theta = math.atan((yShould - yElb)/(xElb - xShould + 1))
        thetaPrime = math.pi - math.pi/2 - theta

        self.leftArmAngle = theta

    def updateRightArm(self):
        # right arm
        xShould = self.sensorToScreenX(self.xRightShoulder)
        yShould = self.sensorToScreenY(self.yRightShoulder) + 40
        xElb = self.sensorToScreenX(self.xRightElbow)
        yElb = self.sensorToScreenY(self.yRightElbow)
        try:
            theta = -1 * math.atan((yShould - yElb)/(xElb - xShould))
        except:
            theta = -1 * math.atan((yShould - yElb)/(xElb - xShould + 1))
        thetaPrime = math.pi - math.pi/2 - theta

        self.rightArmAngle = theta

    def drawRightArm(self):
        # right arm
        xShould = self.sensorToScreenX(self.xRightShoulder)
        yShould = self.sensorToScreenY(self.yRightShoulder) + 40
        xElb = self.sensorToScreenX(self.xRightElbow)
        yElb = self.sensorToScreenY(self.yRightElbow)
        try:
            theta = -1 * math.atan((yShould - yElb)/(xElb - xShould))
        except:
            theta = -1 * math.atan((yShould - yElb)/(xElb - xShould + 1))
        thetaPrime = math.pi - math.pi/2 - theta
        if theta < 0: yElb += 80
        sleeveLength = 35
        sign = 1 if abs(theta) < math.pi/8 or abs(theta) > math.pi*3/8 else -1
        # lower right arm
        leftY1 = yElb + sleeveLength * math.sin(thetaPrime) * sign
        leftX1 = xElb + sleeveLength * math.cos(thetaPrime)
        leftY2 = yElb - sleeveLength * math.sin(thetaPrime) * sign
        leftX2 = xElb - sleeveLength * math.cos(thetaPrime)
        # upper right arm
        leftY3 = yShould - sleeveLength * math.cos(theta) * sign
        leftX3 = xShould - sleeveLength * math.sin(theta)
        leftY4 = yShould + sleeveLength * math.cos(theta) * sign
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

        # Model Code Start
        key = pygame.key.get_pressed()
        if sum(key) > 0:
            self.model.cam.keyPressed(key, self.model)
        self.model.draw()
        #Model Code End

        # reads color images from kinect
        if self.kinect.has_new_color_frame():
            frame = self.kinect.get_last_color_frame()
            self.drawColorFrame(frame, self.frameSurface)
            frame = None

        self.model.draw()

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

    def run(self):
        while self.runLoop():
            pass
        self.kinect.close()
        pygame.quit()

game = Game()
game.run()

