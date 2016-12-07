
from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *

import ctypes
import _ctypes
import pygame
import sys
import math
import os

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
        self.trackedBodies = {}
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
                        ])
        self.closetModel = Model(
                                self.frameSurface,
                                [],
                                )
        self.initPics()
        self.initGUIVars()
        self.initDesignVars()

    def initDesignVars(self):
        self.redGradient = [(255,31,82), (245,29,75), (236,27,68), (226,25,61),
                            (217,24,54), (207,22,48), (198,20,41), (188,19,34),
                            (179,17,27), (169,15,20), (160,14,14)]
        self.greenGradient = [(162,229,0), (147,218,6), (132,208,12),
                              (118,198,18), (103,188,24), (89,178,30),
                              (74,167,36), (59,157,42), (45,147,48),
                              (30,137,54), (16,127,61)]
        self.blueGradient = [(23,232,253), (21,208,246), (20,185,240),
                             (18,162,234), (17,139,228), (16,116,222),
                             (14,92,215), (13,69,209), (11,46,203),
                             (10,23,197), (9,0,191)]

    def initGUIVars(self):
        self.MENU = 1
        self.CLOSET = 2
        self.DESIGN = 3
        self.CAMERA = 4
        self.DESIGNFRONT = 5
        self.DESIGNSIDES = 6
        self.DESIGNSLEEVES = 7
        self.FULLSCREEN = 8
        self.CAMERADONE = 9
        self.DESIGNFRONTMIX = 10
        self.DESIGNSIDESMIX = 11
        self.DESIGNSLEEVESMIX = 12
        self.mode = self.MENU
        self.nextColors = [(43, 156, 54),(200,  0,0),(61, 187, 198)]
        self.frontColor = [200, 0, 0]
        self.sidesColor = [43, 156, 54]
        self.sleevesColor = [61, 187, 198]
        self.initGUILocks()


    def initGUILocks(self):
        # Controls gui to make sure screens arent selected multiple times
        self.nextMode = None
        self.lock = [False, False, False, False, False, False]
        self.sign = 1
        self.flipLock = [False, False, False, False, False, False]
        self.designLock = [True, True, True, True, True, True]
        self.cameraStart = 0
        self.cameraTimer = 3000

    def initPics(self):
        mainPath = os.getcwd()
        os.chdir("sourcePictures")
        self.menu = pygame.image.load("menu.png")
        self.design = pygame.image.load("design.png")
        self.addMode = pygame.image.load("addMode.png")
        self.minusMode = pygame.image.load("minusMode.png")
        self.palette = pygame.image.load("palette.png")
        self.fullScreen = pygame.image.load("fullscreen.png")
        self.cameraDone = pygame.image.load("cameradone.png")
        self.designColors = pygame.image.load("designcolors.png")
        self.mix = pygame.image.load("mix.png")
        os.chdir(mainPath)
        self.screenshot = None
        self.tempScreenShot = None

    def initScreenVar(self):
        # screen variables
        self.screenWidth = 1920
        self.screenHeight = 1080
        self.sensorScreenHeight = 1.2
        self.sensorScreenWidth = 3
        self.cornerToMiddleConstant = 1000
        self.shirtCompensationHeight = 50
        self.shirtCompensationWidth = 0
        self.modelAngle = 20

    def initBodyVar(self):
        # body variables
        self.initShoulderHip()
        self.initArm()

    def initShoulderHip(self):
        self.yLeftShoulder = [0,0,0,0,0,0]
        self.xLeftShoulder = [0,0,0,0,0,0]
        self.zLeftShoulder = [0,0,0,0,0,0]
        self.xRightShoulder = [0,0,0,0,0,0]
        self.yRightShoulder = [0,0,0,0,0,0]
        self.zRightShoulder = [0,0,0,0,0,0]
        self.xLeftHip = [0,0,0,0,0,0]
        self.yLeftHip = [0,0,0,0,0,0]
        self.zLeftHip = [0,0,0,0,0,0]
        self.xRightHip = [0,0,0,0,0,0]
        self.yRightHip = [0,0,0,0,0,0]
        self.zRightHip = [0,0,0,0,0,0]


    def initArm(self):
        self.xLeftElbow = [1,1,1,1,1,1]
        self.yLeftElbow = [1,1,1,1,1,1]
        self.xRightElbow = [1,1,1,1,1,1]
        self.yRightElbow = [1,1,1,1,1,1]
        self.leftArmAngle = [0,0,0,0,0,0]
        self.rightArmAngle = [0,0,0,0,0,0]
        self.xRightHand = [0,0,0,0,0,0]
        self.yRightHand = [0,0,0,0,0,0]
        self.xLeftHand = [0,0,0,0,0,0]
        self.yLeftHand = [0,0,0,0,0,0]
        self.leftDownSwing = [True, True, True, True, True, True]
        self.rightDownSwing = [True, True, True, True, True, True]

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

    def updateBody(self, joints, i):
        # Update body trackers
        (self.xLeftHip[i],
        self.yLeftHip[i],
        self.zLeftHip[i]) = self.data(joints, "HipLeft", True)
        (self.xRightHip[i],
        self.yRightHip[i],
        self.zRightHip[i]) = self.data(joints, "HipRight", True)
        (self.xLeftShoulder[i],
        self.yLeftShoulder[i],
        self.zLeftShoulder[i]) = self.data(joints, "ShoulderLeft", True)
        (self.xRightShoulder[i],
        self.yRightShoulder[i],
        self.zRightShoulder[i]) = self.data(joints, "ShoulderRight", True)

        self.updateBodyVars(i)

    def updateBodyVars(self, i):
        # XYZ movement calculations
        rightPart = (self.xRightShoulder[i] + self.xRightHip[i]) / 2
        leftPart = (self.xLeftShoulder[i] + self.xLeftHip[i]) / 2
        upPart = (self.yRightShoulder[i] + self.yLeftShoulder[i]) / 2
        downPart = (self.yRightHip[i] + self.yLeftHip[i]) / 2
        zAvg = (self.zRightShoulder[i] + self.zLeftShoulder[i]
                + self.zRightHip[i] + self.zLeftHip[i]) / 4
        # Converts sensor coords to pygame screen coords
        bodyX1 = self.sensorToScreenX(rightPart) + 50
        bodyY1 = self.sensorToScreenY(upPart)
        bodyX2 = self.sensorToScreenX(leftPart) - 50
        bodyY2 = self.sensorToScreenY(downPart) - self.shirtCompensationHeight
        bodyZ = zAvg * 600/1.5
        #bodyZ = 600
        #print(zAvg)
        bodyCenterX = ((bodyX1 + bodyX2) / 2) - 960
        bodyCenterY = ((bodyY1 + bodyY2) / 2) - 540
        bodyWidth = bodyX2 - bodyX1
        bodyHeight = -1 * (bodyY1 - bodyY2)
        # Rotation calculations
        angleXZ = 3.8/5 * self.getAngleXZ(i)
        #print(angleXZ, bodyCenterX)
        #print(self.angleCorrection(bodyCenterX))
        angleXZ += self.angleCorrection(bodyCenterX)
        # Update body shape in model
        self.model.shapes[i].update(bodyCenterX,bodyCenterY,
                                    bodyWidth,bodyHeight,
                                    angleXZ,
                                    self.leftArmAngle[i],
                                    self.rightArmAngle[i],
                                    bodyZ)

    def angleCorrection(self, bodyX):
        # As person moves along the X, there is automatic angle added on because
        # of difference in Z of shoulders. This function corrects it so standing
        # straight on an X shold give an angleXZ of 0
        return -1 * (bodyX**3 * -4*10**-8 + bodyX**2 * 6*10**-6 -
                    bodyX * 0.0081)

    def getAngleXZ(self, i):
        # Compares shoulder width difference and depth differences to get angle
        # Convert to degrees for debugging and testing readibility
        return (math.atan2(self.zRightShoulder[i] - self.zLeftShoulder[i],
                          self.xRightShoulder[i] - self.xLeftShoulder[i])
                           * 180.0/math.pi)

    def updateArms(self, joints, i):
        # Updates arm variables
        self.xLeftElbow[i], self.yLeftElbow[i] = self.data(joints, "ElbowLeft")
        self.xRightElbow[i], self.yRightElbow[i] = self.data(joints, "ElbowRight")
        self.updateLeftArm(i)
        self.updateRightArm(i)

    def updateLeftArm(self, i):
        # left arm
        xShould = self.sensorToScreenX(self.xLeftShoulder[i])
        yShould = self.sensorToScreenY(self.yLeftShoulder[i]) + 40
        xElb = self.sensorToScreenX(self.xLeftElbow[i])
        yElb = self.sensorToScreenY(self.yLeftElbow[i])

        try:
            theta = math.atan((yShould - yElb)/(xElb - xShould))
        except:
            theta = math.atan((yShould - yElb)/(xElb - xShould + .01))
        thetaPrime = math.pi - math.pi/2 - theta

        if theta <= 0 and theta >= -1.4: self.leftDownSwing[i] = False
        elif theta >= 0: self.leftDownSwing[i] = True
        if theta < -1.4 and self.leftDownSwing[i]:
            theta = 1.57

        self.leftArmAngle[i] = theta

    def updateRightArm(self, i):
        # right arm
        xShould = self.sensorToScreenX(self.xRightShoulder[i])
        yShould = self.sensorToScreenY(self.yRightShoulder[i]) + 40
        xElb = self.sensorToScreenX(self.xRightElbow[i])
        yElb = self.sensorToScreenY(self.yRightElbow[i])
        try:
            theta = -1 * math.atan((yShould - yElb)/(xElb - xShould))
        except:
            theta = -1 * math.atan((yShould - yElb)/(xElb - xShould + .01))
        thetaPrime = math.pi - math.pi/2 - theta

        if theta <= 0 and theta >= -1.4: self.rightDownSwing[i] = False
        elif theta >= 0: self.rightDownSwing[i] = True
        if theta < -1.4 and self.rightDownSwing[i]:
            theta = 1.57
        self.rightArmAngle[i] = theta

    def updateHands(self, joints, i):
        self.xRightHand[i], self.yRightHand[i] = self.data(joints, "HandRight")
        self.xLeftHand[i], self.yLeftHand[i] = self.data(joints, "HandLeft")

    def updateAllGUI(self):
        for body in self.trackedBodies:
            i = self.trackedBodies[body][0]
            self.updateGUI(i)

    def updateGUI(self, i):
        rHandX = self.sensorToScreenX(self.xRightHand[i])
        rHandY = self.sensorToScreenY(self.yRightHand[i])
        lHandX = self.sensorToScreenX(self.xLeftHand[i])
        lHandY = self.sensorToScreenY(self.yLeftHand[i])

        # Draw Hands
        # pygame.draw.rect(
        #     self.frameSurface,
        #     (0,0,200),
        #     (rHandX, rHandY, 100, 100)
        # )
        # pygame.draw.rect(
        #     self.frameSurface,
        #     (200,200,0),
        #     (lHandX, lHandY, 100, 100)
        # )

        # Exit Button
        if lHandX < 200 and lHandY < 200:
            self.done = True
        # Fullscreen Button
        elif lHandX < 400 and lHandY > 880:
            if self.mode == self.CLOSET:
                self.closetModel.shapes.pop()
                self.closetModel.shapes.pop()
                self.closetModel.shapes.pop()
            self.mode = self.FULLSCREEN
        # Back to menu, gesture
        elif abs(lHandX-rHandX) <= 20 and abs(lHandY-rHandY) <= 20 and rHandY > 30:
            if self.mode == self.CLOSET:
                self.closetModel.shapes.pop()
                self.closetModel.shapes.pop()
                self.closetModel.shapes.pop()
            elif self.mode == self.CAMERA:
                self.cameraStart = 0
            elif self.mode == self.CAMERADONE:
                self.screenshot = None
                self.tempScreenshot = None
            self.mode = self.MENU
        # Update all modes
        if self.mode == self.MENU: self.updateMenu(rHandX,rHandY,i)
        elif self.mode == self.CLOSET: self.updateCloset(rHandX, rHandY, lHandY)
        elif self.mode == self.CAMERA: self.updateCamera()
        elif self.mode == self.CAMERADONE: self.updateCameraDone(rHandX,rHandY,lHandY,i)
        elif self.mode == self.DESIGN: self.updateDesign(rHandX,rHandY,lHandY,i)
        elif self.mode == self.DESIGNFRONT:
            self.updateFront(rHandX,rHandY,lHandY,lHandX,i)
        elif self.mode == self.DESIGNFRONTMIX:
            self.updateMixFront(rHandX,rHandY,lHandY,i)
        elif self.mode == self.DESIGNSIDES:
            self.updateSides(rHandX,rHandY,lHandY,lHandX,i)
        elif self.mode == self.DESIGNSIDESMIX:
            self.updateMixSides(rHandX,rHandY,lHandY,i)
        elif self.mode == self.DESIGNSLEEVES:
            self.updateSleeves(rHandX,rHandY,lHandY,lHandX,i)
        elif self.mode == self.DESIGNSLEEVESMIX:
            self.updateMixSleeves(rHandX,rHandY,lHandY,i)

    def updateCameraDone(self,rHandX,rHandY,lHandY,i):
        pygame.draw.rect(
                self.frameSurface,
                (200,200,200),
                ((288-40)-30,(162-40),2*(672+40),2*(378+40))
                )
        if rHandX < 1300: self.lock[i] = False
        if rHandX >= 1620 and not self.lock[i]:
            mainPath = os.getcwd()
            os.chdir("screenshots")
            if rHandY <= 360:
                screenshotCount = 0
                for fileName in os.listdir("."):
                    if fileName.startswith("picture"):
                        if int(fileName[7:-4]) >= screenshotCount:
                            screenshotCount = int(fileName[7:-4]) + 1
                    if fileName.startswith("screenshot"):
                        name = "picture%d.png" % (screenshotCount)
                        os.rename(fileName, name)
                self.screenshot = None
                self.tempScreenshot = None
                self.nextMode = self.MENU
            elif rHandY > 360 and rHandY < 720:
                os.remove("screenshot.png")
                self.screenshot = None
                self.tempScreenshot = None
                self.nextMode = self.MENU
            elif rHandY < 1080:
                os.remove("screenshot.png")
                self.screenshot = None
                self.tempScreenshot = None
                self.nextMode = self.CAMERA
            self.mode = self.nextMode
            self.lock[i] = True
            os.chdir(mainPath)

    def updateCamera(self):
        print(self.cameraStart, pygame.time.get_ticks())
        if self.cameraStart == 0:
            self.cameraStart = pygame.time.get_ticks()
        if pygame.time.get_ticks() - self.cameraStart >= self.cameraTimer:
            mainPath = os.getcwd()
            os.chdir("screenshots")
            pygame.image.save(self.screen,"screenshot.png")
            os.chdir(mainPath)
            self.cameraStart = 0
            self.mode = self.CAMERADONE
        timeLeft = self.cameraTimer - (pygame.time.get_ticks() - self.cameraStart)
        if timeLeft > 100:
            pygame.draw.rect(
                self.frameSurface,
                (0,200,0),
                (0,0,(timeLeft/self.cameraTimer)*1820,50)
                )

    def updateMenu(self,rHandX,rHandY,i):
        # Menu processing
        if rHandX < 1300: self.lock[i] = False
        # Right Panel
        if rHandX >= 1520 and not self.lock[i]:
            self.lock[i] = True
            if rHandY <= 360:
                self.mode = self.CLOSET
                self.closetModel.shapes.extend(
                    [
                    shirt(800, -400, 0, self.frameSurface,
                        [(43, 156, 54),(200,0,0),(61, 187, 198)],
                        60,100,10),
                    shirt(800, 0, 0, self.frameSurface,
                        [(200,0,0),(61, 187, 198),(43, 156, 54)],
                        60,100,10),
                    shirt(800, 400, 0, self.frameSurface,
                        [(61, 187, 198),(43, 156, 54),(200,0,0)],
                        60,100,10)
                    ])

            elif rHandY > 360 and rHandY < 720: self.mode = self.DESIGN
            elif rHandY < 1080: self.mode = self.CAMERA

    def updateCloset(self,rHandX,rHandY,lHandY):
        # Right Panel
        if rHandX >= 1520:
            if rHandY <= 360:
                self.nextColors = [(43, 156, 54),(200,0,0),(61, 187, 198)]
            elif rHandY > 360 and rHandY < 720:
                self.nextColors = [(200,0,0),(61, 187, 198),(43, 156, 54)]
            elif rHandY < 1080:
                self.nextColors = [(61, 187, 198),(43, 156, 54),(200,0,0)]
        # Change Colors
        if rHandY < 30 and lHandY < 30:
            for shape in self.model.shapes:
                shape.colors = self.nextColors
        # Rotation of Closet
        # self.modelAngle += 1
        # for i in range(len(self.model.shapes)):
        #     if i == 0: pass
        #     else:
        #         shirt = self.model.shapes[i]
        #         shirt.update(shirt.initX,shirt.initY,100,150,self.modelAngle,60,60)

    def updateDesign(self, rHandX, rHandY, lHandY, i):
        # Right Panel
        if rHandX >= 1520 and not self.lock[i]:
            self.lock[i] = True
            if rHandY <= 360:
                self.nextMode = self.DESIGNFRONT
            elif rHandY > 360 and rHandY < 720:
                self.nextMode = self.DESIGNSIDES
            elif rHandY < 1080:
                self.nextMode = self.DESIGNSLEEVES
            self.mode = self.nextMode
            self.lock[i] = True

        if rHandX < 1300 and self.lock[i] and self.nextMode != None:
            self.nextMode = None
        if rHandX < 1300: self.lock[i] = False

        for shape in self.model.shapes:
                shape.colors[0] = tuple(self.sidesColor)
                shape.colors[1] = tuple(self.frontColor)
                shape.colors[2] = tuple(self.sleevesColor)

    def updateSleeves(self, rHandX, rHandY, lHandY, lHandX, i):
        step = 154.4
        if lHandX < 200 and lHandY <= 366 and lHandY > 200:
            self.mode = self.DESIGNSLEEVESMIX
        elif rHandX >= 1520:
            if not self.lock[i]:
                if rHandY <= step: self.sleevesColor = [237,28,36]
                elif rHandY <= step*2: self.sleevesColor = [255,127,39]
                elif rHandY <= step*3: self.sleevesColor = [255,242,0]
                elif rHandY <= step*4: self.sleevesColor = [34,177,76]
                elif rHandY <= step*5: self.sleevesColor = [0,162,232]
                elif rHandY <= step*6: self.sleevesColor = [63,72,204]
                elif rHandY <= step*7: self.sleevesColor = [163,73,164]
            self.lock[i] = True
            for shape in self.model.shapes:
                shape.colors[2] = tuple(self.sleevesColor)

        if rHandX <= 1400:
            self.lock[i] = False

    def updateSides(self, rHandX, rHandY, lHandY, lHandX, i):
        step = 154.4
        if lHandX < 200 and lHandY <= 366 and lHandY > 200:
            self.mode = self.DESIGNSIDESMIX
        elif rHandX >= 1520:
            if not self.lock[i]:
                if rHandY <= step: self.sidesColor = [237,28,36]
                elif rHandY <= step*2: self.sidesColor = [255,127,39]
                elif rHandY <= step*3: self.sidesColor = [255,242,0]
                elif rHandY <= step*4: self.sidesColor = [34,177,76]
                elif rHandY <= step*5: self.sidesColor = [0,162,232]
                elif rHandY <= step*6: self.sidesColor = [63,72,204]
                elif rHandY <= step*7: self.sidesColor = [163,73,164]
            self.lock[i] = True
            for shape in self.model.shapes:
                shape.colors[0] = tuple(self.sidesColor)

        if rHandX <= 1400:
            self.lock[i] = False

    def updateFront(self, rHandX, rHandY, lHandY, lHandX, i):
        step = 154.4
        if lHandX < 200 and lHandY <= 366 and lHandY > 200:
            self.mode = self.DESIGNFRONTMIX
        elif rHandX >= 1520:
            if not self.lock[i]:
                if rHandY <= step: self.frontColor = [237,28,36]
                elif rHandY <= step*2: self.frontColor = [255,127,39]
                elif rHandY <= step*3: self.frontColor = [255,242,0]
                elif rHandY <= step*4: self.frontColor = [34,177,76]
                elif rHandY <= step*5: self.frontColor = [0,162,232]
                elif rHandY <= step*6: self.frontColor = [63,72,204]
                elif rHandY <= step*7: self.frontColor = [163,73,164]
            self.lock[i] = True
            for shape in self.model.shapes:
                shape.colors[1] = tuple(self.frontColor)

        if rHandX <= 1400:
            self.lock[i] = False

    def updateMixSleeves(self, rHandX, rHandY, lHandY, i):
        # Flip sign gesture
        if (abs(rHandY-lHandY) >= 900 and not self.flipLock[i]
            and rHandX < 1520):
            self.sign *= -1
            self.flipLock[i] = True
        if abs(rHandY-lHandY) <= 500 and self.flipLock[i]:
            self.flipLock[i] = False
        # Right Panel
        if rHandX >= 1520:
            if not self.lock[i]:
                if rHandY <= 360:
                    self.sleevesColor[0] += 20 * self.sign
                elif rHandY > 360 and rHandY < 720:
                    self.sleevesColor[1] += 20 * self.sign
                elif rHandY < 1080:
                    self.sleevesColor[2] += 20 * self.sign
            self.lock[i] = True
        if rHandX <= 1400:
            self.lock[i] = False
        self.sleevesColor[0] = min(255, self.sleevesColor[0])
        self.sleevesColor[1] = min(255, self.sleevesColor[1])
        self.sleevesColor[2] = min(255, self.sleevesColor[2])
        self.sleevesColor[0] = max(0, self.sleevesColor[0])
        self.sleevesColor[1] = max(0, self.sleevesColor[1])
        self.sleevesColor[2] = max(0, self.sleevesColor[2])

        for shape in self.model.shapes:
            shape.colors[2] = tuple(self.sleevesColor)

    def updateMixSides(self, rHandX, rHandY, lHandY, i):
        # Flip sign gesture
        if (abs(rHandY-lHandY) >= 900 and not self.flipLock[i]
            and rHandX < 1520):
            self.sign *= -1
            self.flipLock[i] = True
        if abs(rHandY-lHandY) <= 500 and self.flipLock[i]:
            self.flipLock[i] = False
        # Right Panel
        if rHandX >= 1520:
            if not self.lock[i]:
                if rHandY <= 360:
                    self.sidesColor[0] += 20 * self.sign
                elif rHandY > 360 and rHandY < 720:
                    self.sidesColor[1] += 20 * self.sign
                elif rHandY < 1080:
                    self.sidesColor[2] += 20 * self.sign
            self.lock[i] = True
        if rHandX <= 1400:
            self.lock[i] = False
        self.sidesColor[0] = min(255, self.sidesColor[0])
        self.sidesColor[1] = min(255, self.sidesColor[1])
        self.sidesColor[2] = min(255, self.sidesColor[2])
        self.sidesColor[0] = max(0, self.sidesColor[0])
        self.sidesColor[1] = max(0, self.sidesColor[1])
        self.sidesColor[2] = max(0, self.sidesColor[2])

        for shape in self.model.shapes:
            shape.colors[0] = tuple(self.sidesColor)

    def updateMixFront(self, rHandX, rHandY, lHandY, i):
        # Flip sign gesture
        if (abs(rHandY-lHandY) >= 900 and not self.flipLock[i]
            and rHandX < 1520):
            self.sign *= -1
            self.flipLock[i] = True
        if abs(rHandY-lHandY) <= 500 and self.flipLock[i]:
            self.flipLock[i] = False
        # Right Panel
        if rHandX >= 1520:
            if not self.lock[i]:
                if rHandY <= 360:
                    self.frontColor[0] += 20 * self.sign
                elif rHandY > 360 and rHandY < 720:
                    self.frontColor[1] += 20 * self.sign
                elif rHandY < 1080:
                    self.frontColor[2] += 20 * self.sign
            self.lock[i] = True
        if rHandX <= 1400:
            self.lock[i] = False
        self.frontColor[0] = min(255, self.frontColor[0])
        self.frontColor[1] = min(255, self.frontColor[1])
        self.frontColor[2] = min(255, self.frontColor[2])
        self.frontColor[0] = max(0, self.frontColor[0])
        self.frontColor[1] = max(0, self.frontColor[1])
        self.frontColor[2] = max(0, self.frontColor[2])

        for shape in self.model.shapes:
            shape.colors[1] = tuple(self.frontColor)

    def drawGUI(self):
        if self.mode == self.FULLSCREEN or self.mode == self.CAMERA: return
        # Exit
        pygame.draw.rect(
            self.frameSurface,
            (250,0,0),
            (0, 0, 200, 200)
        )
        pygame.draw.lines(
            self.frameSurface,
            (255,255,255),
            False,
            ([(140,50),(50,50),(50,100),(140,100),(50,100),(50,150),(140,150)]),
            10
            )

        # Design Front
        if self.mode in [self.DESIGNFRONTMIX,self.DESIGNSIDESMIX,self.DESIGNSLEEVESMIX]:
            if self.mode == self.DESIGNFRONTMIX:
                redColor = self.redGradient[int(self.frontColor[0]/255 * 10)]
                greenColor = self.greenGradient[int(self.frontColor[1]/255 * 10)]
                blueColor = self.blueGradient[int(self.frontColor[2]/255 * 10)]
            if self.mode == self.DESIGNSIDESMIX:
                redColor = self.redGradient[int(self.sidesColor[0]/255 * 10)]
                greenColor = self.greenGradient[int(self.sidesColor[1]/255 * 10)]
                blueColor = self.blueGradient[int(self.sidesColor[2]/255 * 10)]
            if self.mode == self.DESIGNSLEEVESMIX:
                redColor = self.redGradient[int(self.sleevesColor[0]/255 * 10)]
                greenColor = self.greenGradient[int(self.sleevesColor[1]/255 * 10)]
                blueColor = self.blueGradient[int(self.sleevesColor[2]/255 * 10)]

            pygame.draw.rect(
                self.frameSurface,
                redColor,
                (1520,0,400,360)
                )
            pygame.draw.rect(
                self.frameSurface,
                greenColor,
                (1520,360,400,360)
                )
            pygame.draw.rect(
                self.frameSurface,
                blueColor,
                (1520,720,400,360)
                )

    def blitCameraDone(self):
        mainPath = os.getcwd()
        os.chdir("screenshots")
        if self.screenshot == None:
            self.screenshot = pygame.image.load("screenshot.png")
            self.tempScreenshot = pygame.transform.scale(self.screenshot,(672,378))
        else:
            self.screen.blit(self.tempScreenshot, (119,81))
            self.screen.blit(self.cameraDone, (810,0))
        os.chdir(mainPath)

    def blitGUI(self):
        if self.mode == self.FULLSCREEN or self.mode == self.CAMERA: return
        self.screen.blit(self.fullScreen, (0,440))
        if self.mode == self.MENU: self.screen.blit(self.menu, (760,0))
        elif self.mode == self.CAMERADONE: self.blitCameraDone()
        elif self.mode == self.DESIGN: self.screen.blit(self.design, (760,0))
        elif self.mode in [self.DESIGNFRONTMIX,self.DESIGNSIDESMIX,self.DESIGNSLEEVESMIX]:
            self.screen.blit(self.palette, (617,0))
            if self.sign == 1: self.screen.blit(self.addMode, (0,100))
            elif self.sign == -1: self.screen.blit(self.minusMode, (0,100))
        elif self.mode in [self.DESIGNFRONT,self.DESIGNSIDES,self.DESIGNSLEEVES]:
            self.screen.blit(self.mix, (0,100))
            self.screen.blit(self.designColors, (810,0))

    def updateBodies(self):
        for i in range(self.kinect.max_body_count):
            body = self.bodies.bodies[i]

            if i in self.trackedBodies: self.trackedBodies[i][1] = False
            if body.is_tracked:
                if i in self.trackedBodies:
                    self.trackedBodies[i][1] = True
                else:
                    prevLen = len(self.trackedBodies)
                    self.trackedBodies[i] = [prevLen, True]
                    self.model.shapes.append(
                        shirt(0, 0, 0, self.frameSurface,
                        [(43, 156, 54),(200,0,0),(61, 187, 198)]))
                joints = body.joints
                self.updateArms(joints, self.trackedBodies[i][0])
                self.updateHands(joints, self.trackedBodies[i][0])
                self.updateBody(joints, self.trackedBodies[i][0])
        self.removeUntrackedBodies()

    def removeUntrackedBodies(self):
        rmData = [] # Stores (index, trackedBody)
        for checkBody in self.trackedBodies:
            if not self.trackedBodies[checkBody][1]:
                rmIndex = self.trackedBodies[checkBody][0]
                rmData.append((rmIndex,checkBody))
                for otherBody in self.trackedBodies:
                    otherIndex = self.trackedBodies[otherBody][0]
                    if otherIndex > rmIndex:
                        self.trackedBodies[otherBody][0] -= 1

        for rm in rmData:
            shape = self.model.shapes[rm[0]]
            self.model.shapes.remove(shape)
            checkBody = rm[1]
            self.trackedBodies.pop(checkBody)

    def drawAll(self):
        self.model.draw()
        self.closetModel.draw()
        self.updateAllGUI()
        self.drawGUI()

    def runLoop(self):
        # Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return False
        # Reads and processes body frame from kinect
        if self.kinect.has_new_body_frame():
            self.bodies = self.kinect.get_last_body_frame()
            self.updateBodies()
        # KeyPresses
        key = pygame.key.get_pressed()
        if sum(key) > 0: self.model.cam.keyPressed(key, self.model)
        # reads color images from kinect
        if self.kinect.has_new_color_frame():
            frame = self.kinect.get_last_color_frame()
            self.drawColorFrame(frame, self.frameSurface)
            frame = None
        self.drawScreen()
        if self.done: return False
        self.clock.tick(60)
        return True

    def drawScreen(self):
        self.drawAll()
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
        self.blitGUI()
        pygame.display.update()

    def run(self):
        while self.runLoop():
            pass
        self.kinect.close()
        pygame.quit()

game = Game()
game.run()
