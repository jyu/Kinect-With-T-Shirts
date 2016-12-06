from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *

import ctypes
import _ctypes
import pygame
import sys
import math
import os
import cv2

from Engine3D import *

# Kinect Runner
# Jerry Yu
# AndrewID: jerryy
# Section I

# Guided by Kinect Workshop and FlapPyKinect
# https://onedrive.live.com/?authkey=%21AMWDgqPgtkPzsAM&id=ED75CBDC5E4AB0FE%211
# 096749&cid=ED75CBDC5E4AB0FE
# Learned homography from tutorial:
# http://www.learnopencv.com/homography-examples-using-opencv-python-c/
# Learned opencv masking from:
# http://docs.opencv.org/trunk/d0/d86/tutorial_py_image_arithmetics.html

class Game(object):
    def __init__(self):
        pygame.init()
        self.initScreenVar()
        self.initGame()
        self.frameSurface = pygame.Surface(
            (self.kinect.color_frame_desc.Width,
             self.kinect.color_frame_desc.Height
            ),
            0,
            32)
        self.initBodyVar()
        self.initModels()
        self.initPics()
        self.initGUIVars()
        self.initDesignVars()

    def initModels(self):
        # Models from 3D engine to render
        self.model = Model(self.frameSurface, [])
        self.closetModel = Model(self.frameSurface,[])

    def initGame(self):
        # screen updates
        self.clock = pygame.time.Clock()
        # set the width and height of the window to fit in screen
        self.screen = pygame.display.set_mode(
            (self.screenWidth//2, self.screenHeight//2),
            pygame.HWSURFACE | pygame.DOUBLEBUF,
            32
        )
        # Exit game
        self.done = False
        # color and body frames from kinect runtime object
        self.kinect = PyKinectRuntime.PyKinectRuntime(
            PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body
        )
        self.bodies = None
        self.trackedBodies = {}

    def initDesignVars(self):
        self.numColors = 7
        # Gradients for different colors for design screen
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
        self.initGUIModes()
        self.mode = self.MENU
        # Inits colors for model
        self.nextColors = [(43, 156, 54),(200,  0,0),(61, 187, 198)]
        self.frontColor = [200, 0, 0]
        self.sidesColor = [43, 156, 54]
        self.sleevesColor = [61, 187, 198]
        self.nextColor = [0,0,0]
        self.initGUILocks()

    def initGUIModes(self):
        # Inits different GUI modes
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
        self.frontImage = cv2.imread("ironManFront.png")
        self.frontImagePoints = np.array([[0,0],[152,0],[152,255],[0,255]])
        os.chdir(mainPath)
        self.screenshot = None
        self.tempScreenShot = None

    def initScreenVar(self):
        # Screen variables
        self.screenWidth = 1920
        self.screenHeight = 1080
        self.sensorScreenHeight = 1.2
        self.sensorScreenWidth = 3
        self.cornerToMiddleConstant = 1000
        self.shirtHeightConstant = 50
        self.shirtWidthConstant = 50
        self.modelAngle = 20
        self.shLift = 40

    def initBodyVar(self):
        # body variables
        self.initShoulderHip()
        self.initArm()
        self.initBody()

    def initShoulderHip(self):
        # Shoulder and hip joints for each person
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
        # Preps body calculations
        self.partsCount = 4
        self.angleXZAdjustment = 3.8/5
        self.zAdjustment = 600/1.5
        self.radToDeg = 180.0/math.pi

    def initArm(self):
        # Arm joints for each person
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
        self.initArmCalc()

    def initArmCalc(self):
        self.downSwingAng = -1.4
        self.divByZeroVar = 0.01

    def initBody(self):
        # Body Vars for calculations
        self.rightPart, self.leftPart, self.upPart, self.downPart = 0, 0, 0, 0
        self.bodyX1, self.bodyX2 = 0, 0
        self.bodyY2, self.bodyY2 = 0, 0
        self.bodyZ = 0

    # Function from Kinect Workshop
    def drawColorFrame(self, frame, target_surface):
        target_surface.lock()
        address = self.kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def sensorToScreenX(self, sensorPosX):
        # Converts sensor coordinate X to screen coordinates
        screenX = (
            sensorPosX * (self.screenWidth / self.sensorScreenWidth) +
            self.cornerToMiddleConstant
        )
        return screenX

    def sensorToScreenY(self, sensorPosY):
        # Converts sensor coordinate Y to screen coordinates
        screenY = (
            -1 *
            (sensorPosY - self.sensorScreenHeight / 2) *
            (self.screenHeight / self.sensorScreenHeight)
        )
        return screenY

    def data(self, joints, type, z=False):
        # Gets data from joints list
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
        # Get XYZ movement calculations
        self.getBodyParts(i)
        zAvg = self.getZAvg(i)
        # Convert to screen
        self.getPygameBodyCoords(i)
        (bodyCenterX, bodyCenterY, bodyWidth, bodyHeight) = self.getBodyCoord(i)
        # Rotation calculations
        angleXZ = self.angleXZAdjustment * self.getAngleXZ(i)
        angleXZ += self.angleCorrection(bodyCenterX)
        # Update body shape in model
        self.model.shapes[i].update(bodyCenterX,bodyCenterY,
                                    bodyWidth,bodyHeight,
                                    angleXZ,
                                    self.leftArmAngle[i],
                                    self.rightArmAngle[i],
                                    bodyZ)

    def getBodyParts(self, i):
        # Gets 4 sides of the body
        self.rightPart = (self.xRightShoulder[i] + self.xRightHip[i]) / 2
        self.leftPart = (self.xLeftShoulder[i] + self.xLeftHip[i]) / 2
        self.upPart = (self.yRightShoulder[i] + self.yLeftShoulder[i]) / 2
        self.downPart = (self.yRightHip[i] + self.yLeftHip[i]) / 2

    def getZAvg(self, i):
        # Gets average z of the body
        return (self.zRightShoulder[i] + self.zLeftShoulder[i]
                + self.zRightHip[i] + self.zLeftHip[i]) / self.partsCount

    def getBodyCoord(self):
        bodyCenterX = ((self.bodyX1 + self.bodyX2) / 2) - self.screenWidth / 2
        bodyCenterY = ((self.bodyY1 + self.bodyY2) / 2) - self.screenHeight / 2
        bodyWidth = self.bodyX2 - self.bodyX1
        bodyHeight = -1 * (self.bodyY1 - self.bodyY2)
        return (bodyCenterX, bodyCenterY, bodyWidth, bodyHeight)

    def getPygameBodyCoords(self, i):
        # Converts sensor coords to pygame screen coords
        self.bodyX1 = self.sensorToScreenX(rightPart) + self.shirtWidthConstant
        self.bodyY1 = self.sensorToScreenY(upPart)
        self.bodyX2 = self.sensorToScreenX(leftPart) - self.shirtWidthConstant
        self.bodyY2 = self.sensorToScreenY(downPart) - self.shirtHeightConstant
        self.bodyZ = zAvg * self.zAdjustment
        #self.bodyZ = 600
        #print(zAvg)

    def angleCorrection(self, bodyX):
        # As person moves along the X, there is automatic angle added on because
        # of difference in Z of shoulders. This function corrects it so standing
        # straight on an X shold give an angleXZ of 0
        # Got function from running cubic regression on data gathered by kinect
        # Function for correcting:
        return -1 * (bodyX**3 * -4*10**-8 + bodyX**2 * 6*10**-6 -
                    bodyX * 0.0081)

    def getAngleXZ(self, i):
        # Compares shoulder width difference and depth differences to get angle
        # Convert to degrees for debugging and testing readibility
        return (math.atan2(self.zRightShoulder[i] - self.zLeftShoulder[i],
                          self.xRightShoulder[i] - self.xLeftShoulder[i])
                           * self.radToDeg)

    def updateArms(self, joints, i):
        # Updates arm variables
        self.xLeftElbow[i],self.yLeftElbow[i] = self.data(joints, "ElbowLeft")
        self.xRightElbow[i],self.yRightElbow[i] = self.data(joints,"ElbowRight")
        self.updateLeftArm(i)
        self.updateRightArm(i)

    def updateLeftArm(self, i):
        # Left arm
        xShould = self.sensorToScreenX(self.xLeftShoulder[i])
        yShould = self.sensorToScreenY(self.yLeftShoulder[i])+self.shLift
        xElb = self.sensorToScreenX(self.xLeftElbow[i])
        yElb = self.sensorToScreenY(self.yLeftElbow[i])
        try:
            theta = math.atan((yShould - yElb)/(xElb - xShould))
        except:
            theta = math.atan((yShould-yElb)/(xElb-xShould +self.divByZeroVar))
        thetaPrime = math.pi - math.pi/2 - theta
        if theta <= 0 and theta >= self.downSwingAng:
            self.leftDownSwing[i] = False
        elif theta >= 0: self.leftDownSwing[i] = True
        if theta < self.downSwingAng and self.leftDownSwing[i]:
            theta = math.pi/2
        self.leftArmAngle[i] = theta

    def updateRightArm(self, i):
        # Right arm
        xShould = self.sensorToScreenX(self.xRightShoulder[i])
        yShould = self.sensorToScreenY(self.yRightShoulder[i])+self.shLift
        xElb = self.sensorToScreenX(self.xRightElbow[i])
        yElb = self.sensorToScreenY(self.yRightElbow[i])
        try:
            theta = -1 * math.atan((yShould - yElb)/(xElb - xShould))
        except:
            theta =-1*math.atan((yShould-yElb)/(xElb-xShould+self.divByZeroVar))
        thetaPrime = math.pi - math.pi/2 - theta
        if theta <= 0 and theta >= self.downSwingAng:
            self.rightDownSwing[i] = False
        elif theta >= 0: self.rightDownSwing[i] = True
        if theta < self.downSwingAng and self.rightDownSwing[i]:
            theta = math.pi/2
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
        elif (abs(lHandX-rHandX) <= 20 and abs(lHandY-rHandY) <= 20 and
                rHandY > 30):
            self.backToMenu()
        self.updateModes(rHandX, rHandY, lHandY, lHandX, i)

    def backToMenu(self):
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

    def updateModes(self, rHandX, rHandY, lHandY, lHandX, i):
        # Update all modes
        if self.mode == self.MENU: self.updateMenu(rHandX,rHandY,i)
        elif self.mode == self.CLOSET: self.updateCloset(rHandX, rHandY, lHandY)
        elif self.mode == self.CAMERA: self.updateCamera()
        elif self.mode == self.CAMERADONE: self.updateCameraDone(rHandX,rHandY,lHandY,i)
        elif self.mode == self.DESIGN: self.updateDesign(rHandX,rHandY,lHandY,i)
        else: self.checkDesignModes(rHandX,rHandY,lHandY,lHandX,i)

    def checkDesignModes(self, rHandX,rHandY,lHandY,lHandX,i):
        if self.mode == self.DESIGNFRONT:
            self.updatePart(rHandX,rHandY,lHandY,lHandX,i,"Front")
        elif self.mode == self.DESIGNFRONTMIX:
            self.updateMixFront(rHandX,rHandY,lHandY,i)
        elif self.mode == self.DESIGNSIDES:
            self.updatePart(rHandX,rHandY,lHandY,lHandX,i,"Sides")
        elif self.mode == self.DESIGNSIDESMIX:
            self.updateMixSides(rHandX,rHandY,lHandY,i)
        elif self.mode == self.DESIGNSLEEVES:
            self.updatePart(rHandX,rHandY,lHandY,lHandX,i,"Sleeves")
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
        if self.cameraStart == 0:
            self.cameraStart = pygame.time.get_ticks()
        if pygame.time.get_ticks() - self.cameraStart >= self.cameraTimer:
            mainPath = os.getcwd()
            os.chdir("screenshots")
            pygame.image.save(self.screen,"screenshot.png")
            os.chdir(mainPath)
            self.cameraStart = 0
            self.mode = self.CAMERADONE
        timeLeft = self.cameraTimer-(pygame.time.get_ticks()-self.cameraStart)
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

    def updatePart(self, rHandX, rHandY, lHandY, lHandX, i, part):
        step = self.screenHeight / self.numColors
        if lHandX < 200 and lHandY <= 366 and lHandY > 200:
            if part = "Sleeves": self.mode = self.DESIGNSLEEVESMIX
            elif part = "Sides": self.mode = self.DESIGNSIDESMIX
            elif part = "Front": self.mode = self.DESIGNFRONTMIX
        elif rHandX >= 1520:
            if not self.lock[i]:
                if rHandY <= step: self.nextColor = [237,28,36]
                elif rHandY <= step*2: self.nextColor = [255,127,39]
                elif rHandY <= step*3: self.nextColor = [255,242,0]
                elif rHandY <= step*4: self.nextColor = [34,177,76]
                elif rHandY <= step*5: self.nextColor = [0,162,232]
                elif rHandY <= step*6: self.nextColor = [63,72,204]
                elif rHandY <= step*7: self.nextColor = [163,73,164]
            self.lock[i] = True
            for shape in self.model.shapes:
                if part = "Sleeves": shape.colors[2] = tuple(self.nextColor)
                elif part = "Sides": shape.colors[0] = tuple(self.nextColor)
                elif part = "Front": shape.colors[1] = tuple(self.nextColor)
        if rHandX <= 1400: self.lock[i] = False

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
                elif rHandY < self.screenHeight:
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
        self.drawExit()
        # Design Mix Screens
        if self.mode in [self.DESIGNFRONTMIX,
                         self.DESIGNSIDESMIX,
                         self.DESIGNSLEEVESMIX
                        ]:
            redColor, greenColor, blueColor = self.getColors()
            self.drawMixColors(redColor, greenColor, blueColor)

    def drawExit(self):
        # Exit Logo
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

    def getColors(self):
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
        return (redColor, greenColor, blueColor)

    def drawMixColors(self, redColor, greenColor, blueColor):
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
        # GUI for cameraDone mode
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
        # Blits images to the screen for different GUI modes
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
        # Goes through each body detected by kinect
        for i in range(self.kinect.max_body_count):
            body = self.bodies.bodies[i]
            if i in self.trackedBodies: self.trackedBodies[i][1] = False
            if body.is_tracked:
                # Checks if tracked bodies are seen
                if i in self.trackedBodies: self.trackedBodies[i][1] = True
                else:
                    # Adds a new body to model
                    prevLen = len(self.trackedBodies)
                    self.trackedBodies[i] = [prevLen, True]
                    self.model.shapes.append(
                        shirt(0, 0, 0, self.frameSurface,
                        [(43, 156, 54),(200,0,0),(61, 187, 198)]))
                # Updates each shirt iwth joint information
                joints = body.joints
                self.updateArms(joints, self.trackedBodies[i][0])
                self.updateHands(joints, self.trackedBodies[i][0])
                self.updateBody(joints, self.trackedBodies[i][0])
        self.removeUntrackedBodies()

    def removeUntrackedBodies(self):
        # Removes all bodies that are not being tracked
        rmData = [] # Stores (index, trackedBody)
        # Finds indices
        for checkBody in self.trackedBodies:
            if not self.trackedBodies[checkBody][1]:
                rmIndex = self.trackedBodies[checkBody][0]
                rmData.append((rmIndex,checkBody))
                for otherBody in self.trackedBodies:
                    otherIndex = self.trackedBodies[otherBody][0]
                    if otherIndex > rmIndex:
                        self.trackedBodies[otherBody][0] -= 1
        # Removes the shapes
        for rm in rmData:
            shape = self.model.shapes[rm[0]]
            self.model.shapes.remove(shape)
            checkBody = rm[1]
            self.trackedBodies.pop(checkBody)

    def drawAll(self):
        # Calls all draw functions on the surface
        self.model.draw()
        self.closetModel.draw()
        self.updateAllGUI()
        self.drawGUI()

    def drawScreen(self):
        self.drawAll()
        # Changes ratio of image to output to window
        h_to_w = float(
            self.frameSurface.get_height() /
            self.frameSurface.get_width()
        )
        target_height = int(h_to_w * self.screen.get_width())
        surface_to_draw = pygame.transform.scale(
            self.frameSurface,
            (self.screen.get_width(), target_height)
        )
        # Does homography on shirt
        if surface_to_draw != None:
            surface_to_draw = self.addCostume(surface_to_draw)
        self.screen.blit(surface_to_draw, (0,0))
        surface_to_draw = None
        # Blits GUI images onto image
        self.blitGUI()
        pygame.display.update()

    def addCostume(self, image):
        # Guided by homography and opencv tutorials
        # Get pygame screen and convert to BGR for opencv
        source = cv2.cvtColor(self.pygame_to_cvimage(image), cv2.COLOR_RGB2BGR)
        # Search through all bodies
        if len(self.model.shapes) > 0:
            for shirt in self.model.shapes:
                # Get pointlist of front of the shirt
                pointList = shirt.getFrontFace()
                pointList = np.array([list(pointList[0]),
                                     list(pointList[1]),
                                     list(pointList[2]),
                                     list(pointList[3])
                                     ])
                # Convert coord systems
                for i in range(len(pointList)):
                    point = pointList[i]
                    pointList[i] = [point[0]/2, point[1]/2]
                source = self.warp(pointList, source)

        result = np.transpose(source,(1,0,2))
        image = self.cvimage_to_pygame(cv2.cvtColor(result, cv2.COLOR_BGR2RGB))
        return image

    def warp(self, pointList, source):
        # Get warped shirt with homography
        h, status = cv2.findHomography(self.frontImagePoints, pointList)
        warped = cv2.warpPerspective(self.frontImage,
                                    h,
                                    (source.shape[1],source.shape[0]))
        # Get region of interest to put shirt onto
        rows,cols,channels = warped.shape
        roi = source[0:rows, 0:cols]
        # Create mask of shirt and the inverse mask
        img2gray = cv2.cvtColor(warped,cv2.COLOR_BGR2GRAY)
        limit, maxColor = 10, 255
        ret, mask = cv2.threshold(img2gray, limit, maxColor, cv2.THRESH_BINARY)
        mask_inv = cv2.bitwise_not(mask)
        # Get background from source image to replace
        sourceBG = cv2.bitwise_and(roi,roi,mask = mask_inv)
        # Get the foreground of the shirt
        shirtFG = cv2.bitwise_and(warped,warped,mask = mask)
        # Put shirt in region of interest and put it onto the source
        dst = cv2.add(sourceBG,shirtFG, dtype=0)
        source[0:rows, 0:cols] = dst
        return source

    def pygame_to_cvimage(self, surface):
        image = np.array(pygame.surfarray.pixels3d(surface),dtype='uint16')
        image = np.transpose(image,(1,0,2))
        return image

    def cvimage_to_pygame(self, image):
        surface =  pygame.surfarray.make_surface(image)
        return surface.convert()

    def runLoop(self):
        # Pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return False
        # Reads and processes body frame from kinect
        if self.kinect.has_new_body_frame():
            self.bodies = self.kinect.get_last_body_frame()
            self.updateBodies()
        # Reads color images from kinect
        if self.kinect.has_new_color_frame():
            frame = self.kinect.get_last_color_frame()
            self.drawColorFrame(frame, self.frameSurface)
            frame = None
        self.drawScreen()
        if self.done: return False
        self.clock.tick(60)
        return True

    def run(self):
        while self.runLoop():
            pass
        self.kinect.close()
        pygame.quit()

game = Game()
game.run()

