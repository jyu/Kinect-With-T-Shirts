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
        self.screen = pygame.display.set_mode((960,540),
                       pygame.HWSURFACE|pygame.DOUBLEBUF, 32)
        # exit game
        self.done = False
        # color and body frames from kinect runtime object
        self.kinect = PyKinectRuntime.PyKinectRuntime(
                       PyKinectV2.FrameSourceTypes_Color |
                       PyKinectV2.FrameSourceTypes_Body)

        self.bodies = None
        self.frameSurface = pygame.Surface(
                            (self.kinect.color_frame_desc.Width,
                             self.kinect.color_frame_desc.Height),
                             0,
                             32)
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
        #body variables
        self.yLeftShoulder = 0
        self.yRightShoulder = 0
        self.yLeftHip = 0
        self.yRightHip = 0
        self.xLeftShoulder = 0
        self.xRightShoulder = 0
        self.xLeftHip = 0
        self.xRightHip = 0
        self.xLeftElbow = 0
        self.yLeftElbow = 0
        self.xRightElbow = 0
        self.yRightElbow = 0


    def drawColorFrame(self, frame, target_surface):
        target_surface.lock()
        address = self.kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()

    def updateBody(self, joints):
        #update body trackers
        self.yLeftHip = joints[PyKinectV2.JointType_HipLeft].Position.y
        self.yRightHip = joints[PyKinectV2.JointType_HipRight].Position.y
        self.yLeftShoulder = joints[
                            PyKinectV2.JointType_ShoulderLeft].Position.y
        self.yRightShoulder = joints[
                              PyKinectV2.JointType_ShoulderRight].Position.y
        self.xLeftHip = joints[PyKinectV2.JointType_HipLeft].Position.x
        self.xRightHip = joints[PyKinectV2.JointType_HipRight].Position.x
        self.xLeftShoulder = joints[
                            PyKinectV2.JointType_ShoulderLeft].Position.x
        self.xRightShoulder = joints[
                              PyKinectV2.JointType_ShoulderRight].Position.x
    def updateArms(self, joints):
        #updates arms
        self.xLeftElbow = joints[PyKinectV2.JointType_ElbowLeft].Position.x
        self.yLeftElbow = joints[PyKinectV2.JointType_ElbowLeft].Position.y
        self.xRightElbow = joints[PyKinectV2.JointType_ElbowRight].Position.x
        self.yRightElbow = joints[PyKinectV2.JointType_ElbowRight].Position.y

    def drawArms(self):
        sleve1 = xLeftElbow

    def drawBody(self):
        rightPart = (self.xRightShoulder + self.xRightHip)/2
        leftPart = (self.xLeftShoulder + self.xLeftHip)/2
        upPart = (self.yRightShoulder + self.yLeftShoulder)/2
        downPart = (self.yRightHip + self.yLeftHip)/2
        #converts sensor coords to pygame screen coords

        bodyX1 = ((rightPart*(self.screenWidth/3)+self.cornerToMiddleConstant)+
                 self.shirtCompensationWidth)
        bodyY1 = (-1*(upPart-self.sensorScreenHeight/2)*
                 (self.screenHeight/self.sensorScreenHeight))
        bodyX2 = ((leftPart*(self.screenWidth/3)+self.cornerToMiddleConstant)-
                 self.shirtCompensationWidth)
        bodyY2 = (-1*(downPart-self.sensorScreenHeight/2)*
                 (self.screenHeight/self.sensorScreenHeight))

        bodyWidth = bodyX2 - bodyX1
        bodyHeight = -1 * (bodyY1 - bodyY2) - self.shirtCompensationHeight

        pygame.draw.rect(self.frameSurface,
                     (0,200,0),
                     (bodyX1,bodyY1,bodyWidth,bodyHeight)
                     )
        pygame.draw.rect(self.frameSurface,
                     (200,0,0),
                     (bodyX2,bodyY2,20,20)
                     )
        pygame.draw.rect(self.frameSurface,
                     (200,0,0),
                     (bodyX1,bodyY1,20,20)
                     )
        pygame.draw.rect(self.frameSurface,
                     (200,0,0),
                     (bodyX1,bodyY2,20,20)
                     )
        pygame.draw.rect(self.frameSurface,
                     (200,0,0),
                     (bodyX2,bodyY1,20,20)
                     )

    def run(self):
        while not self.done:
            #pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.done = True

            #reads and processes body frame from kinect
            if self.kinect.has_new_body_frame():
                self.bodies = self.kinect.get_last_body_frame()

                for i in range(0, self.kinect.max_body_count):
                    body = self.bodies.bodies[i]
                    if body.is_tracked:
                        joints = body.joints
                        self.updateBody(joints)
                        self.updateArms(joints)

            #reads color images from kinect
            if self.kinect.has_new_color_frame():
                frame = self.kinect.get_last_color_frame()
                self.drawColorFrame(frame, self.frameSurface)
                frame = None

            self.drawBody()

            #changes ratio of image to output to window
            h_to_w = float(self.frameSurface.get_height() /
                           self.frameSurface.get_width())
            target_height = int(h_to_w*self.screen.get_width())
            surface_to_draw = pygame.transform.scale(self.frameSurface,
                                                    (self.screen.get_width(),
                                                    target_height))
            self.screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()

            self.clock.tick(60)

        self.kinect.close()
        pygame.quit()


game = Game()
game.run()
