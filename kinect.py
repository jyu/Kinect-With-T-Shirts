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

        self.screen_width = 1920
        self.screen_height = 1080
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
        self.frame_surface = pygame.Surface(
                            (self.kinect.color_frame_desc.Width,
                             self.kinect.color_frame_desc.Height),
                             0,
                             32)
        #body variables
        self.yLeftShoulder = 0
        self.yRightShoulder = 0
        self.yLeftHip = 0
        self.yRightHip = 0
        self.xLeftShoulder = 0
        self.xRightShoulder = 0
        self.xLeftHip = 0
        self.xRightHip = 0


    def draw_color_frame(self, frame, target_surface):
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
        print(self.yLeftHip)
        print(self.yRightHip)
        print(self.yLeftShoulder)
        print(self.yRightShoulder)

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

            #reads color images from kinect
            if self.kinect.has_new_color_frame():
                frame = self.kinect.get_last_color_frame()
                self.draw_color_frame(frame, self.frame_surface)
                frame = None

            #changes ratio of image to output to window
            h_to_w = float(self.frame_surface.get_height() /
                           self.frame_surface.get_width())
            target_height = int(h_to_w*self.screen.get_width())
            surface_to_draw = pygame.transform.scale(self.frame_surface,
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
