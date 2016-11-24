from pykinect2 import PyKinectV2, PyKinectRuntime
from pykinect2.PyKinectV2 import *

import pygame
from pygame.locals import *

import ctypes
import _ctypes
import sys
import math

from Engine3D import *


class GameRuntime(object):
    def __init__(self):
        pygame.init()

        self.screen_width = 1920
        self.screen_height = 1080

        # Used to manage how fast the screen updates
        self._clock = pygame.time.Clock()

        # Set the width and height of the window [width/2, height/2]
        self._screen = pygame.display.set_mode(
                        (960,540),
                        pygame.HWSURFACE|pygame.DOUBLEBUF, 32)

        # Loop until the user clicks the close button.
        self._done = False

        # Kinect runtime object, we want color and body frames
        self._kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Color | PyKinectV2.FrameSourceTypes_Body)

        self._bodies = None
        self._frame_surface = pygame.Surface(
            (self._kinect.color_frame_desc.Width,
             self._kinect.color_frame_desc.Height),
             0,
             32)

        #Model Code Start
        self.model = Model(self._frame_surface)
        #Model Code End

    def draw_color_frame(self, frame, target_surface):
        target_surface.lock()
        address = self._kinect.surface_as_array(target_surface.get_buffer())
        ctypes.memmove(address, frame.ctypes.data, frame.size)
        del address
        target_surface.unlock()


    def run(self):

        while not self._done:
            for event in pygame.event.get(): # User did something
                if event.type == pygame.QUIT: # If user clicked close
                    self._done = True # Flag that we are done so we exit this loop

            #reads and processes body frame
            if self._kinect.has_new_body_frame():
                self._bodies = self._kinect.get_last_body_frame() #reads what body Kinect is looking at

                for i in range(0, self._kinect.max_body_count): #stores all of the bodies it is looking at into a list
                    body = self._bodies.bodies[i] #processes each individual body based on its index, i
                    if body.is_tracked: #only tracks joints in the Kinect is actively tracking body
                        joints = body.joints

            #reads color images from kinect
            if self._kinect.has_new_color_frame():
                frame = self._kinect.get_last_color_frame()
                self.draw_color_frame(frame, self._frame_surface)
                frame = None

            # Model Code Start
            key = pygame.key.get_pressed()
            self.model.cam.keyPressed(key, self.model)
            self.model.draw()
            #Model Code End

            #changes ratio of image to output to window
            h_to_w = float(self._frame_surface.get_height() /
                           self._frame_surface.get_width())
            target_height = int(h_to_w*self._screen.get_width())
            surface_to_draw = pygame.transform.scale(self._frame_surface,
                                                     (self._screen.get_width(),target_height))
            self._screen.blit(surface_to_draw, (0,0))
            surface_to_draw = None
            pygame.display.update()

            self._clock.tick(60)

        self._kinect.close()
        pygame.quit()

game = GameRuntime();
game.run();
