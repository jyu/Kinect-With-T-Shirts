import numpy
import cv2

import pygame
from pygame.locals import *

def process(points):
    img = cv2.imread("tshirt1.png")
    x = sorted([i[0] for i in points])
    y = sorted([i[1] for i in points])
    xMax = (x[-1] + x[-2]) // 2
    xMin = (x[0] + x[1]) // 2
    yMax = (y[-1] + x[-2]) // 2
    yMin = (y[0] + y[1]) // 2

    crop_img = img[yMin:yMax, xMin:xMax]
    cv2.imshow("cropped", crop_img)
    cv2.waitKey(0)

def main():
    pygame.init()
    background = pygame.image.load("tshirt1.png")
    bgRect = background.get_rect()
    size = (width, height) = background.get_size()
    screen = pygame.display.set_mode(size)

    points = set()

    while True:
        screen.blit(background, bgRect)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                points.add(pygame.mouse.get_pos())
                print(points)
            if event.button == 3:
                process(points)


        pygame.display.flip()
        pygame.time.wait(10)


main()
