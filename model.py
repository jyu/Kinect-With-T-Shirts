import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *

vertices = (
    (.75, -1, 0.75),
    (.75, 1, 0.75),
    (-.75, 1, 0.75),
    (-.75, -1, 0.75),
    (.75, -1, 1.25),
    (.75, 1, 1.25),
    (-.75, -1, 1.25),
    (-.75, 1, 1.25)
    )

edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
    )


def Cube():
    glBegin(GL_LINES)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()


def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)
    # move back to see shape
    glTranslatef(0.0,0.0, -5)

    direction = (0, 0, 0, 0)
    # ontrols to move shape
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        if event.type == KEYDOWN:
            if event.key == K_RIGHT:
                direction = (1, 0, 3, 0)
            elif event.key == K_LEFT:
                direction = (1, 0, -3, 0)
            elif event.key == K_UP:
                direction = (1, -1, 0, 0)
            elif event.key == K_DOWN:
                direction = (1, 1, 0, 0)
            elif event.key == K_a:
                direction = (1, 0, 0, 1)
            elif event.key == K_d:
                direction = (1, 0, 0, -1)

            else:
                print ("Unrecognized key")

        if event.type == KEYUP:
            direction = (0, 0, 0, 0)

        glRotatef(*direction)

        #mouse translations
        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                glTranslatef(0,0,1.0)

            if event.button == 3:
                glTranslatef(0,0,-1.0)


        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT)
        Cube()
        pygame.display.flip()
        pygame.time.wait(10)


main()

