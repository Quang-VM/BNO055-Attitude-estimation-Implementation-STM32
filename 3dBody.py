"""
Visualization of orientation over Serial
Should work with format %f,%f,%f\n

Reference:

    Based on:
        https://github.com/thecountoftuscany/PyTeapot-Quaternion-Euler-cube-rotation

"""

import pygame
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

useSerial = True # set true for using serial for data transmission, false for wifi
useQuat = False   # set true for using quaternions, false for using y,p,r angles


import serial
ser = serial.Serial('COM19', 115200)


def main():
    video_flags = OPENGL | DOUBLEBUF
    pygame.init()
    screen = pygame.display.set_mode((1280, 720), video_flags)
    
    resizewin(1280, 720)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()

    while True:
        event = pygame.event.poll()
        
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            break

        if ser.in_waiting > 0:
            line = ser.readline().decode('UTF-8').strip()
            if line:  # Ensure line is not empty
                try:
                    values = line.split(',')  # Split by comma
                    if len(values) == 3:
                        yaw = float(values[0])
                        pitch = float(values[1])
                        roll = -float(values[2])

                        pitch = -pitch
                        roll = -roll

                        pitch -= 0
                        roll -= 0

                        draw(1, yaw, pitch, roll)
                        pygame.display.flip()
                        frames += 1
                except ValueError:
                    # Handle the case where data is not in the expected format
                    print(f"Invalid data received: {line}")

    print("fps: %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))
    if useSerial:
        ser.close()


def resizewin(width, height):
    """
    For resizing window
    """
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0*width/height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)

def draw(w, nx, ny, nz):
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    drawText((-2.6, 1.6, 2), "BNO055 IMPLEMENTATION - PROPOSAL ALGORITHM", 20)


    yaw = nx
    pitch = ny
    roll = nz
    drawText((-2.6, -1.8, 2), "Yaw: %f, Pitch: %f, Roll: %f" %(yaw, pitch, roll), 16)
    glRotatef(-roll, 0.00, 0.00, 1.00)
    glRotatef(pitch, 1.00, 0.00, 0.00)
    glRotatef(yaw, 0.00, 1.00, 0.00)

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def drawText(position, textString, size):
    font = pygame.font.SysFont("Courier", size, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

if __name__ == '__main__':
    main()