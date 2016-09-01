#!/usr/bin/env python

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from itertools import product, combinations
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import sys

# Drawing 3D vectors with arrow heads
class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

# Global length for the system of coordinates axis
AXIS_LEN = .1
LEN = .2

# Global tables used to contain all the Rotation and Translation to be shown
Rs = []
Ts = []

#Rs.append(np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]))
#Ts.append(np.array([0, 0, 0]))

# Initializing the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_aspect("equal")
ax.set_xlim3d(-1, 1)
ax.set_ylim3d(-1, 1)
ax.set_zlim3d(-1, 1)

# Draw a 3D line in the figure between two 3D points
def draw3DLine(p1, p2, color='black', alpha=1, linewidth=1):
    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], color=color, alpha=alpha, linewidth=linewidth)

# Draw the camera at its position thanks to (R,T) and draw the shape of the field of vision of a camera
# NOTE: this field of vision is always the same and set arbitrary with the LEN constant so it has no meaning in reality except showing the direction where the camera is "looking"
def drawCamera(i):
    #Ts[i] = np.array([0, 0, 0])
    ## Draw the arrow of the translation from the origin to the center of the system of coordinate of the camera
    #a = Arrow3D([0, Ts[i][0]],[0, Ts[i][1]],[0, Ts[i][2]], mutation_scale=20, lw=1, arrowstyle="->", color="k", alpha=0.5)
    #ax.add_artist(a)

    ## Draw the system of coordinates axis of the camera and write its name
    pt1 = (Rs[i].dot(np.array([0,0,0])) + Ts[i])
    pt2 = (Rs[i].dot(np.array([AXIS_LEN, 0, 0])) + Ts[i])
    pt3 = (Rs[i].dot(np.array([0, AXIS_LEN, 0])) + Ts[i])
    pt4 = (Rs[i].dot(np.array([0, 0, AXIS_LEN])) + Ts[i])

    ax.text(pt1[0], pt1[1], pt1[2], str(i))
    draw3DLine(pt1, pt2, color='red', alpha=0.5, linewidth=3)
    draw3DLine(pt1, pt3, color='green', alpha=0.5, linewidth=3)
    draw3DLine(pt1, pt4, color='blue', alpha=0.5, linewidth=3)

    ## Draw the shape of the field of vision of the camera
    pt5 = (Rs[i].dot(np.array([LEN/2, LEN/2, LEN])) + Ts[i])
    pt6 = (Rs[i].dot(np.array([LEN/2, -LEN/2, LEN])) + Ts[i])
    pt7 = (Rs[i].dot(np.array([-LEN/2, LEN/2, LEN])) + Ts[i])
    pt8 = (Rs[i].dot(np.array([-LEN/2, -LEN/2, LEN])) + Ts[i])
    draw3DLine(pt1, pt5, alpha=0.6)
    draw3DLine(pt1, pt6, alpha=0.6)
    draw3DLine(pt1, pt7, alpha=0.6)
    draw3DLine(pt1, pt8, alpha=0.6)
    draw3DLine(pt5, pt6, alpha=0.6)
    draw3DLine(pt5, pt7, alpha=0.6)
    draw3DLine(pt6, pt8, alpha=0.6)
    draw3DLine(pt7, pt8, alpha=0.6)

# Draw all the cameras shapes
def drawCameras():
    for i in range(len(Rs)):
        drawCamera(i)

# Main
def main():
    ## Parse the standard input
    data = ""
    for line in sys.stdin:
        data += line

    res = data.replace("\n", "").split("|")
    for i in range(0,len(res),2):
        if res[i]:
            Rs.append(np.array(eval(res[i])))
            Ts.append(np.array(eval(res[i+1])))

    ## Draw the translations and the cameras and display the figure
    drawCameras()
    plt.show()

if __name__ == "__main__":
    main()
