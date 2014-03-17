# -*- coding: utf-8 -*-
from mpl_toolkits.mplot3d import Axes3D
import random
import matplotlib.pyplot as plt
from pylab import ion
import time

SERIALPORT = "TTYUsb0"



fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_aspect("equal")


#draw a vector
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

import numpy as np
import math

def rotation_matrix(axis,theta):
    axis = axis/math.sqrt(np.dot(axis,axis))
    a = math.cos(theta/2)
    b,c,d = -axis*math.sin(theta/2)
    return np.array([[a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
                     [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
                     [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]])


def rotate_vector(vector,attitude):
    roll,pitch,yaw = attitude

    rollmatrix = rotation_matrix(np.array([1,0,0]),roll)
    pitchmatrix = rotation_matrix(np.array([1,0,0]),pitch)
    yawmatrix = rotation_matrix(np.array([0,0,1]),yaw)

    new_vector = np.dot(rollmatrix,vector)
    new_vector = np.dot(pitchmatrix,new_vector)
    new_vector = np.dot(yawmatrix,new_vector)
    return new_vector


class Arrow3D(FancyArrowPatch):
    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0,0), (0,0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0],ys[0]),(xs[1],ys[1]))
        FancyArrowPatch.draw(self, renderer)

    def set_pos(self,x,y,z):
        self._verts3d = [0,x],[0,y],[0,z]

class AttitudeMarker():
    def __init__(self,pos = (0,0,0),attitude = (0,0,0)):
        self.pos = pos
        self.x = Arrow3D([pos[0],1],[pos[1],0],[pos[2],0], mutation_scale=20, lw=1, arrowstyle="-|>", color="r")
        self.y = Arrow3D([pos[0],0],[pos[1],1],[pos[2],0], mutation_scale=20, lw=1, arrowstyle="-|>", color="g")
        self.z = Arrow3D([pos[0],0],[pos[1],0],[pos[2],1], mutation_scale=20, lw=1, arrowstyle="-|>", color="b")
        self.set_attitude(attitude)

    def set_attitude(self,attitude):
        self.x.set_pos(*rotate_vector([1,0,0],attitude))
        self.y.set_pos(*rotate_vector([0,1,0],attitude))
        self.z.set_pos(*rotate_vector([0,0,1],attitude))

    def get_artists(self):
        return [self.x,self.y,self.z]





ion()
x = Arrow3D([0,1],[0,0],[0,0], mutation_scale=20, lw=1, arrowstyle="-|>", color="r")
y = Arrow3D([0,0],[0,1],[0,0], mutation_scale=20, lw=1, arrowstyle="-|>", color="g")
z = Arrow3D([0,0],[0,0],[0,1], mutation_scale=20, lw=1, arrowstyle="-|>", color="b")
ax.add_artist(x)
ax.add_artist(y)
ax.add_artist(z )

attitude = AttitudeMarker()

roll =0
pitch = 0
yaw = 0

for i in attitude.get_artists():
    ax.add_artist(i)

plt.show()



port = None
import serial
try:
    port = serial.Serial("/dev/{0}".format(SERIALPORT),baudrate=115200)
except:
    pass


#buffer = "control ->       687.0000      713.0000      692.0000 control ->       687.0000      713.0000      692.0000 control ->       687.0000      713.0000      692.0000 control -> "
buffer = ""




def update_attitude():
    attitude.set_attitude([roll,pitch,yaw])
    plt.draw()
    plt.pause(0.0001)


while True:
    while "control ->" not in buffer:
        if port != None:
            buffer = buffer + port.read(5)
            time.sleep(0.001)
    splited = buffer.split("control ->")
    try:
        for substring in splited:
            if len(substring) > 30:
                numbers = substring.split("    ")
                converted_numbers = []
                for number in numbers[1:]:
                    converted_numbers.append(float(number))
                print converted_numbers
                roll, pitch, yaw = converted_numbers
                update_attitude()
    except:
        pass
    buffer = splited[-1]



#
#
#
# while True:
#     roll+=1.0/300
#     yaw+=1.0/300
#     update_attitude()
#
#
#
