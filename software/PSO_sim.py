# -*- coding: utf-8 -*-
#
import math
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from pyswarm import pso
from time import sleep
import serial

connected = False
serial_port = serial.Serial('/dev/cu.bqUNO-SPPDev', 19200, timeout=10)
f1=open('resultadosPSO.txt', 'w+')
f1.write('Vel_Base\tKp\tKd\tTiempo_Vuelta\n')



P = []
D = []
V = []
T = []

def f_robot(params):
    global serial_port
    kp=params[0]
    kd=params[1]
    vel=65

    serial_port.write("=P"+str(kp)+"+")
    serial_port.write("=D"+str(kd)+"+")
    serial_port.write("=B"+str(vel)+"+")

    lin = "a"
    while lin.replace('.','',1).isdigit() == False:
        lin = serial_port.readline().rstrip()
        #print lin, lin.replace('.','',1).isdigit()
    print "Evaluación: VEL= "+str(vel)+" KP= "+str(kp)+" KD= "+str(kd)+" TIEMPO: "+str(float(lin))
    P.append(kp)
    D.append(kd)
    V.append(vel)
    T.append(float(lin))
    f1.write(str(vel)+"\t"+str(kp)+"\t"+str(kd)+"\t"+str(float(lin))+"\n")
    return float(lin)



def f(xv):
    '''
    Funcion a minimizar: minimo = 0.73006 en el punto (x=-3.263,y=-3.186)
    '''
    global x,y,z
    x = xv[0]
    y = xv[1]
    z = math.sin( math.sqrt(pow(x,2) + pow(y,2) ))/5 +  math.sin( math.sqrt(pow(x-1,2)  + pow(y+1,2)  ))/5 +  math.sin( math.sqrt(pow(x+1,2)  + pow(y-1,2)  ))/3
    sleep(0.05)
    print x,y,z
    #X.append(x)
    #Y.append(y)
    #Z.append(z)
    return z

print f([0,0])
print f([-3.263,-3.186])
print f([-3.186,-3.263])

print "busqueda PSO"
lb = [0.045, 2200]
ub = [0.085, 3800]

xopt, fopt = pso(f_robot, lb, ub, swarmsize=5, minstep=30)

X = []
Y = []
Z = []

print len(X)
print xopt
print fopt


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
for i in range(len(X)):
    xs = X[i]
    ys = Y[i]
    zs = Z[i]
    ax.scatter(xs, ys, zs, c=[i/len(X), 0.2, 0.3], marker='o')



# xx = yy = np.arange(-5.0, 5.0, 0.1)
# XX, YY = np.meshgrid(xx, yy)
# zs = np.array([f([x,y])-0.1 for x,y in zip(np.ravel(XX), np.ravel(YY))])
# ZZ = zs.reshape(XX.shape)

# ax.plot_surface(XX, YY, ZZ)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
