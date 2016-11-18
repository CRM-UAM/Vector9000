# coding=utf-8

import argparse
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

R = 31/2
n = 10
Ce = 6
Cm = (2 * math.pi * R) / (n * Ce)
L = 140

list_of_pos = [(0,0,0,math.pi)]

def ticksToPos(tiempo, enc1, enc2):
    (_,xLast,yLast, tLast) = list_of_pos[-1]
    print tiempo, enc1, enc2, xLast, yLast, tLast
    avance1 = (float(enc1))*Cm
    avance2 = (float(enc2))*Cm
    varPosC = (avance1+avance2)/2
    Ang = tLast + (avance1- avance2 )/L
    x = xLast +varPosC*math.cos(Ang)
    y = yLast + varPosC*math.sin(Ang)
    list_of_pos.append( (tiempo,x,y,Ang) )
    return x, y, Ang

fName=""

def importEncodersData(t=0):
    f = open("4vueltasSinPesos.data.txt", 'r')
    lastData = [0,0,0]
    num=0
    for linea in f:
        data = linea.split(" ")
        yield ticksToPos(float(data[0]),float(data[1])-float(lastData[1]),float(data[2])-float(lastData[2]))
        lastData[1]=data[1]
        lastData[2]=data[2]
    f.close()


def init():
    ax.set_ylim(-1.1, 1.1)
    ax.set_xlim(0, 10)
    del xdata[:]
    del ydata[:]
    line.set_data(xdata, ydata)
    return line,



def run(data):
    # update the data
    x, y, t = data
    xdata.append(x)
    ydata.append(y)
    xmin, xmax = ax.get_xlim()
    ymin, ymax = ax.get_ylim()
    if x >= xmax:
        ax.set_xlim(xmin, x*1.1)
        ax.figure.canvas.draw()
    if x < xmin:
        ax.set_xlim(x*1.1, xmax)
        ax.figure.canvas.draw()
    if y >= ymax:
        ax.set_ylim(ymin, y*1.1)
        ax.figure.canvas.draw()
    if y < ymin:
        ax.set_ylim(y*1.1, ymax)
        ax.figure.canvas.draw()
    line.set_data(xdata, ydata)

    return line,





if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Simulacion posicion Vector9000')
    parser.add_argument('fName', help='Nombre del fichero a leer')
    args = parser.parse_args()
    fNmame = args.fName

    # importEncodersData(args.fName, numLines)
    # tIni,_,_,_= list_of_pos[0]
    # listX = []
    # listY = []
    # listR1X = []
    # listR1Y = []
    # listR2X = []
    # listR2Y = []
    # # ax = plt.axes()
    # for (a,x,y,t) in list_of_pos:
    #     listX.append(x)
    #     listY.append(y)
    #     listR1X.append(x-(L/2)*math.sin(t))
    #     listR1Y.append(y+(L/2)*math.cos(t))
    #     listR2X.append(x+(L/2)*math.sin(t))
    #     listR2Y.append(y-(L/2)*math.cos(t))
        # plt.plot([x-(L/2)*math.sin(t),x+(L/2)*math.sin(t)], [y+(L/2)*math.cos(t),y-(L/2)*math.cos(t)], linestyle='-', linewidth=2, color="red")
        # plt.plot([x,x+100*math.cos(t)], [y,y+100*math.sin(t)], linestyle='-', linewidth=2, color="red")
        # print (a,x,y,t)

    # plt.scatter(listX,listY)
    # plt.scatter(listR1X,listR1Y, color="red")
    # plt.scatter(listR2X,listR2Y, color="red")
    # plt.ylim([-1500,900])
    # plt.xlim([-1500,900])
    # plt.show()

    fig, ax = plt.subplots()
    line, = ax.plot([], [], lw=2)
    ax.grid()
    ax.set_xlim(-1500, 1500)
    ax.set_ylim(-500, 2500)
    xdata, ydata = [], []


    ani = animation.FuncAnimation(fig, run, importEncodersData, blit=False, interval=10,
                              repeat=False, init_func=init)
    plt.show()
