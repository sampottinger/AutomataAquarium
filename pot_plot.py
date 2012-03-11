#!/usr/bin/env python
import cairo
import matplotlib.pyplot as plt
from cStringIO import StringIO
from math import log
import time


def load(filename = 'pot_feedback_2'):
    with open(filename) as f:
        return map(int, f.readlines())


def plot(in1,in2=None):
    if in2:
        xs=in1
        ys=in2
    else:
        xs = range(len(in1))
        ys = in1
    
    x2s =[i for i in range(1,len(ys)) if ys[i-1]> 512 and ys[i] <=512]
    #print xs,ys
    
    x3s = [x2s[i]-x2s[i-1] for i in range(1,len(x2s))]
    
    #plt.plot(x2s, [512]*len(x2s),'ro')
    #plt.plot(x3s)
    x2s=[]
    for x in x2s:
        plt.plot(ys, 'bo')
        plt.xlim(x-250,x+250)
        plt.ylim(400,800)
        show()
        plt.close()
        time.sleep(1)
    
    #xs = xs[:3000]
    #ys = ys[:3000]
    plt.plot(xs,ys,'bo-')
    #plt.plot([0,len(ys)],[400]*2,'r-')
    #plt.plot([0,len(ys)],[800]*2,'r-')
    #plt.xlim(1100,1200)
    plt.xlabel('speed number')
    plt.ylabel('avg revolutions per second')

    plt.savefig('speed.png')
    show()
    plt.close()

import numpy.fft as fft

def fourier(data):
    ys = abs(fft.rfft(data)[1:])
    plt.plot(ys,'b-')
    plt.xlim(0,1000)
    show()
    plt.close()
    
def show():
    try:
        g = gui
    except:
        return
    
    png = StringIO()
    plt.savefig(png,format='png',bbox_inches='tight')
    s = cairo.ImageSurface.create_from_png(StringIO(png.getvalue()))
    ctx = cairo.Context(gui.background)
    ctx.set_source_surface(s)
    ctx.paint()
    gui.redraw()
    


def load_speed(filename='dj_speed_data'):
    data = []
    with open(filename) as f:
        for line in f:
            line = line.split()
            data.append(map(int,[line[0],line[2],line[4],line[6]]))
    return data

def process_speed(data):
    counts = {}
    times = {}
    for start,end,time,speed in data:
        if speed in counts:
            counts[speed] += 1
            times[speed] += time
        else:
            counts[speed] = 1
            times[speed] = time
       
    points = [(speed,float(1000*counts[speed])/times[speed]) for speed in counts.keys()]
    
    fst = lambda x:x[0]
    snd = lambda x:x[1]
    
    points.sort()
    xs=map(fst,points)
    ys=map(snd,points)
    return xs,ys        

def plot_speed(filename):
    plot(*process_speed(load_speed(filename)))

import sys

if __name__ == '__main__':
    if len(sys.argv) > 1:
        plot_speed(sys.argv[1])
    else:
        print 'usage:',sys.argv[0],'<filename of speed data>'
