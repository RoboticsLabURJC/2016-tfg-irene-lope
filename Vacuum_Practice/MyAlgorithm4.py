#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import threading
import time
from datetime import datetime
import jderobot
import math
import cv2
from math import pi as pi
import random

time_cycle = 80
        

class MyAlgorithm4(threading.Thread):

    def __init__(self, pose3d, motors, laser, bumper):
        self.pose3d = pose3d
        self.motors = motors
        self.laser = laser
        self.bumper = bumper
        
        self.map = cv2.imread("resources/images/mapgrannyannie.png", cv2.IMREAD_GRAYSCALE)
        self.map = cv2.resize(self.map, (500, 500))
        
        self.grid = np.ones([500, 500], float)
        
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)
        
        self.VACUUM_PX_SIZE = 16  
        self.VACUUM_PX_HALF = 8  
        self.VACUUM_SIZE = 0.32
        self.VIRTUAL_OBST = 128
        self.SCALE = 50 #50 px = 1 m

        self.x = None
        self.y = None
        self.xPix = None
        self.yPix = None
        
        self.goSouth = False
        
        self.currentCell = []
        self.returnPoints = []
        
 
    def parse_laser_data(self,laser_data):
        laser = []
        for i in range(laser_data.numLaser):
            dist = laser_data.distanceData[i]/1000.0
            angle = math.radians(i)
            laser += [(dist, angle)]
        return laser
    
    
    def laser_vector(self,laser_array):
        laser_vectorized = []
        for d,a in laser_array:
            x = d * math.cos(a) * -1
            y = d * math.sin(a) * -1 
            v = (x, y)
            laser_vectorized += [v]
        return laser_vectorized
        
        
    def run (self):
        while (not self.kill_event.is_set()):
            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)


    def stop (self):
        self.stop_event.set()


    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()


    def kill (self):
        self.kill_event.set()
        
        
    ######   VACUUM FUNCTIONS   #######
    
    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT
        

    def RTVacuum(self):
        RTy = self.RTy(pi, 5.8, 4.2, 0)
        return RTy


    def euclideanDist(self, p1, p2):
        # p1 = (x1, y1)
        # p2 = (x2, y2)
        d = math.sqrt(pow((p2[0]-p1[0]),2)+pow((p2[1]-p1[1]),2))     
        return d
        
        
    def zigzag(self):
        if self.x == None and self.y == None:
            # Is the first position
            self.x = self.pose3d.getX()
            self.y = self.pose3d.getY()
            self.xPix, self.yPix = self.coordToPix(self.x, self.y)
            self.currentCell = [self.xPix, self. yPix]
            self.paintCell(self.currentCell[0], self.currentCell[1])
        else:
            north, east, west = self.calculateNeigh(self.currentCell[0], self.currentCell[1])
            nCell, eCell, wCell = self.checkNeigh(north, east, west)
            #print 'n', north, nCell
            #print 'e', east, eCell
            #print 'w', west, wCell
            self.checkReturnPoints()
            if self.goSouth == False:
                if nCell == 0:
                    self.currentCell = north
                    self.paintCell(self.currentCell[0], self.currentCell[1])
                else:
                    if eCell == 0:
                        self.currentCell = east
                        self.paintCell(self.currentCell[0], self.currentCell[1])
                        self.goSouth = True                      
            else:
                south = self.calculateSouth(self.currentCell[0], self.currentCell[1])
                sCell = self.checkSouth(south)
                #print 's: ', sCell
                if sCell == 0:
                    self.currentCell = south
                    self.paintCell(self.currentCell[0], self.currentCell[1])        
                else:
                    self.goSouth = False


    ######   MAP FUNCTIONS   ######
    
    def paintCell(self, x, y):
        for i in range((y - self.VACUUM_PX_HALF), (y + self.VACUUM_PX_HALF)):
            for j in range((x - self.VACUUM_PX_HALF), (x + self.VACUUM_PX_HALF)):
                self.map[i][j] = self.VIRTUAL_OBST             
        cv2.imshow("MAP ", self.map)
                 
                 
    def calculateNeigh(self, x, y):
        northCell = [x, y - self.VACUUM_PX_SIZE]
        eastCell = [x + self.VACUUM_PX_SIZE, y]
        westCell = [x - self.VACUUM_PX_SIZE, y]
        return northCell, eastCell, westCell    
        
    
    def calculateSouth(self, x, y):
        southCell = [x, y + self.VACUUM_PX_SIZE]
        return southCell
        
        
    def coordToPix(self, coordX, coordY):
        final_poses = self.RTVacuum() * np.matrix([[coordX], [coordY], [1], [1]]) * self.SCALE
        xPix = int(final_poses.flat[0])
        yPix = int(final_poses.flat[1])
        return xPix, yPix
    
    
    def checkCell(self, centerX, centerY): 
        # center: the central position of the cell
        obstacle = 0
        virtualObst = 0
        
        for i in range((centerY - self.VACUUM_PX_HALF), (centerY + self.VACUUM_PX_HALF)):
            for j in range((centerX - self.VACUUM_PX_HALF), (centerX + self.VACUUM_PX_HALF)):
                #self.map[i][j] = 75
                if self.map[i][j] == 0:
                    # There is an obstacle
                    obstacle = 1
                elif self.map[i][j] == self.VIRTUAL_OBST:
                    # There is a virtual obstacle
                    virtualObst = 1
                          
        if obstacle == 1:
            cell = 1
        elif virtualObst == 1:
            cell = 2
        else:
            cell = 0
            
        return cell
        
        
    def checkNeigh(self, n, e, w):
        northCell = self.checkCell(n[0], n[1])  
        eastCell = self.checkCell(e[0], e[1])  
        westCell = self.checkCell(w[0], w[1])

        if northCell == 0:
            self.savePoint(n)
        if eastCell == 0:
            self.savePoint(e)
        if westCell == 0:
            self.savePoint(w)  
             
        return northCell, eastCell, westCell

                
    def checkSouth(self, s):
        southCell = self.checkCell(s[0], s[1]) 
        if southCell == 0:
            self.savePoint(s)
        return southCell
        
             
    def savePoint(self, p):
        x = 0
        for i in range(len(self.returnPoints)): 
            if (self.returnPoints[i][0] == p[0]) and (self.returnPoints[i][1] == p[1]):
                x = 1
        if x == 0:
            self.returnPoints.append(p)
            
                  
    def checkReturnPoints(self):
        print 'RETURN POINTS: ', self.returnPoints
        x = None
        for i in range(len(self.returnPoints)): 
            if (self.returnPoints[i][0] == self.currentCell[0]) and (self.returnPoints[i][1] == self.currentCell[1]):
                print 'Remove: ', self.returnPoints[i]
                x = i        
        if x != None:
            self.returnPoints.pop(x)
           
          
    
                 
    def execute(self):

        # TODO
        self.zigzag()

