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
        
        self.currentCell = []
        
 
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

   
    def zigzag(self):
        if self.x == None and self.y == None:
            self.x = self.pose3d.getX()
            self.y = self.pose3d.getY()
            self.xPix, self.yPix = self.coordToPix(self.x, self.y)
            self.paintCell(self.xPix, self.yPix)
            self.currentCell = [self.xPix, self. yPix]
        else:
            print 'currentCell', self.currentCell
            north, east, west, south = self.calculateNeigh(self.currentCell[0], self.currentCell[1])
            nCell, eCell, wCell, sCell = self.checkNeigh(north, east, west, south)
            print 'n', north, nCell
            print 'e', east, eCell
            print 'w', west, wCell
            if nCell == 0:
                self.paintCell(north[0], north[1])
                self.currentCell = north
            else:
                if eCell == 0:
                    self.paintCell(east[0], east[1])
                    self.currentCell = east

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
        
    
    def coordToPix(self, coordX, coordY):
        final_poses = self.RTVacuum() * np.matrix([[coordX], [coordY], [1], [1]]) * self.SCALE
        xPix = int(final_poses.flat[0])
        yPix = int(final_poses.flat[1])
        return xPix, yPix
    
    
    def checkCell(self, centerX, centerY):  
    def checkNorthCell(self, centerX, centerY):
        # center: the central position of the cell
        obstacle = 0
        virtualObst = 0
        
        for i in range((centerY - self.VACUUM_PX_HALF), (centerY + self.VACUUM_PX_HALF)):
            for j in range((centerX - self.VACUUM_PX_HALF), (centerX + self.VACUUM_PX_HALF)):
                #self.map[i][j] = 75
                if self.map[i][j] == 0:
                    # There is an obstacle
                    obstacle = 1
                elif self.map[i][j] == 128:
                    # There is a virtual obstacle
                    virtualObst = 1
                      
        cv2.imshow("MAP ", self.map)
        
        if obstacle == 1:
            northCell = 1
        elif virtualObst == 1:
            northCell = 2
        else:
            northCell = 0
            
        return northCell
                
                           
    def checkWestCell(self, centerX, centerY):
        # center: the central position of the cell
        obstacle = 0
        virtualObst = 0
 
        for i in range((centerY - self.VACUUM_PX_HALF), (centerY + self.VACUUM_PX_HALF)):
            for j in range((centerX - self.VACUUM_PX_HALF), (centerX + self.VACUUM_PX_HALF)):
                #self.map[i][j] = 200
                if self.map[i][j] == 0:
                    # There is an obstacle
                    obstacle = 1
                elif self.map[i][j] == 128:
                    # There is a virtual obstacle
                    virtualObst = 1
                      
        cv2.imshow("MAP ", self.map)
        
        if obstacle == 1:
            westCell = 1
        elif virtualObst == 1:
            westCell = 2
        else:
            westCell = 0
            
        return westCell 

        
    def checkEastCell(self, centerX, centerY):
        # center: the central position of the cell
        obstacle = 0
        virtualObst = 0
 
        for i in range((centerY - self.VACUUM_PX_HALF), (centerY + self.VACUUM_PX_HALF)):
            for j in range((centerX - self.VACUUM_PX_HALF), (centerX + self.VACUUM_PX_HALF)):
                #self.map[i][j] = 30
                if self.map[i][j] == 0:
                    # There is an obstacle
                    obstacle = 1
                elif self.map[i][j] == 128:
                    # There is a virtual obstacle
                    virtualObst = 1 
                    
        cv2.imshow("MAP ", self.map) 
        
        if obstacle == 1:
            eastCell = 1
        elif virtualObst == 1:
            eastCell = 2
        else:
            eastCell = 0
            
        return eastCell 
        
        
    def checkNeigh(self, n, e, w, s):
        northCell = self.checkCell(n[0], n[1])  
        eastCell = self.checkCell(e[0], e[1])  
        westCell = self.checkCell(w[0], w[1])  
        southCell = self.checkCell(s[0], s[1])  
        return northCell, eastCell, westCell, southCell
              
                    
    def execute(self):

        # TODO
        self.zigzag()
        
