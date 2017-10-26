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
        self.MIN_MAP = 24
        self.MAX_MAP = 476

        self.x = None
        self.y = None
        self.minDist = None
        
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
  
        
    def sweep(self):
        if self.x == None and self.y == None:
            # Is the first position
            self.x = self.pose3d.getX()
            self.y = self.pose3d.getY()
            xPix, yPix = self.coordToPix(self.x, self.y)
            self.currentCell = [xPix, yPix]
            self.paintCell(self.currentCell)
        else:
            neighbors = self.calculateNeigh(self.currentCell)
            cells = self.checkNeigh(neighbors)
            self.checkReturnPoints()

            if self.isCriticalPoint(cells):
                print ('CRITICAL POINT')
                if len(self.returnPoints) > 0:
                    print 'NEW ZIGZAG'
                    self.currentCell = self.checkMinDist()
                    self.paintCell(self.currentCell)
                    print 'GO TO:', self.currentCell
                else:
                    print 'END THE SWEEP'
            else:
                print 'ZIGZAG: i am in cell: ', self.currentCell
                print 'n', cells[0]
                print 'e', cells[1]
                print 'w', cells[2]
                print 's', cells[3]
                self.zigzag(cells, neighbors)
                
                
    def zigzag(self, cells, neighbors):
        #cells = [nCell, eCell, wCell, sCell] -> Can be: 0,1,2
        #neighbors = [north, east, west, south] -> Positions in the map
        print 'I go to cell: '
        if self.goSouth == False:
            if cells[0] == 0:
                self.currentCell = neighbors[0]
                self.paintCell(self.currentCell)
            else:
                if cells[1] == 0:
                    self.currentCell = neighbors[1]
                    self.paintCell(self.currentCell)
                    self.goSouth = True 
                elif cells[2] == 0:
                    self.currentCell = neighbors[2]
                    self.paintCell(self.currentCell)
                    self.goSouth = True                                   
        else:
            if cells[3] == 0:
                self.currentCell = neighbors[3]
                self.paintCell(self.currentCell)        
            else:
                self.goSouth = False 
        print self.currentCell
            

    ######   MAP FUNCTIONS   ######
    
    def paintCell(self, cell):
        # cell = [x,y]
        for i in range((cell[1] - self.VACUUM_PX_HALF), (cell[1] + self.VACUUM_PX_HALF)):
            for j in range((cell[0] - self.VACUUM_PX_HALF), (cell[0] + self.VACUUM_PX_HALF)):
                self.map[i][j] = self.VIRTUAL_OBST             
        cv2.imshow("MAP ", self.map)
        
        
                 
    def paintNextCell(self, cell):
        # cell = [x,y]
        for i in range((cell[1] - self.VACUUM_PX_HALF), (cell[1] + self.VACUUM_PX_HALF)):
            for j in range((cell[0] - self.VACUUM_PX_HALF), (cell[0] + self.VACUUM_PX_HALF)):
                self.map[i][j] = 35             
        cv2.imshow("MAP ", self.map)
        
                    
    def calculateNeigh(self, cell):
        # cell = [x,y]

        if cell[1] >= self.MIN_MAP:
            northCell = [cell[0], cell[1] - self.VACUUM_PX_SIZE]
        else:
            northCell = [None, None]
            
        if cell[1] <= self.MAX_MAP:
            southCell = [cell[0], cell[1] + self.VACUUM_PX_SIZE]
        else:
            southCell = [None, None]
            
        if cell[1] >= self.MIN_MAP:
            westCell = [cell[0] - self.VACUUM_PX_SIZE, cell[1]]
        else:
            westCell = [None, None]
            
        if cell[1] <= self.MAX_MAP:
            eastCell = [cell[0] + self.VACUUM_PX_SIZE, cell[1]]
        else:
            eastCell = [None, None]
        
        neighbors = [northCell, eastCell, westCell, southCell]   
        return neighbors
        

    def coordToPix(self, coordX, coordY):
        final_poses = self.RTVacuum() * np.matrix([[coordX], [coordY], [1], [1]]) * self.SCALE
        xPix = int(final_poses.flat[0])
        yPix = int(final_poses.flat[1])
        return xPix, yPix
    
    
    def checkCell(self, cell): 
        # cell: the central position of the cell
        # cell = [x,y]
        obstacle = 0
        virtualObst = 0
        c = None
        if cell[0] != None and cell[1] != None:
            for i in range((cell[1] - self.VACUUM_PX_HALF), (cell[1] + self.VACUUM_PX_HALF)):
                for j in range((cell[0] - self.VACUUM_PX_HALF), (cell[0] + self.VACUUM_PX_HALF)):
                    if self.map[i][j] == 0:
                        # There is an obstacle
                        obstacle = 1
                    elif self.map[i][j] == self.VIRTUAL_OBST:
                        # There is a virtual obstacle
                        virtualObst = 1
                              
            if obstacle == 1:
                c = 1
            elif virtualObst == 1:
                c = 2
            else:
                c = 0
            
        return c
        
        
    def checkNeigh(self, neighbors):
        # neighbors = [north, east, west, south]
        northCell = self.checkCell(neighbors[0])  
        eastCell = self.checkCell(neighbors[1])  
        westCell = self.checkCell(neighbors[2])
        southCell = self.checkCell(neighbors[3])

        if northCell == 0:
            self.savePoint(neighbors[0])
        if eastCell == 0:
            self.savePoint(neighbors[1])
        if westCell == 0:
            self.savePoint(neighbors[2])
        if southCell == 0:
            self.savePoint(neighbors[3])  
        
        cells = [northCell, eastCell, westCell, southCell] 
        return cells
  
             
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
           
          
    def euclideanDist(self, p1, p2):
        # p1 = [x1, y1]
        # p2 = [x2, y2]
        d = math.sqrt(pow((p2[0]-p1[0]),2)+pow((p2[1]-p1[1]),2))     
        return d
        
        
    def checkMinDist(self):
        for i in self.returnPoints:
            d = self.euclideanDist(self.currentCell, i)

            if self.minDist ==None:
                self.minDist = d
                nextCell = i
                
            if d < self.minDist:
                nextCell = i
        #self.paintNextCell(nextCell)
        return nextCell
           

    def isCriticalPoint(self, cells):
        #cells = [nCell, eCell, wCell, sCell]
        if (cells[0] > 0) and (cells[1] > 0) and (cells[2] > 0) and (cells[3] > 0):
            return True
        else:
            return False
    
           
    def execute(self):

        # TODO
        self.sweep()

