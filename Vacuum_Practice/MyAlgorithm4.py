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
        
        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)
        
        self.map = cv2.imread("resources/images/mapgrannyannie.png", cv2.IMREAD_GRAYSCALE)
        self.map = cv2.resize(self.map, (500, 500))
        
        self.SCALE = 50 #50 px = 1 m
        self.VACUUM_PX_SIZE = 16  
        self.VACUUM_PX_HALF = 8  
        self.VACUUM_SIZE = 0.32
        self.VIRTUAL_OBST = 128
        self.MIN_MAP = 24
        self.MAX_MAP = 476
        self.MAX_DESV = 5
        self.MIN_DESV = 2
        self.MAX_XY = 2
        self.MIN_XY = 0

        self.x = None
        self.y = None
        self.yaw = None
        self.xPix = None
        self.yPix = None
        self.minDist = None
        self.direction =None
        
        self.goSouth = False

        self.currentCell = []
        self.nextCell = []
        self.returnPoints = []
        self.path = []
        
 
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
            self.xPix, self.yPix = self.coordToPix(self.x, self.y)
            self.currentCell = [self.xPix, self.yPix]
            self.savePath(self.currentCell)
            self.nextCell = self.currentCell
            self.paintCell(self.currentCell)
        else:
            neighbors = self.calculateNeigh(self.currentCell)
            cells = self.checkNeigh(neighbors)
            self.checkReturnPoints()

            if self.isCriticalPoint(cells):
                print ('CRITICAL POINT')
                if len(self.returnPoints) > 0:
                    print 'NEW ZIGZAG'
                    self.nextCell = self.checkMinDist()
                    # goToReturnPoint()
                    self.savePath(self.nextCell)
                else:
                    print 'END SWEEP'
            else:
                self.driving(cells, neighbors)
         
                
    def driving(self, cells, neighbors):
        #cells = [nCell, eCell, wCell, sCell] -> Can be: 0,1,2
        #neighbors = [north, east, west, south] -> Positions in the map
        print '  CURRENT CELL', self.currentCell
        print '  NEXT CELL', self.nextCell
        if self.nextCell == self.currentCell:
            self.zigzag(cells, neighbors)                  
        else:
            arrive = self.checkArriveCell(self.nextCell)
            if arrive == False:
                self.goNextCell()  
            else:
                print '    VACUUM ARRIVED'
                self.currentCell = self.nextCell
                self.paintCell(self.currentCell)
                print '    NEW CURRENT CELL', self.currentCell
                self.stopVacuum()
                print '    STOP'
        
            
    def zigzag(self, cells, neighbors):
        #cells = [nCell, eCell, wCell, sCell] -> Can be: 0,1,2
        #neighbors = [north, east, west, south] -> Positions in the map
        print 'PLANNING ZIGZAG'
        print 'neighbors[north, east, west, south]', neighbors
        print 'cells[n, e, w, s]', cells
        if self.goSouth == False:
            if cells[0] == 0: #north
                self.savePath(neighbors[0])
                self.direction = 'north'
            else:
                if cells[3] == 0: #south
                    self.savePath(neighbors[3])
                    self.goSouth = True 
                    self.direction = 'south'
                elif cells[1] == 0: #east
                    self.savePath(neighbors[1])
                    self.goSouth = True 
                    self.direction = 'east'
                elif cells[2] == 0: #west
                    self.savePath(neighbors[2])
                    self.goSouth = True 
                    self.direction = 'west'                                         
        else:
            if cells[3] == 0: #south
                self.savePath(neighbors[3]) 
                self.direction = 'south1'      
            else:
                self.goSouth = False
        print '-> -> -> Go to', self.direction
                    
                    
                    
    ######   MAP FUNCTIONS   ######
    
    def coordToPix(self, coordX, coordY):
        final_poses = self.RTVacuum() * np.matrix([[coordX], [coordY], [1], [1]]) * self.SCALE
        xPix = int(final_poses.flat[0])
        yPix = int(final_poses.flat[1])
        return xPix, yPix
        
    
    def paintCell(self, cell):
        # cell = [x,y]
        for i in range((cell[1] - self.VACUUM_PX_HALF), (cell[1] + self.VACUUM_PX_HALF)):
            for j in range((cell[0] - self.VACUUM_PX_HALF), (cell[0] + self.VACUUM_PX_HALF)):
                self.map[i][j] = self.VIRTUAL_OBST             
        cv2.imshow("MAP ", self.map)
        
                    
    def calculateNeigh(self, cell):
        # cell = [x,y]
        # Check that the cells are inside the map
        if cell[1] >= self.MIN_MAP:
            northCell = [cell[0], cell[1] - self.VACUUM_PX_SIZE]
        else:
            northCell = [None, None]
            
        if cell[1] <= self.MAX_MAP:
            southCell = [cell[0], cell[1] + self.VACUUM_PX_SIZE]
        else:
            southCell = [None, None]
            
        if cell[0] >= self.MIN_MAP:
            westCell = [cell[0] - self.VACUUM_PX_SIZE, cell[1]]
        else:
            westCell = [None, None]
            
        if cell[0] <= self.MAX_MAP:
            eastCell = [cell[0] + self.VACUUM_PX_SIZE, cell[1]]
        else:
            eastCell = [None, None]
        
        neighbors = [northCell, eastCell, westCell, southCell]   
        return neighbors
    
    
    def checkCell(self, cell): 
        # cell = [x,y] : the central position of the cell
        obstacle = 0
        virtualObst = 0
        c = None
        if cell[0] != None and cell[1] != None:
            for i in range((cell[1] - self.VACUUM_PX_HALF), (cell[1] + self.VACUUM_PX_HALF)):
                for j in range((cell[0] - self.VACUUM_PX_HALF), (cell[0] + self.VACUUM_PX_HALF)):
                    if self.map[i][j] == 0:#black
                        # There is an obstacle
                        obstacle = 1
                    elif self.map[i][j] == self.VIRTUAL_OBST:#grey
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
            self.saveReturnPoint(neighbors[0])
        if eastCell == 0:
            self.saveReturnPoint(neighbors[1])
        if westCell == 0:
            self.saveReturnPoint(neighbors[2])
        if southCell == 0:
            self.saveReturnPoint(neighbors[3])  
        
        cells = [northCell, eastCell, westCell, southCell] 
        return cells
  
             
    def saveReturnPoint(self, p):
        x = 0
        for i in range(len(self.returnPoints)): 
            if (self.returnPoints[i][0] == p[0]) and (self.returnPoints[i][1] == p[1]):
                x = 1
        if x == 0:
            self.returnPoints.append(p)
            
                  
    def checkReturnPoints(self):
        #print 'RETURN POINTS: ', self.returnPoints
        x = None
        for i in range(len(self.returnPoints)): 
            if self.returnPoints[i] == self.currentCell:
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
            if self.minDist == None:
                self.minDist = d
                nextCell = i                
            if d < self.minDist:
                nextCell = i
        self.minDist = None
        return nextCell
           

    def isCriticalPoint(self, cells):
        #cells = [nCell, eCell, wCell, sCell]
        if (cells[0] > 0) and (cells[1] > 0) and (cells[2] > 0) and (cells[3] > 0):
            return True
        else:
            return False
    
 
    def savePath(self, cell):
        self.nextCell = cell
        self.path.append(cell)

        
    
    ######   DRIVING FUNCTIONS   ######     
    
    def goNextCell(self):
        self.x = self.pose3d.getX()
        self.y = self.pose3d.getY()   
        self.xPix, self.yPix = self.coordToPix(self.x, self.y)
        poseVacuum = [self.xPix, self.yPix]
        desviation = self.calculateDesv(poseVacuum, self.nextCell)
        self.controlDrive(desviation)
        
        
    def calculateDesv(self, poseVacuum, cell):
        # poseVacuum = [x1, y1]
        # cell = [x2, y2]
        a = self.euclideanDist(poseVacuum, cell)
        b = abs(cell[1] - poseVacuum[1])
        yaw = math.degrees(self.pose3d.getYaw()) + 180
        q = self.quadrant(poseVacuum, cell)
        print 'YAW: ', yaw
            
        if a > 0:
            if q == 1:
                alfa = math.degrees(math.asin(b/a))
            elif q == 2:
                alfa = math.degrees(math.acos(b/a)) + 90
            elif q == 3:
                alfa = math.degrees(math.asin(b/a)) + 180
            else:
                alfa = math.degrees(math.acos(b/a)) + 270
            print 'alfa', alfa
            
            if alfa >= 0 and alfa <= 95:
                if yaw > 355 and yaw <= 360:
                    yaw = 0
                    print 'NEW YAW:', yaw
            elif alfa >= 265 and alfa <= 360:
                if yaw < 5 and yaw >= 0:
                    yaw = 360
                    print 'NEW YAW:', yaw
                    
            if yaw > 265 and yaw <= 360:
                if alfa < 5 and alfa >= 0:
                    alfa = 360
                    print 'new alfa', alfa
            if yaw <= 95 and yaw >= 0:
                if alfa <= 360 and alfa >= 355:
                    alfa = 0
                    print 'new alfa', alfa
            desv = yaw - alfa
        else:
            desv = 0       
        print 'DESV:', desv
        return desv
        
    
    def quadrant(self, poseVacuum, cell):
        # poseVacuum = [x1, y1]
        # cell = [x2, y2]
        if cell[0] > poseVacuum[0] and cell[1] <= poseVacuum[1]:
            q = 1
        elif cell[0] <= poseVacuum[0] and cell[1] < poseVacuum[1]:
            q = 2
        elif cell[0] < poseVacuum[0] and cell[1] >= poseVacuum[1]:
            q = 3
        else:
            q = 4
        return q
    
    
    def controlDrive(self, desv):
        w = 0.1 
        if desv > 0: #right
            self.controlDesv(desv, -w)
        else: #left
            self.controlDesv(desv, w)
                
                
    def controlDesv(self, desv, w):
        desv = abs(desv) 
        v = 0.15  
        if desv >= self.MAX_DESV:
            self.motors.sendV(0)
            self.motors.sendW(w)
            #print 'Turn ...', w
        elif self.MIN_DESV < desv and desv < self.MAX_DESV:
            self.motors.sendV(v)
            self.motors.sendW(w)
            #print 'Go and turn ...', w
        else:
            self.motors.sendV(v)
            self.motors.sendW(0)
            #print 'Go straight...', w
       
                               
    def checkArriveCell(self, cell):
        x = False
        y = False
        xdif = abs(cell[0] - self.xPix)
        ydif = abs(cell[1] - self.yPix)
        if xdif >= self.MIN_XY and xdif < self.MAX_XY:
            x = True
        if ydif >= self.MIN_XY and ydif < self.MAX_XY:
            y = True
        if x == True and y == True:
            arrive = True
        else:
            arrive = False
        return arrive
    
    
    def stopVacuum(self):
        self.motors.sendV(0)
        self.motors.sendW(0)
        
             
    def execute(self):

        # TODO        
        self.sweep()
               
