#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from numpy.linalg import inv
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
        
        self.map_orig = cv2.imread("resources/images/mapgrannyannie.png", cv2.IMREAD_GRAYSCALE)
        self.map_orig = cv2.resize(self.map_orig, (500, 500))
        kernel = np.ones((10,10), np.uint8)
        self.mapE = cv2.erode(self.map_orig, kernel, iterations=1)
        self.mapEVis = self.mapE.copy()
        self.mapECopy = self.mapE.copy()
        
        self.SCALE = 50.00 #50 px = 1 m
        self.VACUUM_PX_SIZE = 20  
        self.VACUUM_PX_HALF = 10 
        self.VACUUM_SIZE = 0.34
        self.COLOR_VIRTUAL_OBST = 128
        self.MIN_MAP = 20
        self.MAX_MAP = 480
        self.STEP = 0.15 # 15 cm 

        self.x = None
        self.y = None
        self.yaw = None
        self.xPix = None
        self.yPix = None
        self.minDist = None
        
        self.goSouth = False
        self.goingReturnPoint = False
        self.endSearch = False

        self.currentCell = []
        self.nextCell = []
        self.returnPoints = []
        self.path = []
        self.returnPoint = []
        self.returnPath = []
        
 
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
      
    def sweep(self):
        if self.x == None and self.y == None:
            # Is the first position
            self.x = self.pose3d.getX()
            self.y = self.pose3d.getY()
            print self.x ,self.y 
            self.xPix, self.yPix = self.coord2pix(self.x, self.y)
            self.currentCell = [self.xPix, self.yPix]
            self.savePath(self.currentCell)
            self.nextCell = self.currentCell
            self.paintCell(self.currentCell, self.COLOR_VIRTUAL_OBST, self.mapE)
        else:
            neighbors = self.calculateNeigh(self.currentCell)
            cells = self.checkNeigh(neighbors)
            self.isReturnPoint(cells)
            if self.goingReturnPoint == False:
                if self.isCriticalPoint(cells):
                    print ('\n   ¡¡¡ CRITICAL POINT !!! \n')
                    self.checkReturnPoints()
                    if len(self.returnPoints) > 0:
                        self.returnPoint = self.checkMinDist(self.returnPoints, self.currentCell)
                        print '   RETURN POINT:', self.returnPoint
                        self.goingReturnPoint = True
                        self.stopVacuum()
                    else:
                        print 'END SWEEP'
                else:
                    self.driving(cells, neighbors)
            else:
                arrive = self.checkArriveCell(self.returnPoint)
                if arrive == False:
                    print '...Going to a return cell...'
                    self.goToReturnPoint() 
                else:
                    self.goingReturnPoint = False
                    print '    VACUUM ARRIVED TO THE RETURN POINT'
                    self.returnPath = []
                    self.currentCell = self.returnPoint
                    self.savePath(self.currentCell)
                    self.paintCell(self.currentCell, self.COLOR_VIRTUAL_OBST, self.mapE)
                    print '    NEW CURRENT CELL', self.currentCell
                    self.stopVacuum()
        
                
    def driving(self, cells, neighbors):
        #cells = [nCell, eCell, wCell, sCell1] -> Can be: 0,1,2
        #neighbors = [north, east, west, south] -> Positions in the map
        if self.nextCell == self.currentCell:
            self.zigzag(cells, neighbors)                  
        else:
            arrive = self.checkArriveCell(self.nextCell)
            if arrive == False:
                self.goToCell(self.nextCell)  
            else:
                print '    VACUUM ARRIVED'
                self.currentCell = self.nextCell
                self.savePath(self.currentCell)
                self.paintCell(self.currentCell, self.COLOR_VIRTUAL_OBST, self.mapE)
                print '    NEW CURRENT CELL', self.currentCell
                self.stopVacuum()
        
            
    def zigzag(self, cells, neighbors):
        #cells = [nCell[1,2], eCell[1,2], wCell[1,2], sCell[1,2]] -> Can be: 0,1,2
        #neighbors = [north[0,1,2], east[0,1,2], west[0,1,2], south[0,1,2]] -> Positions in the map
        nCell = cells[0]
        eCell = cells[1]
        wCell = cells[2]
        sCell = cells[3]
        north = neighbors[0]
        east = neighbors[1]
        west = neighbors[2]
        south = neighbors[3]
        
        print '\n        nCell:', nCell 
        print '\n        sCell:', sCell
        print '\n        eCell:', eCell
        print '\n        wCell:', wCell, '\n'
        
        if self.goSouth == False:
            if (nCell[0] == 0 and nCell[1] == 0) or (nCell[0] == 0 and nCell[1] == 2) or (nCell[0] == 2 and nCell[1] == 0): #north
                self.nextCell = north[0]
            else:
                if (sCell[0] == 0 and sCell[1] == 0) or (sCell[0] == 0 and sCell[1] == 2) or (sCell[0] == 2 and sCell[1] == 0): #south
                    self.nextCell = south[0]
                    self.goSouth = True 
                elif (eCell[0] == 0 and eCell[1] == 0) or (eCell[0] == 0 and eCell[1] == 2) or (eCell[0] == 2 and eCell[1] == 0): #east
                    self.nextCell = east[0]
                    self.goSouth = True 
                elif (wCell[0] == 0 and wCell[1] == 0) or (wCell[0] == 0 and wCell[1] == 2) or (wCell[0] == 2 and wCell[1] == 0): #west
                    self.nextCell = west[0]
                    self.goSouth = True                                          
        else:
            if (sCell[0] == 0 and sCell[1] == 0) or (sCell[0] == 0 and sCell[1] == 2) or (sCell[0] == 2 and sCell[1] == 0): #south
                self.nextCell = south[0]      
            else:
                self.goSouth = False
                    
                    
                    
    ######   MAP FUNCTIONS   ######
    
    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT
        

    def RTVacuum(self):
        RTy = self.RTy(pi, 5.8, 4.2, 0)
        return RTy
        
    
    def coord2pix(self, coordX, coordY):
        RTVacuum = self.RTVacuum()
        origPoses = np.matrix([[coordX], [coordY], [1], [1]])
        finalPoses = RTVacuum * origPoses * self.SCALE
        xPix = finalPoses.flat[0]
        yPix = finalPoses.flat[1]
        return xPix, yPix
        
    
    def pix2coord(self, xPix, yPix):
        RTVacuum = self.RTVacuum()
        RTinv = inv(RTVacuum)
        origPoses = np.matrix([[xPix/(self.SCALE)], [yPix/(self.SCALE)], [1], [1]]) 
        finalPoses = RTinv * origPoses
        x = finalPoses.flat[0]
        y = finalPoses.flat[1]
        return x, y
    
    
    def paintCell(self, cell, color, img):
        # cell = [x,y]
        # color = 0, 255
        # img : map
        cell = [int(cell[0]), int(cell[1])]
        for i in range((cell[1] - self.VACUUM_PX_HALF), (cell[1] + self.VACUUM_PX_HALF)):
            for j in range((cell[0] - self.VACUUM_PX_HALF), (cell[0] + self.VACUUM_PX_HALF)):
                img[i][j] = color             
        
                        
    def paintMap(self):    
        if len(self.path) > 0:
            for cell in self.path:
                self.paintCell(cell, self.COLOR_VIRTUAL_OBST, self.mapECopy)
                        
        if len(self.returnPoints) > 0:
            for cell in self.returnPoints:
                self.paintCell(cell, 85, self.mapECopy)  
        
        if self.nextCell != []:
            self.paintCell(self.nextCell, 180, self.mapECopy)  
 
        if self.currentCell != []:
            self.paintCell(self.currentCell, 150, self.mapECopy)   
              
        if self.returnPoint != []:
            self.paintCell(self.returnPoint, 30, self.mapECopy)
            
        if self.nextCell != []:
            self.paintPoint(self.nextCell, 10, self.mapECopy)
        
        if self.returnPoint != []:
            self.paintPoint(self.returnPoint, 100, self.mapECopy)
        
        if len(self.returnPath) > 0:
            for cell in self.returnPath:
                self.paintPoint(cell, 70, self.mapECopy)
        
        if self.x != None and self.y != None:
            x,y = self.coord2pix(self.x,self.y)
            pose = [x, y]
            self.paintPoint(pose, 220, self.mapECopy)

    
    def paintPoint(self, point, color, img):
        point = [int(point[0]),int(point[1])]
        img[point[1]][point[0]] = color
        img[point[1]-1][point[0]] = color
        img[point[1]+1][point[0]] = color
        img[point[1]][point[0]-1] = color
        img[point[1]][point[0]+1] = color
        img[point[1]-1][point[0]-1] = color
        img[point[1]-1][point[0]+1] = color
        img[point[1]+1][point[0]+1] = color
        img[point[1]+1][point[0]-1] = color
        
    
    def showMaps(self, n=3): 
        if n == 0:                                                         
            cv2.imshow("MAP ", self.map_orig) 
        elif n == 1:
            cv2.imshow("MAP1 ", self.mapE)
        elif n == 2:
            cv2.imshow('MapECopy', self.mapECopy)  
        else:  
            cv2.imshow("MAP ", self.map_orig) 
            cv2.imshow("MAP1 ", self.mapE)  
            cv2.imshow('MapECopy', self.mapECopy)
            
                      
    def calculateNeigh(self, cell):
        # cell = [x,y]
        # Check that the cells are inside the map
        dif = 15 #15px
        if cell[1] >= self.MIN_MAP:
            n0 = [cell[0], cell[1] - self.VACUUM_PX_HALF] #center
            n1 = [cell[0] - self.VACUUM_PX_HALF/2, cell[1] - dif] #left       
            n2 = [cell[0] + self.VACUUM_PX_HALF/2, cell[1] - dif] #right
            northCell = [n0, n1, n2]
        else:
            northCell = [[None, None], [None, None], [None, None]]
            
        if cell[1] <= self.MAX_MAP:
            s0 = [cell[0], cell[1] + self.VACUUM_PX_HALF] #center
            s1 = [cell[0] - self.VACUUM_PX_HALF/2, cell[1] + dif] #left
            s2 = [cell[0] + self.VACUUM_PX_HALF/2, cell[1] + dif] #right
            southCell = [s0, s1, s2]
        else:
            southCell = [[None, None], [None, None], [None, None]]
            
        if cell[0] >= self.MIN_MAP:
            w0 = [cell[0] - self.VACUUM_PX_HALF, cell[1]] #center
            w1 = [cell[0] - dif, cell[1] - self.VACUUM_PX_HALF/2] #up
            w2 = [cell[0] - dif, cell[1] + self.VACUUM_PX_HALF/2] #down
            westCell = [w0, w1, w2]
        else:
            westCell = [[None, None], [None, None], [None, None]]
            
        if cell[0] <= self.MAX_MAP:
            e0 = [cell[0] + self.VACUUM_PX_HALF, cell[1]] #center
            e1 = [cell[0] + dif, cell[1] - self.VACUUM_PX_HALF/2] #up
            e2 = [cell[0] + dif, cell[1] + self.VACUUM_PX_HALF/2] #down
            eastCell = [e0, e1, e2]
        else:
            eastCell = [[None, None], [None, None], [None, None]]
        
        neighbors = [northCell, eastCell, westCell, southCell]   
        return neighbors
    
    
    def checkCell(self, cell): 
        # cell = [x,y] : the central position of the cell
        obstacle = 0
        virtualObst = 0
        c = None
        if cell[0] != None and cell[1] != None:
            cell = [int(cell[0]), int(cell[1])]
            for i in range((cell[1] - self.VACUUM_PX_HALF/2), (cell[1] + self.VACUUM_PX_HALF/2)):
                for j in range((cell[0] - self.VACUUM_PX_HALF/2), (cell[0] + self.VACUUM_PX_HALF/2)):
                    if self.mapE[i][j] == 0:#black
                        # There is an obstacle
                        obstacle = 1
                    elif self.mapE[i][j] == self.COLOR_VIRTUAL_OBST:#grey
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
        
        north = neighbors[0]
        northCell1 = self.checkCell(north[1]) 
        northCell2 = self.checkCell(north[2])
        northCell = [northCell1, northCell2] 
        
        east = neighbors[1]
        eastCell1 = self.checkCell(east[1]) 
        eastCell2 = self.checkCell(east[2]) 
        eastCell =[eastCell1, eastCell2]
        
        west = neighbors[2]
        westCell1 = self.checkCell(west[1]) 
        westCell2 = self.checkCell(west[2]) 
        westCell =[westCell1, westCell2]
        
        south = neighbors[3]
        southCell1 = self.checkCell(south[1]) 
        southCell2 = self.checkCell(south[2]) 
        southCell =[southCell1, southCell2]
        
        cells = [northCell, eastCell, westCell, southCell] 
        return cells
        
        
    def isReturnPoint(self, cells):
        #If there is a free neighbor, save the current cell
        nCell = cells[0]
        eCell = cells[1]
        wCell = cells[2]
        sCell = cells[3]
        if nCell[0] == 0 and nCell[1] == 0:
            self.saveReturnPoint(self.currentCell, self.returnPoints)
        if eCell[0] == 0 and eCell[1] == 0:
            self.saveReturnPoint(self.currentCell, self.returnPoints)
        if wCell[0] == 0 and wCell[1] == 0:
            self.saveReturnPoint(self.currentCell, self.returnPoints)
        if sCell[0] == 0 and sCell[1] == 0:
            self.saveReturnPoint(self.currentCell, self.returnPoints) 
             
             
    def saveReturnPoint(self, p, l):
        # p: the point 
        # l: the list where save the point
        saved = False
        for i in range(len(l)): 
            if (l[i][0] == p[0]) and (l[i][1] == p[1]):
                saved = True
        if saved == False:
            # This point wasn't saved before
            l.append(p)
            
                  
    def checkReturnPoints(self):
        cont = 0
        for i in range(len(self.returnPoints)): 
            neighbors = self.calculateNeigh(self.returnPoints[i])
            cells = self.checkNeigh(neighbors)       
            nCell = cells[0]
            eCell = cells[1]
            wCell = cells[2]
            sCell = cells[3]
            if nCell[0] == 0 and nCell[1] == 0:
                cont += 1
            if eCell[0] == 0 and eCell[1] == 0:
                cont += 1
            if wCell[0] == 0 and wCell[1] == 0:
                cont += 1
            if wCell[0] == 0 and wCell[1] == 0:
                cont += 1
            
            if cont == 0:
                # There are no free neighbors
                self.returnPoints.pop(i)
                self.checkReturnPoints()
                break
            cont = 0
         
             
    def euclideanDist(self, p1, p2):
        # p1 = [x1, y1]
        # p2 = [x2, y2]
        d = math.sqrt(pow((p2[0]-p1[0]),2)+pow((p2[1]-p1[1]),2))     
        return d


    def checkMinDist(self, points, cell):
        # points: array with the points to compare
        # cell: the point to compare
        minDist = None
        for i in points:
            d = self.euclideanDist(cell, i)
            if minDist == None or d < minDist:
                nextCell = i
                minDist = d
        return nextCell
           

    def isCriticalPoint(self, cells):
        #cells = [nCell[0,1], eCell[0,1], wCell[0,1], sCell[0,1]]
        # 0: no obstacle
        # 1: obstacle
        # 2: virtual obstacle
        
        nCell= cells[0]
        eCell= cells[1]
        wCell= cells[2]
        sCell= cells[3]
        critical = False

        if nCell[0] > 0 and nCell[1] > 0:
            if eCell[0] > 0 and eCell[1] > 0:
                if wCell[0] > 0 and wCell[1] > 0:
                    if sCell[0] > 0 and sCell[1] > 0:
                        critical = True      
               
        return critical
 
 
    def savePath(self, cell):
        self.path.append(cell)

        
    
    ######   DRIVING FUNCTIONS   ######     
           
    def goToCell(self, cell):
        self.x = self.pose3d.getX()
        self.y = self.pose3d.getY()
        self.yaw = self.pose3d.getYaw()
        poseVacuum = [round(self.x,1), round(self.y,1)]
        xc, yc = self.pix2coord(cell[0], cell[1])
        nextCell = [round(xc,1),round(yc,1)]
        desviation = self.calculateDesv(poseVacuum, nextCell)
        self.controlDrive(desviation)
         
            
    def calculateDesv(self, poseVacuum, cell):
        # poseVacuum = [x1, y1] coord gazebo
        # cell = [x2, y2] coord gazebo
        x, y = self.abs2rel(cell, poseVacuum, self.yaw)
        desv = math.degrees(math.atan2(y,x))
        print '\nMY POSE:   NEXT CELL:'
        print '  x:', poseVacuum[0], '    xc:', cell[0]
        print '  y:', poseVacuum[1], '    yc:', cell[1]
        print '\n      DESV:', desv
        return desv


    def abs2rel(self, target, poseVacuum, yaw):
        # target: [xt, yt]
        # poseVacuum: [xv, yv]
        # yaw: orientation vacuum
        dx = target[0] - poseVacuum[0]
        dy = target[1] - poseVacuum[1]
        # Rotate with current angle
        x = dx*math.cos(-yaw) - dy*math.sin(-yaw)
        y = dx*math.sin(-yaw) + dy*math.cos(-yaw)
        return x,y
 
    
    def controlDrive(self, desv):
        w = 0.1
        if desv > 0: #LEFT
            self.controlDesv(desv, w)
        else: #RIGHT
            self.controlDesv(desv, -w)
       
       
    def controlDesv(self, desv, w): 
        desv = abs(desv) 
        th1 = 3
        th2 = 10
        v1 = 0.07
        v2 = 0.1
        if desv >= th2:
            self.motors.sendV(0)
            self.motors.sendW(w)
        elif desv < th2 and desv >= th1:
            self.motors.sendV(v1)
            self.motors.sendW(w)
        else:
            self.motors.sendV(v2)
            self.motors.sendW(0)
                        
                             
    def checkArriveCell(self, cell):
        distMax = 0.08 #8 cm
        distMin = 0
        x = False
        y = False
        xc, yc = self.pix2coord(cell[0], cell[1])
        xdif = abs(xc - self.x)
        ydif = abs(yc - self.y)
        if xdif >= distMin and xdif < distMax:
            x = True
        if ydif >= distMin and ydif < distMax:
            y = True
        if x == True and y == True:
            arrive = True
        else:
            arrive = False
        print '            xdif', xdif
        print '            ydif', ydif
        return arrive
    
    
    def stopVacuum(self):
        self.motors.sendV(0)
        self.motors.sendW(0)
        
   
    def goToReturnPoint(self):
        self.saveReturnPoint(self.returnPoint, self.returnPath)
        visPoseRet = self.visibility(self.currentCell, self.returnPoint) 
        print 'VISIBILIDAD RETURN Y POSE', visPoseRet  
        if visPoseRet == False:
            self.searchReturnPath(self.returnPoint)
        
        print 'return point:', self.returnPoint                    
        print '\n         RETURN PATH:\n', self.returnPath, '\n'
        
        length = len(self.returnPath)
        print 'length', length
        if length > 0:
            print self.returnPath[length-1]
            self.nextCell = self.returnPath[length-1]
            arrive = self.checkArriveCell(self.nextCell)
            if arrive == False:
                self.goToCell(self.nextCell)  
            else:
                print ('    VACUUM ARRIVED TO THE NEXT CELL')
                self.currentCell = self.nextCell
                self.returnPath.pop(length-1)
                print '\nNEW RETURN PATH:\n', self.returnPath, '\n'
                self.stopVacuum()

        
    def searchReturnPath(self, cell):
        for i in range(len(self.path)-1, -1, -1):
            newCell = self.path[i]
            if cell != newCell:
                vis = self.visibility(cell, newCell)
                if vis == True:
                    self.saveReturnPoint(newCell, self.returnPath)
                    break
        if vis == True:
            vis1 = self.visibility(self.currentCell, newCell)
            if vis1 == False:
                self.searchReturnPath(newCell)           
                
                 
    def pointOfLine(self, A, B):
        # A and B : coord gazebo
        # P = A + s(B - A)
        s = self.step(A, B)
        xp = A[0] + s*(B[0] - A[0])
        yp = A[1] + s*(B[1] - A[1]) 
        return [xp, yp]
        
        
    def step(self, A, B):
        distMax = self.euclideanDist(A, B)
        step = self.STEP / distMax # 15cm
        return step
        
      
    def numSteps(self, A, B):
        distMax = self.euclideanDist(A, B)
        numSteps =  distMax / self.STEP         
        d = numSteps - int(numSteps)
        if d > 0:
            numSteps = numSteps + 1
        return int(numSteps)

    
    def visibility(self, A, B):
        visibility = True
        nextPoint = []
        self.paintPoint(A, 70, self.mapECopy)
        self.paintPoint(B, 200, self.mapECopy)
        A = self.pix2coord(A[0], A[1])
        B = self.pix2coord(B[0], B[1])
        numSteps = self.numSteps(A,B)
        if nextPoint == []:
            nextPoint = A
        for i in range(0, numSteps):
            P = self.pointOfLine(nextPoint, B)
            pPix = self.coord2pix(P[0],P[1])
            obst = self.isObstacle(pPix)
            self.paintPoint(pPix, 120, self.mapECopy) 
            if obst == True:
                visibility = False
                print 'LINE ENDS BREAK'
                break
            else:
                nextPoint = P
        return visibility
        
        
    def isObstacle(self, P):
        # P [xp, yp] pix map
        P = [int(P[0]), int(P[1])]
        if self.mapE[P[1]][P[0]] == 0:
            # Obstacle
            obst = True
        else:
            obst = False
        return obst
        
              
    def execute(self):

        # TODO 
               
        self.sweep()
        self.paintMap()
        self.showMaps(2)
        self.showMaps(1)

