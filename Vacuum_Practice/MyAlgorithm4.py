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
        kernel = np.ones((10, 10), np.uint8)
        kernelV = np.ones((12, 12), np.uint8)
        self.mapE = cv2.erode(self.map_orig, kernel, iterations=1)
        self.mapEVis = cv2.erode(self.map_orig, kernelV, iterations=1)
        self.mapECopy = self.mapE.copy()
        
        self.SCALE = 50.00 #50 px = 1 m
        self.VACUUM_PX_SIZE = 16 
        self.VACUUM_PX_HALF = 8 
        self.VACUUM_SIZE = 0.34
        self.COLOR_VIRTUAL_OBST = 128
        self.MIN_MAP = 24
        self.MAX_MAP = 476
        self.STEP = 0.10 # 10 cm 

        self.x = None
        self.y = None
        self.yaw = None
        self.xPix = None
        self.yPix = None
        self.minDist = None
        self.goTo =  None
        
        self.goSouth = False
        self.goingReturnPoint = False
        self.endSearch = False

        self.currentCell = []
        self.nextCell = []
        self.returnPoints = []
        self.path = []
        self.returnPoint = []
        self.returnPath = []
        self.visPoints = []
        
 
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
            print '\n\nPOSE INICIAL:', 'x:', self.x, 'y:', self.y ,'\n'
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
                    print '\n...Checking the return points...\n'
                    self.stopVacuum()
                    self.goSouth = False
                    self.checkReturnPoints()
                    if len(self.returnPoints) > 0:
                        self.returnPoint = self.checkMinDist(self.returnPoints, self.currentCell)
                        print '   RETURN POINT:', self.returnPoint
                        self.goingReturnPoint = True
                    else:
                        print 'END SWEEP'
                        self.stop()
                else:
                    self.driving(cells, neighbors)
            else:
                arrive = self.checkArriveCell(self.returnPoint)
                if arrive == False:
                    print '\n\n...Going to the return cell...\n'
                    self.goToReturnPoint() 
                else:
                    self.goingReturnPoint = False
                    print '    VACUUM ARRIVED TO THE RETURN POINT'
                    self.returnPath = []
                    self.currentCell = self.returnPoint
                    self.savePath(self.currentCell)
                    self.paintCell(self.currentCell, self.COLOR_VIRTUAL_OBST, self.mapE)
                    print '    NEW CURRENT CELL', self.currentCell
        

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
            
            
            
    ######   MOVEMENT PLANNING FUNCTIONS   ######
            
    def zigzag(self, cells, neighbors):
        #cells = [nCell, eCell, wCell, sCell] -> Can be: 0,1,2
        #neighbors = [north, east, west, south] -> Positions in the map
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
            if nCell == 0: #north
                self.nextCell = north
                self.goTo = 'north'
            else:
                if sCell == 0: #south
                    self.nextCell = south
                    self.goTo = 'south'
                    self.goSouth = True 
                elif eCell == 0: #east
                    self.nextCell = east
                    self.goTo = 'east'
                    self.goSouth = True 
                elif wCell == 0: #west
                    self.nextCell = west
                    self.goTo = 'west'
                    self.goSouth = True                                          
        else:
            if sCell == 0: #south
                self.nextCell = south
                self.goTo = 'south'      
            else:
                self.goSouth = False
          
                      
    def calculateNeigh(self, cell):
        # cell = [x,y]
        # Check that the cells are inside the map
        if cell[0] != None and cell[1] != None:
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
        else:  
            northCell = [None, None]
            southCell = [None, None]
            westCell = [None, None]
            eastCell = [None, None]
        neighbors = [northCell, eastCell, westCell, southCell]   
        return neighbors
    
    
    def checkCell(self, cell): 
        # cell = [x,y] : the central position of the cell
        obstacle = 0
        virtualObst = 0
        c = None
        if cell[0] != None and cell[1] != None:
            cell = [int(cell[0]), int(cell[1])]
            for i in range((cell[1] - self.VACUUM_PX_HALF), (cell[1] + self.VACUUM_PX_HALF)):
                for j in range((cell[0] - self.VACUUM_PX_HALF), (cell[0] + self.VACUUM_PX_HALF)):
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
        northCell = self.checkCell(north) 
        
        east = neighbors[1]
        eastCell = self.checkCell(east) 

        west = neighbors[2]
        westCell = self.checkCell(west) 

        south = neighbors[3]
        southCell = self.checkCell(south) 
        
        cells = [northCell, eastCell, westCell, southCell] 
        return cells
        
        
    def isReturnPoint(self, cells):
        #If there is a free neighbor, save the current cell
        nCell = cells[0]
        eCell = cells[1]
        wCell = cells[2]
        sCell = cells[3]
        if nCell == 0 :
            self.savePoint(self.currentCell, self.returnPoints)
        if eCell == 0:
            self.savePoint(self.currentCell, self.returnPoints)
        if wCell == 0:
            self.savePoint(self.currentCell, self.returnPoints)
        if sCell == 0:
            self.savePoint(self.currentCell, self.returnPoints) 
             
             
    def savePoint(self, p, l):
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
            if nCell == 0:
                cont += 1
            if eCell == 0:
                cont += 1
            if wCell == 0:
                cont += 1
            if wCell == 0:
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
        if nCell > 0:
            if eCell > 0:
                if wCell > 0:
                    if sCell > 0:
                        critical = True      
               
        return critical
 
 
    def savePath(self, cell):
        self.path.append(cell)

        
    
    ######   DRIVING FUNCTIONS   ###### 
    
    def driving(self, cells, neighbors):
        #cells = [nCell, eCell, wCell, sCell] -> Can be: 0,1,2
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
            
           
    def goToCell(self, cell):
        self.x = self.pose3d.getX()
        self.y = self.pose3d.getY()
        self.yaw = self.pose3d.getYaw()
        poseVacuum = [round(self.x, 2), round(self.y, 2)]
        nextNCell = self.calculateNeigh(cell)[0]
        nextECell = self.calculateNeigh(cell)[1]
        nextWCell = self.calculateNeigh(cell)[2]
        nextSCell = self.calculateNeigh(cell)[3]
        xc, yc = self.pix2coord(cell[0], cell[1])
        if self.goingReturnPoint == False:
            #Calculate the desviation with the next cell
            if self.goTo == 'north' and nextNCell[0] != None and nextNCell[1] != None: 
                xc, yc = self.pix2coord(nextNCell[0], nextNCell[1])
            elif self.goTo == 'east' and nextECell[0] != None and nextECell[1] != None:
                xc, yc = self.pix2coord(nextECell[0], nextECell[1])
            elif self.goTo == 'west' and nextWCell[0] != None and nextWCell[1] != None:
                xc, yc = self.pix2coord(nextWCell[0], nextWCell[1])
            elif self.goTo == 'south' and nextSCell[0] != None and nextSCell[1] != None:
                xc, yc = self.pix2coord(nextSCell[0], nextSCell[1])
        cell = [round(xc, 2),round(yc, 2)]
        desviation = self.calculateDesv(poseVacuum, cell)
        self.controlDrive(desviation)
                 
            
    def calculateDesv(self, poseVacuum, cell):
        # poseVacuum = [x1, y1] coord gazebo
        # cell = [x2, y2] coord gazebo
        x, y = self.abs2rel(cell, poseVacuum, self.yaw)
        desv = math.degrees(math.atan2(y,x))
        desv = round(desv, 1)
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
        wFast = 0.18
        wSlow = 0.09
        if desv > 0: #LEFT
            self.controlDesv(desv, wFast, wSlow)
        else: #RIGHT
            self.controlDesv(desv, -wFast, -wSlow)
       
       
    def controlDesv(self, desv, wFast, wSlow): 
        desv = abs(desv) 
        th1 = 6
        th2 = 15
        v = 0.1
        if desv >= th2:
            self.motors.sendV(0)
            self.motors.sendW(wFast)
        elif desv < th2 and desv >= th1:
            self.motors.sendV(v)
            self.motors.sendW(wSlow)
        else:
            v = self.setV()
            self.motors.sendV(v)
            self.motors.sendW(0)
                        
    
    def setV(self):
        #Velocity
        vMax = 0.3
        vFast = 0.25
        vMed = 0.14
        vSlow = 0.1
        
        v = vSlow
         
        #Distance [m]
        dMax = 2.3
        dMed = 1.1
        dMin = 0.4
            
        if self.goingReturnPoint == False:
            if self.goTo == 'north' or self.goTo == 'south':
                cells = self.calculateNext4C(self.currentCell, self.goTo)
                c = self.checkNext4C(cells)
                c1 = c[0] # 0: free/ 1:obstacle/ 2: virual obstacle
                c2 = c[1]
                c3 = c[2]
                c4 = c[3]
                if c4 != None and (c1 + c2 + c3 + c4) == 0: #All cells are free
                    v = vMax
                elif c3 != None and (c1 + c2 + c3) == 0:
                    v = vFast
                elif c2 != None and (c1 + c2) == 0:
                    v = vMed
                elif c1 != None and c1 == 0:
                    v = vSlow 
        else:
            pose = [self.x, self.y]
            #xRet, yRet = self.pix2coord(self.returnPoint[0], self.returnPoint[1])
            xRet, yRet = self.pix2coord(self.nextCell[0], self.nextCell[1])
            returnPoint = [xRet, yRet]
            d = self.euclideanDist(pose, returnPoint) 
            print '\n    DISTANCIA AL PUNTO DE RETORNO:\n       ', d , '\n'
            if d <= dMin:
                v = vSlow 
            elif d > dMin and d <= dMed: 
                v = vMed
            elif d > dMed and d <= dMax: 
                v = vFast
            else:
                v = vMax 
        print '\n                       v:', v, '\n'        
        return v
        
        
    def calculateNext4C(self, cell, direction):
        # cell: [xPix, yPix] 
        # direction: north, east, west or south
        # cell: [xPix, yPix] 
        # direction: north, east, west or south
        if direction == 'north':
            cell1 = self.calculateNeigh(cell)[0]
            cell2 = self.calculateNeigh(cell1)[0]
            cell3 = self.calculateNeigh(cell2)[0]
            cell4 = self.calculateNeigh(cell3)[0]
        elif direction == 'south':
            cell1 = self.calculateNeigh(cell)[3]
            cell2 = self.calculateNeigh(cell1)[3]  
            cell3 = self.calculateNeigh(cell2)[3] 
            cell4 = self.calculateNeigh(cell3)[3]         
        cells = [cell1, cell2, cell3, cell4]      
        return cells
            
    
    def checkNext4C(self, cells):
        # cells = [cell1, cell2, cell3, cell4]
        c1 = self.checkCell(cells[0])
        c2 = self.checkCell(cells[1])
        c3 = self.checkCell(cells[2])
        c4 = self.checkCell(cells[3])       
        c = [c1, c2, c3, c4]
        return c
        
                            
    def checkArriveCell(self, cell):
        north = self.checkCell(self.calculateNeigh(cell)[0])
        south = self.checkCell(self.calculateNeigh(cell)[3])
        dMin = 0.067
        dMax = 0.3
        dist = dMax
        if self.goTo == 'east' or self.goTo == 'west' or (north != 0 and self.goTo == 'north') or (south != 0 and self.goTo == 'south') or self.goingReturnPoint == True:
            dist = dMin
            
        print '\n \n                        DIST: ', dist
        
        x = False
        y = False
        xc, yc = self.pix2coord(cell[0], cell[1])
        xdif = abs(xc - self.x)
        ydif = abs(yc - self.y)
        if xdif <= dist:
            x = True
        if ydif <= dist:
            y = True
        if x == True and y == True:
            arrive = True
            if dist == dMin:
                self.stopVacuum()
        else:
            arrive = False
        return arrive
        
    
    def stopVacuum(self):
        self.motors.sendV(0)
        self.motors.sendW(0)
        
   
    def goToReturnPoint(self):
        length = len(self.returnPath)
        if length > 0:
            #print self.returnPath[length-1]
            self.nextCell = self.returnPath[length-1]
            arrive = self.checkArriveCell(self.nextCell)
            if arrive == False:
                self.goToCell(self.nextCell)  
            else:
                print ('    VACUUM ARRIVED TO THE NEXT CELL')
                self.currentCell = self.nextCell
                self.returnPath.pop(length-1)
                print '\nNEW RETURN PATH:\n', self.returnPath, '\n'
        else:
            self.savePoint(self.returnPoint, self.returnPath)
            visPoseRet = self.visibility(self.currentCell, self.returnPoint)   
            if visPoseRet == False:
                self.searchReturnPath(self.returnPoint)
            
            print 'return point:', self.returnPoint                    
            print '\n         RETURN PATH:\n', self.returnPath, '\n'
            
            
    def searchReturnPath(self, cell):
        for i in range(len(self.path)-1, -1, -1):
            newCell = self.path[i]
            if cell != newCell:
                vis = self.visibility(cell, newCell)
                if vis == True:
                    self.savePoint(newCell, self.returnPath)
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
        A = self.pix2coord(A[0], A[1])
        B = self.pix2coord(B[0], B[1])
        numSteps = self.numSteps(A,B)
        if nextPoint == []:
            nextPoint = A
        for i in range(0, numSteps):
            P = self.pointOfLine(nextPoint, B)
            pPix = self.coord2pix(P[0],P[1])
            obst = self.isObstacle(pPix)
            if obst == True:
                visibility = False
                break
            else:
                nextPoint = P
        return visibility
        
        
    def isObstacle(self, P):
        # P [xp, yp] pix map
        P = [int(P[0]), int(P[1])]
        if self.mapEVis[P[1]][P[0]] == 0:
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
