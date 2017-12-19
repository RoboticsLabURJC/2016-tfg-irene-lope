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
        
        self.map = cv2.imread("resources/images/mapgrannyannie.png", cv2.IMREAD_GRAYSCALE)
        self.map = cv2.resize(self.map, (500, 500))
        self.map1 = cv2.imread("resources/images/mapgrannyannie.png", cv2.IMREAD_GRAYSCALE)
        self.map1 = cv2.resize(self.map1, (500, 500))
        
        self.SCALE = 50.0 #50 px = 1 m
        self.VACUUM_PX_SIZE = 20  
        self.VACUUM_PX_HALF = 10 
        self.VACUUM_SIZE = 0.34
        self.VIRTUAL_OBST = 128
        self.MIN_MAP = 20
        self.MAX_MAP = 480

        self.x = None
        self.y = None
        self.yaw = None
        self.xPix = None
        self.yPix = None
        self.minDist = None
        self.direction =None
        
        self.goSouth = False
        self.goingReturnPoint = False

        self.currentCell = []
        self.nextCell = []
        self.returnPoints = []
        self.path = []
        self.returnPoint = []
        
 
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
            self.xPix, self.yPix = self.coord2pix(self.x, self.y)
            self.currentCell = [self.xPix, self.yPix]
            self.savePath(self.currentCell)
            self.nextCell = self.currentCell
            self.paintCell(self.currentCell, self.VIRTUAL_OBST, self.map)
        else:
            neighbors = self.calculateNeigh(self.currentCell)
            cells = self.checkNeigh(neighbors)
            self.isReturnPoint(cells)
            self.checkReturnPoints() 
            #print '        GOING TO RETURN POINT:' , self.goingReturnPoint
            if self.goingReturnPoint == False:
                if self.isCriticalPoint(cells):
                    print ('\n   !!! CRITICAL POINT !!! \n')
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
                    self.currentCell = self.returnPoint
                    self.savePath(self.currentCell)
                    self.paintCell(self.currentCell, self.VIRTUAL_OBST, self.map)
                    print '    NEW CURRENT CELL', self.currentCell
        
                
    def driving(self, cells, neighbors):
        #cells = [[nCell1, nCell2], [eCell1, eCell2], [wCell1, wCell], [sCell1, sCell2]] -> Can be: 0,1,2
        #neighbors = [north, east, west, south] -> Positions in the map
        if self.nextCell == self.currentCell:
            self.zigzag(cells, neighbors)                  
        else:
            arrive = self.checkArriveCell(self.nextCell)
            if arrive == False:
                self.goNextCell()  
            else:
                print '    VACUUM ARRIVED'
                self.currentCell = self.nextCell
                self.savePath(self.currentCell)
                self.paintCell(self.currentCell, self.VIRTUAL_OBST, self.map)
                print '    NEW CURRENT CELL', self.currentCell
                self.stopVacuum()
                #time.sleep(0.5)
        
            
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
        
        print '\n...Planing zigzag...\n'
        print '  CURRENT CELL:', self.currentCell
        print '  NEIGHBORS:'
        print '    north:', north
        print '    east:', east
        print '    west:', west
        print '    south:', south
        print '  CELLS:'
        print '    nCell:', nCell
        print '    eCell:', eCell
        print '    wCell:', wCell
        print '    sCell:', sCell
        
        if self.goSouth == False:
            if nCell[0] == 0 and nCell[1] == 0: #north
                self.nextCell = north[0]
                self.direction = 'north'
            else:
                if sCell[0] == 0 and sCell[1] == 0: #south
                    self.nextCell = south[0]
                    self.goSouth = True 
                    self.direction = 'south'
                elif eCell[0] == 0 and eCell[1] == 0: #east
                    self.nextCell = east[0]
                    self.goSouth = True 
                    self.direction = 'east'
                elif wCell[0] == 0 and wCell[1] == 0: #west
                    self.nextCell = west[0]
                    self.goSouth = True 
                    self.direction = 'west'                                         
        else:
            if sCell[0] == 0 and sCell[1] == 0: #south
                self.nextCell = south[0]
                self.direction = 'south'      
            else:
                self.goSouth = False
        #print '-> -> -> Go to', self.direction
                    
                    
                    
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
        xPix = int(finalPoses.flat[0])
        yPix = int(finalPoses.flat[1])
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
        for i in range((cell[1] - self.VACUUM_PX_HALF), (cell[1] + self.VACUUM_PX_HALF)):
            for j in range((cell[0] - self.VACUUM_PX_HALF), (cell[0] + self.VACUUM_PX_HALF)):
                img[i][j] = color             
        
                        
    def paintMap(self):
        # cell = [x,y]
            
        if len(self.path) > 0:
            for cell in self.path:
                self.paintCell(cell, self.VIRTUAL_OBST, self.map1)
                        
        if len(self.returnPoints) > 0:
            for cell in self.returnPoints:
                self.paintCell(cell, 85, self.map1)  
        
        if self.nextCell != []:
            self.paintCell(self.nextCell, 180, self.map1)  
 
        if self.currentCell != []:
            self.paintCell(self.currentCell, 150, self.map1)   
              
        if self.returnPoint != []:
            self.paintCell(self.returnPoint, 30, self.map1)
            
        if self.nextCell != []:
            self.paintPoint(self.nextCell, 10, self.map1)
        
        if self.returnPoint != []:
            self.paintPoint(self.returnPoint, 100, self.map1)
        
        if self.x != None and self.y != None:
            x,y = self.coord2pix(self.x,self.y)
            pose = [x, y]
            self.paintPoint(pose, 220, self.map1)

    
    def paintPoint(self, point, color, img):
        img[point[1]][point[0]] = color
        img[point[1]-1][point[0]] = color
        img[point[1]+1][point[0]] = color
        img[point[1]][point[0]-1] = color
        img[point[1]][point[0]+1] = color
        img[point[1]-1][point[0]-1] = color
        img[point[1]-1][point[0]+1] = color
        img[point[1]+1][point[0]+1] = color
        img[point[1]+1][point[0]-1] = color
        
    
    def showMaps(self, n=2): 
        if n == 0:                                                         
            cv2.imshow("MAP ", self.map) 
        elif n == 1:
            cv2.imshow("MAP1 ", self.map1)  
        else:  
            cv2.imshow("MAP ", self.map) 
            cv2.imshow("MAP1 ", self.map1)  
            
                      
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
            for i in range((cell[1] - self.VACUUM_PX_HALF/2), (cell[1] + self.VACUUM_PX_HALF/2)):
                for j in range((cell[0] - self.VACUUM_PX_HALF/2), (cell[0] + self.VACUUM_PX_HALF/2)):
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
            self.saveReturnPoint(self.currentCell)
        if eCell[0] == 0 and eCell[1] == 0:
            self.saveReturnPoint(self.currentCell)
        if wCell[0] == 0 and wCell[1] == 0:
            self.saveReturnPoint(self.currentCell)
        if sCell[0] == 0 and sCell[1] == 0:
            self.saveReturnPoint(self.currentCell) 
             
             
    def saveReturnPoint(self, p):
        saved = False
        for i in range(len(self.returnPoints)): 
            if (self.returnPoints[i][0] == p[0]) and (self.returnPoints[i][1] == p[1]):
                saved = True
        if saved == False:
            # This point wasn't saved before
            self.returnPoints.append(p)
            print '                               Save return point', p
            
                  
    def checkReturnPoints(self):
        cont = 0
        index = []
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
                # There are no free neighbors, save the index
                index.append(i)
            cont = 0
                            
        for i in index:
            print '                               Remove: ', self.returnPoints[i] 
            self.returnPoints.pop(i)
                 
             
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
        nCell= cells[0]
        eCell= cells[1]
        wCell= cells[2]
        sCell= cells[3]
        critical = False
        if (nCell[0] > 0) and (nCell[1] > 0):
            if (eCell[0] > 0) and (eCell[1] > 0):
                if (wCell[0] > 0) and (wCell[1] > 0):
                    if (sCell[0] > 0) and (sCell[1] > 0):
                        critical = True                        
        return critical
 
 
    def savePath(self, cell):
        self.path.append(cell)

        
    
    ######   DRIVING FUNCTIONS   ######     
           
    def goNextCell(self):
        self.x = round(self.pose3d.getX(),1)
        self.y = round(self.pose3d.getY(),1)
        self.yaw = self.pose3d.getYaw()
        poseVacuum = [self.x, self.y]
        desviation = self.calculateDesv(poseVacuum, self.nextCell)
        self.controlDrive(desviation)
         
            
    def calculateDesv(self, poseVacuum, cell):
        # poseVacuum = [x1, y1] coord
        # cell = [x2, y2] pix
        xc, yc = self.pix2coord(cell[0], cell[1])
        cell = [round(xc,1), round(yc,1)]
        x, y = self.abs2rel(cell, poseVacuum, self.yaw)
        desv = math.degrees(math.atan2(y,x))
        print '\nMY POSE:   NEXT CELL:'
        print '  x:', self.x, '    xc:', cell[0]
        print '  y:', self.y, '    yc:', cell[1]
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
        w1 = 0.1
        w2 = 0.12

        if desv > 0: #LEFT
            self.controlDesv(desv, w1, w2)
        else: #RIGHT
            self.controlDesv(desv, -w1, -w2)
       
       
    def controlDesv(self, desv, w1, w2): 
        desv = abs(desv) 
        th1 = 1.5
        th2 = 12 
        v1 = 0.1
        v2 = 0.12
   
        if desv >= th2:
            self.motors.sendV(0)
            self.motors.sendW(w2)
            #print '...Turn ...', w2
        elif desv < th2 and desv >= th1:
            self.motors.sendV(v1)
            self.motors.sendW(w1)
            #print '...Go and turn ...', w1
        else:
            self.motors.sendV(v2)
            self.motors.sendW(0)
            #print '...Go straight...' 
                        
                             
    def checkArriveCell(self, cell):
        distMax = 0.05 #5 cm
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
        neighbors = self.calculateNeigh(self.currentCell)    
        myCells = []
        
        north = neighbors[0]
        east = neighbors[1]
        west = neighbors[2]
        south = neighbors[3]
        
        print '\nNEIGHBORS RETURN:'
        print '    north:', north
        print '    east:', east
        print '    west:', west
        print '    south:', south
        
        for n in neighbors:
            n1 = self.checkCell(n[1])
            n2 = self.checkCell(n[2])
            if n1 == 2 and n2 == 2: #Virtual Obstacle
                myCells.append(n[0])
    
        # Check the closest cell to the return point
        self.nextCell = self.checkMinDist(myCells, self.returnPoint)
        
        arrive = self.checkArriveCell(self.nextCell)
        if arrive == False:
            self.goNextCell()  
        else:
            print ('    VACUUM ARRIVED TO THE NEXT NEIGHBOR')
            self.currentCell = self.nextCell
            
              
    def execute(self):

        # TODO 
               
        self.sweep()
        self.paintMap()
        self.showMaps(1)
        
