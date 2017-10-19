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
        

class MyAlgorithm3(threading.Thread):

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
        
        self.VACUUM_PX_SIZE = 8  
        self.VACUUM_SIZE = 0.32
        self.VIRTUAL_OBST = 128
        self.SCALE = 50 #50 px = 1 m
        self.V = 0.4
        self.W_R = -0.1
        self.W_L = 0.1
        
        '''
        self.x = self.pose3d.getX()
        self.y = self.pose3d.getY()
        self.yaw = self.pose3d.getYaw()
        '''
        self.x = None
        self.y = None
        self.yaw = None
        
        self.firstTurn = False
        self.secondTurn = False
        self.northCell = False
 
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
        
        
    def stopVacuum(self):
        self.motors.sendW(0)
        self.motors.sendV(0)
        print ('Stop vacuum')
      
        
    def goForward(self,v):
        self.motors.sendW(0)
        self.motors.sendV(v)
        print ('Go forward')
    
    
    def zigzag(self):
        # The northern direction has priority
        
        if self.x == None:
            self.x = self.pose3d.getX()
        if self.y == None:
            self.y = self.pose3d.getY()
        
        self.changeMap()
        
        north, east, west = self.checkCells()
          
        if north == 0:
            # Go to the north cell
            if self.northCell == False: 
                self.northCell = self.goNorthCell(self.y)
            else:
                print ('SIGUIENTE CELDA')
        '''
        else:
            # There is an obstacle in the north
            if eastCell == 0:
                if self.firstTurn == False:
                    self.firstTurn = self.turn('right', self.yaw, yawNow, 90)
                else:
                    # Go to the east cell
                    self.goForward(self.V)
                    self.secondTurn = self.turn('right', self.yaw, yawNow, 90)
                    
            self.stopVacuum()
        '''
    
    def goNorthCell(self, y):
        northCell = False
        rangeY = 0.1
        yNow = self.pose3d.getY()
        diff = abs(abs(y) - abs(yNow))
        print ('DIFF : ', diff)
        if (diff < (self.VACUUM_SIZE + rangeY)):
            self.goForward(self.V)
        else:
            print ('North cell')
            self.stopVacuum()
            northCell = True
        return northCell
        
        
    def turn(self, direction, yaw, yawNow, angle):
        yawDeg = self.toDegrees(yaw)
        yawNowDeg = self.toDegrees(yawNow)
        diff = abs(abs(yawDeg) - abs(yawNowDeg))
        #print ('DIFF', diff)
        turn = False
        rangeDeg = 0.5

        self.motors.sendV(0)
        if (diff > (angle + rangeDeg)) or (diff < (angle - rangeDeg)):
            if direction == 'left':
                self.motors.sendW(self.W_L)
            else:
                self.motors.sendW(self.W_R)
        else:
            self.stopVacuum()
            print ('Giro hecho')
            turn = True    
        return turn
        
    def toDegrees(self, angle):
        degrees = angle * 180/pi
        return degrees  
        
        
    ######   MAP FUNCTIONS   ######
              
    def changeMap(self):
        # Change the value of the pixels depending on where the vacuum goes
        final_poses = self.RTVacuum() * np.matrix([[self.x], [self.y], [1], [1]]) * self.SCALE

        poseX = int(final_poses.flat[0])
        poseY = int(final_poses.flat[1])
 
        for i in range((poseY - self.VACUUM_PX_SIZE), (poseY + self.VACUUM_PX_SIZE)):
            for j in range((poseX - self.VACUUM_PX_SIZE), (poseX + self.VACUUM_PX_SIZE)):
                self.map[i][j] = self.VIRTUAL_OBST
                      
        cv2.imshow("MAP ", self.map)
        
        
    def checkNorthCell(self, centerX, centerY):
        # center: the central position of the cell
        obstacle = 0
        virtualObst = 0
        
        final_poses = self.RTVacuum() * np.matrix([[centerX], [centerY-self.VACUUM_SIZE], [1], [1]]) * self.SCALE

        poseX = int(final_poses.flat[0])
        poseY = int(final_poses.flat[1])
 
        for i in range((poseY - self.VACUUM_PX_SIZE), (poseY + self.VACUUM_PX_SIZE)):
            for j in range((poseX - self.VACUUM_PX_SIZE), (poseX + self.VACUUM_PX_SIZE)):
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
            print ('NORTH: OBSTACLE')
        elif virtualObst == 1:
            northCell = 2
            print ('NORTH: VIRTUAL OBSTACLE')
        else:
            northCell = 0
            print ('NORTH: FREE')
            
        return northCell
                
                
    def checkWestCell(self, centerX, centerY):
        # center: the central position of the cell
        obstacle = 0
        virtualObst = 0
        
        final_poses = self.RTVacuum() * np.matrix([[centerX+self.VACUUM_SIZE], [centerY], [1], [1]]) * self.SCALE

        poseX = int(final_poses.flat[0])
        poseY = int(final_poses.flat[1])
 
        for i in range((poseY - self.VACUUM_PX_SIZE), (poseY + self.VACUUM_PX_SIZE)):
            for j in range((poseX - self.VACUUM_PX_SIZE), (poseX + self.VACUUM_PX_SIZE)):
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
            print ('WEST: OBSTACLE')
        elif virtualObst == 1:
            westCell = 2
            print ('WEST: VIRTUAL OBSTACLE')
        else:
            westCell = 0
            print ('WEST: FREE')
            
        return westCell 

        
    def checkEastCell(self, centerX, centerY):
        # center: the central position of the cell
        obstacle = 0
        virtualObst = 0
        
        final_poses = self.RTVacuum() * np.matrix([[centerX-self.VACUUM_SIZE], [centerY], [1], [1]]) * self.SCALE

        poseX = int(final_poses.flat[0])
        poseY = int(final_poses.flat[1])
 
        for i in range((poseY - self.VACUUM_PX_SIZE), (poseY + self.VACUUM_PX_SIZE)):
            for j in range((poseX - self.VACUUM_PX_SIZE), (poseX + self.VACUUM_PX_SIZE)):
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
            print ('EAST: OBSTACLE')
        elif virtualObst == 1:
            eastCell = 2
            print ('EAST: VIRTUAL OBSTACLE')
        else:
            eastCell = 0
            print ('EAST: FREE')
            
        return eastCell 
        
        
    def checkCells(self):
        northCell = self.checkNorthCell(self.x, self.y)  
        eastCell = self.checkEastCell(self.x, self.y)  
        westCell = self.checkWestCell(self.x, self.y)  
        return northCell, eastCell, westCell
              
                    
    def execute(self):

        # TODO

        self.zigzag()
        #self.changeMap()
        #self.checkNorthCell(self.x, self.y)
        #self.checkWestCell(self.x, self.y)
        #self.checkEastCell(self.x, self.y)

