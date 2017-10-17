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
        

class MyAlgorithm2(threading.Thread):

    def __init__(self, pose3d, motors, laser, bumper):
        self.pose3d = pose3d
        self.motors = motors
        self.laser = laser
        self.bumper = bumper
        
        self.map = cv2.imread("resources/images/mapgrannyannie.png", cv2.IMREAD_GRAYSCALE)
        self.map = cv2.resize(self.map, (500, 500))

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)
        
 
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
      
        
    def goForward(self,v):
        self.motors.sendW(0)
        self.motors.sendV(v)
        self.changeMap()
        
    ######   MAP FUNCTIONS   ######
    
    def whitePixels(self):
        # Calculate the white pixels of the map
        numPixels = 0
        for i in range(0, self.map.shape[1]):
            for j in range(0, self.map.shape[0]):
                if self.map[i][j] == 255:
                    numPixels = numPixels + 1
        return numPixels
         
              
    def changeMap(self):
        # Change the value of the pixels depending on where the vacuum goes
        final_poses = self.RTVacuum() * np.matrix([[self.x], [self.y], [1], [1]]) * self.SCALE

        poseX = int(final_poses.flat[0])
        poseY = int(final_poses.flat[1])
 
        for i in range((poseY - self.VACUUM_SIZE), (poseY + self.VACUUM_SIZE)):
            for j in range((poseX - self.VACUUM_SIZE), (poseX + self.VACUUM_SIZE)):
                self.map[i][j] = self.ADD_VAL_MAP
                      
        cv2.imshow("MAP ", self.map)
        
        
    def execute(self):

        # TODO
        self.goForward(20)
