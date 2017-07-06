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
        
        self.grid = np.ones([500, 500], float)
        
        self.yaw = 0
        self.turn = False
        self.turnFound = True
        self.crash = False
        self.crashPerimeter = False
        self.horizontal = True
        self.numIteracion = 0
        self.time = 0
        self.saturation = False

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
        
        
    def RTy(self, angle, tx, ty, tz):
        RT = np.matrix([[math.cos(angle), 0, math.sin(angle), tx], [0, 1, 0, ty], [-math.sin(angle), 0, math.cos(angle), tz], [0,0,0,1]])
        return RT

    def RTVacuum(self):
        RTy = self.RTy(pi, 5.6, 4, 0)
        return RTy
        
    def reduceValueSquare(self, numRow, numColumn):
        # Reduce el valor de un cuadrado en concreto
        scale = 50
        for i in range((numColumn * scale), (numColumn*scale + scale)):
            for j in range((numRow * scale), (numRow*scale + scale)):
                if self.grid[i][j] != 0:
                    self.grid[i][j] = self.grid[i][j] - 1
        
    def reduceValueTime(self):
        # number of rows is 10 and number of columns is 10
        numRowsColumns = 10
        # Recorre la imagen entera
        for i in range(0, numRowsColumns):
            for j in range(0, numRowsColumns):
                self.reduceValueSquare(i, j)
            
        
    def changeValuesGrid(self):
        # Cambia el valor de la rejilla segun por donde pase la aspiradora
        x = self.pose3d.getX()
        y = self.pose3d.getY()
        scale = 50

        final_poses = self.RTVacuum() * np.matrix([[x], [y], [1], [1]]) * scale

        # grid 500 x 500 y queremos un grid de 10 x 10
        # Nos quedamos con la parte entera para saber en que cuadrado esta
        numX = int(final_poses.flat[0] / scale)
        numY = int(final_poses.flat[1] / scale)
        
        # Vuelvo a 500 x 500 para cambiar todos los valores del grid segun el cuadrado en el que este
        for i in range((numX * scale), (numX*scale + scale)):
            for j in range((numY * scale), (numY*scale + scale)):
                self.grid[j][i] = self.grid[j][i] + 10.0
        
        
    def showGrid(self):
        # Para que se muestre bien el grid
		maxVal = np.amax(self.grid) # Maximo valor de los pixeles
		if maxVal != 0:
		    # Guarda una copia de la imagen pero dividida entre el valor maximo para que no se muestre saturado
			nCopy = np.dot(self.grid, (1/maxVal))
		else:
			nCopy = self.grid
		cv2.imshow("Grid ", nCopy)
		
		
    def checkSaturation(self):
        saturation = False
        numRowsColumns = 10
        scale = 50
        numSquaresVisited = 0
        for i in range(0, numRowsColumns):
            for j in range(0, numRowsColumns):
                # Compruebo el primer pixel del cuadrado porque todos tienen el mismo valor
                valuePos = self.grid[j*scale][i*scale]
                if valuePos != 0:
                    numSquaresVisited = numSquaresVisited + 1
                    
        if numSquaresVisited < 3:
            saturation = True
        return saturation

        
    def checkCrash(self):
        for i in range(0, 350):
            # Returns 1 if it collides, and 0 if it doesn't collide
            crash = self.bumper.getBumperData().state
            if crash == 1:
                self.motors.sendW(0)
                self.motors.sendV(0)
                break
        return crash
        

    def turn90(self, angle1, angle2, yawNow):
        turn = True
        # Mira a la izq gira a la izq
        if (-0.4 <= self.yaw <= 0.4 or (-pi/2-0.4) <= self.yaw <= (-pi/2+0.4)) and (yawNow <= (angle1-0.115) or yawNow >= (angle1+0.115)):
            self.motors.sendV(0)
            self.motors.sendW(0.2)
        # Mira a la dcha gira a la dcha
        elif ((pi/2-0.4) <= self.yaw <= (pi/2+0.4) or (-pi+0.4) >= self.yaw or self.yaw >= (pi - 0.4)) and (yawNow <= (angle2-0.115) or yawNow >= (angle2+0.115)):
            self.motors.sendV(0)
            self.motors.sendW(-0.2)
        else:
            turn = False
        return turn
        

    def execute(self):

        # TODO
        
        # Time
        self.numIteracion = self.numIteracion + 1
        if self.numIteracion % 5 == 0:
            self.time = self.time + 1
            
        if self.saturation == False:

            if self.time % 5 == 0:
                # If 5 seconds have elapsed we reduce the value of the squares of the grid
                self.reduceValueTime()
            
            # Comprobar si hay atasco   
            if self.time != 0 and self.time % 60 == 0:
                self.saturation = self.checkSaturation()
                if self.saturation == True:
                    # Frena 
                    self.motors.sendV(0)
                    self.motors.sendW(0)   
                print('Saturation: ', self.saturation)
            
            # Show grid
            self.changeValuesGrid()
            self.showGrid()
                
        # Vacuum's poses
        x = self.pose3d.getX()
        y = self.pose3d.getY()
        yaw = self.pose3d.getYaw()
        
        # Check crash
        crash = self.checkCrash()
        
        print (crash)
        
        if self.saturation == False:
            if crash == 1:
                print ("CRAAASH")
                # Stop
                self.motors.sendW(0)
                self.motors.sendV(0)
                time.sleep(1)
                # Go backwards
                self.motors.sendV(-0.1)
                time.sleep(1)
                
                # Yaw 
                self.yaw = self.pose3d.getYaw()
                self.turn = False
                self.crash = True
                
            if self.turn == False and self.crash == True:
                print ("PRIMER GIRO")
                # Yaw
                yawNow = self.pose3d.getYaw()
                # Turn 90
                giro = self.turn90(pi/2, pi/2, yawNow)
                    
                if giro == False:
                    print ("GIRO HECHO")
                    self.turn = True
                    # Go backwards
                    self.motors.sendW(0)
                    time.sleep(2)
                    # Avanza un poco
                    self.motors.sendV(0.32)                                        
                    time.sleep(1)
                    self.turnFound = False
                    
                    
            elif self.turnFound == False and self.crash == True:
                print ("SEGUNDO GIRO")
                # Yaw
                yawNow = self.pose3d.getYaw()
                giro = self.turn90(pi, 0, yawNow)
                
                if giro == False:
                    self.turnFound = True
            
            else:
                print ("AVANZAR")
                # Go forward
                self.motors.sendW(0.0)
                time.sleep(1)
                self.motors.sendV(0.5)
                self.crash = False
                self.turn == True
                
        else:
            # There is saturation
            print ("RECORRER PERIMETRO")
            
            # Get the data of the laser sensor, which consists of 180 pairs of values
            laser_data = self.laser.getLaserData()
            # print laser_data.numLaser
            # print laser_data.distanceData[0], laser_data.distanceData[laser_data.numLaser -1]
            laserRight = laser_data.distanceData[0]/10 # Pasamos a cm
            
            
            if crash == 0 and self.crashPerimeter == False:
                # Avanzo hasta que encuentro un obstaculo
                self.motors.sendV(0.5)
            elif crash == 1:
                self.crashPerimeter = True
                print ("NUEVOOOO CRAAASH")
                # Stop
                self.motors.sendW(0)
                self.motors.sendV(0)
                time.sleep(1)
                # Go backwards
                self.motors.sendV(-0.1)
                time.sleep(1)
                         
            if self.crashPerimeter == True:
                distToObstacle = 30
                # Giro hasta que el obstaculo quede a la derecha
                if laserRight => distToObstacle:
                    self.motors.sendV(0)
                    self.motors.sendW(0.2)
                else: 
                    # Ya esta el obstaculo a la derecha
                    self.motors.sendW(0)
                    self.motors.sendV(0.5)
                    
                    # Esta en una esquina
                    # Ya no hay obstaculo a la derecha
            
            
            
            
        '''

        # Vacuum's pose
        x = self.pose3d.getX()
        y = self.pose3d.getY()
        yaw = self.pose3d.getYaw()
        
        for i in range(0, 350):
            # Devuelve 1 si choca y 0 si no choca
            crash = self.bumper.getBumperData().state
            if crash == 1:
                break
                
        print('crash:     ', crash)
        
        turn = False
        giro = True
        
        # Si esta chocando
        if crash == 1:
            # Frena
            self.motors.sendW(0)
            self.motors.sendV(0)
            time.sleep(1)
            
            # Retrocede
            self.motors.sendV(-0.1)
            time.sleep(1)
            
            if self.orientacion
            while giro == True:
                # Gira 90 grados
                yawNow = self.pose3d.getYaw()
                if self. horizontal == True:                    
                    giro = self.turn90(pi/2, pi/2, yawNow)
                else:
                    giro = self.turn90(0, 0, yawNow)
            time.sleep(2)
            
            # Avanza un poco
            self.motors.sendV(0.32)
            
            #newCrash = self.bumper.getBumperData().state
            for i in range(0, 100000):
                # Devuelve 1 si choca y 0 si no choca
                newCrash = self.bumper.getBumperData().state
                #print("detectando...",newCrash)
                if newCrash == 1:
                    self.motors.sendW(0)
                    self.motors.sendV(0)
                    break
                
            print('new crash:     ', newCrash)
            
            if newCrash == 1 and self.horizontal == True:
                # No puede avanzar mas hacia abajo
                time.sleep(1)
                self.horizontal = False
                
                # Frena
                #self.motors.sendW(0)
                #self.motors.sendV(0)
                time.sleep(1)
                
                # Retrocede
                self.motors.sendV(-0.1)
                time.sleep(1)
            elif newCrash == 1 and self.horizontal == False:
                time.sleep(1)
                # No puede avanzar mas a la derecha
                self.horizontal = True
                
                # Frena
                #self.motors.sendW(0)
                #self.motors.sendV(0)
                time.sleep(1)
                
                # Retrocede
                self.motors.sendV(-0.1)
                time.sleep(1)
            else:
                time.sleep(1)
                yaw = self.pose3d.getYaw()
                while turn == False:
                    # Vuelve a girar para recorrer la siguiente linea
                    yawNow = self.pose3d.getYaw()
                    if self.horizontal == True:
                        giro = self.turn90(pi, 0, yawNow)
                    else:
                        giro = self.turn90(-pi/2, pi/2, yawNow)
                        
                    if giro == False:
                        turn = True
        else:
            # Si no hay choque avanza en recto
            self.motors.sendW(0.0)
            time.sleep(1)
            self.motors.sendV(0.5)
        
        '''
