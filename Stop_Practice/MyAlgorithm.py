import numpy as np
import threading
import time
from datetime import datetime
import jderobot
import math
import cv2

time_cycle = 80
        

class MyAlgorithm(threading.Thread):

    def __init__(self, pose3d, camera, motors):
        self.camera = camera
        self.pose3d = pose3d
        self.motors = motors

        #self.imageRight=None
        self.image=None
        self.template = cv2.imread('resources/template.png',0) # 0 para coger la imagen en escala de grises

        # Car direction
        self.carx = 0.0
        self.cary = 0.0

        # Obstacles direction
        self.obsx = 0.0
        self.obsy = 0.0


        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)


    def setImageFiltered(self, image):
        self.lock.acquire()
        self.lock.release()

    def getImageFiltered(self):
        self.lock.acquire()
        tempImage=self.image
        self.lock.release()
        return tempImage

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

    def execute(self):

        # TODO
        
        # GETTING THE IMAGES
        input_image = self.camera.getImage()

        # RGB model change to HSV
        hsv_image = cv2.cvtColor(input_image, cv2.COLOR_RGB2HSV)

        # Values of red
        value_min_HSV = np.array([131, 71, 0])
        value_max_HSV = np.array([179, 232, 63])

        # Segmentacion
        image_seg = cv2.inRange(hsv_image, value_min_HSV, value_max_HSV)
        cv2.imshow("Segmentacion", image_seg)

        # Close, morphology element
        kernel = np.ones((11,11), np.uint8)
        image_filtered = cv2.morphologyEx(image_seg, cv2.MORPH_CLOSE, kernel)
        cv2.imshow("Cierre ", image_filtered)
        
        detection = False
        
        # Tamano de la imagen template
        h, w = self.template.shape
        
        # Matching con la imagen template
        # match : grayscale image, where each pixel denotes how much does the neighbourhood of that pixel match with template
        match = cv2.matchTemplate(image_filtered,self.template,cv2.TM_CCOEFF_NORMED) 
        cv2.imshow('Matching', match)
        threshold = 0.8 # 0 seria negro y 1 blanco 
        loc = np.where( match >= threshold) # 2 arrays (x e y) con las posiciones que superan el umbral
        for pt in zip(*loc[::-1]): # zip.(): Recorre ambos arrays a la vez y devuelve una tupla con ambos valores
            cv2.rectangle(input_image, pt, (pt[0] + w, pt[1] + h), (0,0,255), 2) # Recuadra el stop en azul en imput
            cv2.rectangle(image_filtered, pt, (pt[0] + w, pt[1] + h), (255,0,0), 2) # Recuadra el stop en la imagen filtrada
            detection = True
            print("Found signal")
            self.motors.sendV(0)

        if detection == False:
            self.motors.sendV(50)
        
        
        
        '''
        # GETTING THE IMAGES
        input_image = self.camera.getImage()

        # Converting the original image into grayscale
        image_gray = cv2.cvtColor(input_image, cv2.COLOR_BGR2GRAY) 

        # Thresholding the grayscale image to get better results
        retval, threshold = cv2.threshold(image_gray, 30, 50, cv2.THRESH_BINARY_INV)
        
        # Close, morphology element
        kernel = np.ones((8,8), np.uint8) #eliminamos la palabra 'STOP'
        image_filtered = cv2.morphologyEx(threshold, cv2.MORPH_CLOSE, kernel)
        
        # Detect edges using canny
        canny_output = cv2.Canny(image_filtered, 100, 100 * 2)
        #cv2.imshow("image canny", canny_output)
        
        image2, contours, hierachy = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours: #Recorro los contornos
            # Approximates a polygonal curve(s) with the specified precision.
            #epsilon = 0.1*cv2.arcLength(cnt,True) #Cuanto mas grande mas suavizo
            approx = cv2.approxPolyDP(cnt,0.01*cv2.arcLength(cnt,True),True)

            # Recorremos los contornos y contamos las rectas para saber si es un octogono
            if len(approx) == 8:
                # Encontramos el octogono
                cv2.drawContours(input_image,[cnt],0,(0,255,0),-1)#lo dibujamos en verde en input
                cv2.drawContours(image_filtered,[cnt],0,(255,255,255),-1)#en blanco en filtered
                # Recuadro el stop
                x, y, w, h = cv2.boundingRect(cnt) #x,y : coordenadas del inicio del rectangulo; w: anchura; h: altura
                cv2.rectangle(input_image, (x,y), (x+w, y+h), (0,0,255), 2)#Mostramos el rectangulo en la imagen
                if h >= 40 and w >= 40:
                    print("Found signal")
                    self.motors.sendV(0)
               
        if len(contours) == 0:      
            self.motors.sendV(30)
        
        #cv2.imshow('image filtered', image_filtered)
        '''
        
        
        
        
        
        
        
        
        
