import cv2
import numpy as np
import math

class Moving:
    def __init__(self):
        ######################### you can change these values ########################
        self.base     =300 #link length setting
        self.crank    =300
        self.coupler  =150
        self.follower =100
        self.offset1=50 #trajectory point setting: horizontal offset from coupler link
        self.offset2=30 #trajectory point setting: vertical offset from coupler link
        self.x = [200.0,250.0] #crank fixed point position
        ##############################################################################

        #value initialize
        self.w = [self.x[0]+self.base,self.x[1]]
        self.y = [0.0,0.0]
        self.z = [0.0,0.0]
        self.t = [0.0,0.0]
        self.c=0
        self.th1=0.0
        self.th2=0.0
        self.trajectoryx=[]
        self.trajectoryy=[]

        #initial pose finding process
        possible=False
        for i in range(361): #possible pose finding by rotating crank and follower links.
            self.y[0]=(self.x[0]+(self.crank*np.cos((45+i)*(math.pi/180.0))))
            self.y[1]=(self.x[1]+(self.crank*np.sin((45+i)*(math.pi/180.0))))
            for j in range(361):
                self.z[0]=(self.w[0]+(self.follower*np.cos((45+j)*(math.pi/180.0))))
                self.z[1]=(self.w[1]+(self.follower*np.sin((45+j)*(math.pi/180.0))))
                distance=math.sqrt(((self.z[0]-self.y[0])**2)#distance between crank and
                                +((self.z[1]-self.y[1])**2)) #followers moving points.
                if abs(distance-self.coupler)<2: #if error is smaller than 2 pixel,
                    self.th1=45+i #crank initial angle
                    self.th2=45+j #follower initial angle
                    possible=True #your link length setting is possible
                    break

        #control loop
        if possible==False: #if pose finding failed
            print "impossible link lengths!"
        if possible==True:
            while(self.c!=ord('q')): #press 'q' to end
                img=self.control() #control loop
                cv2.imshow('fourbar',np.flipud(img))
                self.c=cv2.waitKey(0)
            

    def control(self):
        img = 250*np.ones( (640,700,3), np.uint8) #image size 640 by 700
        increase=False #crank rotating direction

        if self.c==ord('d'): #if you press 'd'
            self.th1=self.th1-0.5 #crank angle -1 degree 
            increase=False
        elif self.c==ord('a'):
            self.th1=self.th1+0.5 #crank angle +1 degree
            increase=True
        #crank's moving point position update
        self.y[0]=(self.x[0]+(self.crank*np.cos((self.th1)*(math.pi/180.0))))
        self.y[1]=(self.x[1]+(self.crank*np.sin((self.th1)*(math.pi/180.0))))
        
        #follower angle finding process
        possible=False
        for k in range(10):
            self.z[0]=(self.w[0]+(self.follower*np.cos((self.th2-5+k)*(math.pi/180.0))))
            self.z[1]=(self.w[1]+(self.follower*np.sin((self.th2-5+k)*(math.pi/180.0))))
            distance=math.sqrt(((self.z[0]-self.y[0])**2)
                                +((self.z[1]-self.y[1])**2))
            if abs(distance-self.coupler)<2:
                self.th2=self.th2-5+k
                possible=True
                break
        # if possible follower angle does not exist        
        if possible==False:
            print "locked"
            if increase==True:
                self.th1=self.th1-0.5 #return moved angle
            elif increase==False:
                self.th1=self.th1+0.5 #return moved angle
            self.y[0]=(self.x[0]+(self.crank*np.cos((self.th1)*(math.pi/180.0))))
            self.y[1]=(self.x[1]+(self.crank*np.sin((self.th1)*(math.pi/180.0))))
        # if possible follower angle exist
        if possible==True:
            print "crank angle :",self.th1-359

        #coupler link's angle finding 
        dx=self.z[0]-self.y[0]
        dy=self.z[1]-self.y[1]
        c_angle=math.degrees(math.atan2(dy,dx)) #coupler's angle

        #trajectory point's position
        self.t[0]=(self.y[0]+(self.offset2*np.cos((c_angle+90)*(math.pi/180)))
                                     +(self.offset1*np.cos((c_angle)*(math.pi/180))))
        self.t[1]=(self.y[1]+(self.offset2*np.sin((c_angle+90)*(math.pi/180)))
                                     +(self.offset1*np.sin((c_angle)*(math.pi/180))))
        #orthogonal point's position(between trajectory point and coupler) 
        o=[0.0,0.0]
        o[0]=(self.y[0]+(self.offset1*np.cos((c_angle)*(math.pi/180))))
        o[1]=(self.y[1]+(self.offset1*np.sin((c_angle)*(math.pi/180))))
        #trajectory point position appending
        if possible==True:
            tpoint=self.t
            self.trajectoryx.append(tpoint[0])
            self.trajectoryy.append(tpoint[1])

        ####visualizing process####
        for i in range(len(self.trajectoryx)): #drawing trajectory
            cv2.circle(img, (int(round(self.trajectoryx[i])),int(round(self.trajectoryy[i]))), 3, (0,100,255), -1)
        #drawing points and lines
        cv2.circle(img, (int(round(self.t[0])),int(round(self.t[1]))), 3, (0,100,255), -1) 
        cv2.circle(img, (int(round(self.x[0])),int(round(self.x[1]))), 3, (255,0,0), -1)
        cv2.circle(img, (int(round(self.y[0])),int(round(self.y[1]))), 3, (0,255,0), -1)
        cv2.circle(img, (int(round(self.z[0])),int(round(self.z[1]))), 3, (0,0,255), -1)
        cv2.circle(img, (int(round(self.w[0])),int(round(self.w[1]))), 3, (255,255,0), -1)
        cv2.line(img, (int(round(self.x[0])),int(round(self.x[1]))), (int(round(self.y[0])),int(round(self.y[1]))), (255, 0, 0), 2)
        cv2.line(img, (int(round(self.y[0])),int(round(self.y[1]))), (int(round(self.z[0])),int(round(self.z[1]))), (0, 255, 0), 2)
        cv2.line(img, (int(round(self.z[0])),int(round(self.z[1]))), (int(round(self.w[0])),int(round(self.w[1]))), (0, 0, 255), 2)
        cv2.line(img, (int(round(self.w[0])),int(round(self.w[1]))), (int(round(self.x[0])),int(round(self.x[1]))), (255, 255, 0), 2)
        cv2.line(img, (int(round(self.t[0])),int(round(self.t[1]))), (int(round(o[0])),int(round(o[1]))), (255, 150, 100), 2)
        return img
        
        

if __name__ == '__main__':
    Moving()