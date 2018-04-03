#!/usr/bin/env python
__author__ = 'jiesun'
#Date: 11 Oct. 2015
#Email: jie.sun@kcl.ac.uk
#2015 10 12 finish the palm geometry calculation
#2015 10 13 finish the hand forward kinematics

import numpy as np
import math as m

class Rotation:
    # Rotation about  axis
    def rotation_y(self,y_Angle):
        #y_Angle = m.radians(y_Angle)
        return np.array([[m.cos(y_Angle), 0, m.sin(y_Angle)],[0, 1, 0],[-m.sin(y_Angle), 0, m.cos(y_Angle)]],dtype = float)

    def rotation_x(self,x_Angle):
        #x_Angle = m.radians(x_Angle)
        return np.array([[1, 0, 0],[0, m.cos(x_Angle), -m.sin(x_Angle)],[0, m.sin(x_Angle), m.cos(x_Angle) ]], dtype=float)

    def rotation_z(self,z_Angle):
        #z_Angle = m.radians(z_Angle)
        return np.array([[m.cos(z_Angle), -m.sin(z_Angle), 0],[m.sin(z_Angle), m.cos(z_Angle), 0],[0, 0, 1]],dtype = float)



class Palm(Rotation):
    # Define the palm geometry
    angleAE = m.radians(120.0)
    angleAB = m.radians(42.0)
    angleBC = m.radians(80.0)
    angleCD = m.radians(80.0)
    angleDE = m.radians(42.0)
    palmRadius = 1.0
    posiE = np.array([0, 0, palmRadius])

    def __init__(self, palmJointA, palmJointE):
        self.palmJointA = m.radians(palmJointA)
        self.palmJointE = m.radians(palmJointE)


    def getPositionA(self):
        return np.dot(self.rotation_y(self.angleAE),self.posiE)

    def getPositionB(self):
        self.posiBMiddle = np.dot(np.dot(self.rotation_y(self.angleAE), self.rotation_z(self.palmJointA)),self.rotation_y(self.angleAB))
        return np.dot(self.posiBMiddle,self.posiE)

    def getPositionD(self):
        return np.dot(np.dot(self.rotation_z(-self.palmJointE), self.rotation_y(-self.angleDE)),self.posiE)

    def palmJointC(self):
        self.vectorBD = self.getPositionB() - self.getPositionD()
        self.distanceSquareBD = np.dot(self.vectorBD,self.vectorBD)
        self.cos_alpha_BD = 1.0-self.distanceSquareBD/2.0
        self.alpha_BD = m.acos(self.cos_alpha_BD)       
        self.angleBCD_value =(self.cos_alpha_BD - m.cos(self.angleBC) * m.cos(self.angleCD)) / (m.sin(self.angleBC)*m.sin(self.angleCD))
        if (self.angleBCD_value < -1.0):
            self.angleBCD_value = -1.0
        self.angleBCD = m.acos(self.angleBCD_value)
        
        if ((self.palmJointA <= 0) & (self.palmJointE <= 0)):
            self.palmJointCValue = self.angleBCD - m.pi
        elif((self.palmJointA > 0) & (self.palmJointE > 0)): 
            self.palmJointCValue = m.pi - self.angleBCD
        else:
            self.palmJointCValue = 0
        #print self.distanceSquareBD
        #print self.cosAngleBD
        #print self.angleBCD
        #print m.degrees(self.palmJointCValue)
        return self.palmJointCValue

    def palmJointD(self):
        self.vectorBE = self.getPositionB()-self.posiE
        self.distanceSquareBE = np.dot(self.vectorBE,self.vectorBE)
        self.cos_alpha_BE = 1.0 - self.distanceSquareBE / 2.0
        self.alpha_BE = m.acos(self.cos_alpha_BE)
        self.angleEDB = m.acos((self.cos_alpha_BE-m.cos(self.angleDE)*self.cos_alpha_BD) / (m.sin(self.angleDE)*m.sin(self.alpha_BD)))
        
        self.angleBDC_value = (m.cos(self.angleBC) - m.cos(self.angleCD)*self.cos_alpha_BD) / (m.sin(self.angleCD)*m.sin(self.alpha_BD))
        
        if(self.angleBDC_value > 1.0):
            self.angleBDC_value = 1.0
        self.angleBDC = m.acos(self.angleBDC_value)
        

        if ((self.palmJointA <= 0) & (self.palmJointE <= 0)):
            self.palmJointDValue = self.angleEDB + self.angleBDC - m.pi
        elif ((self.palmJointA > 0) & (self.palmJointE <= 0)):
            self.palmJointDValue = self.angleEDB + self.angleBDC - m.pi
        elif ((self.palmJointA <= 0) & (self.palmJointE > 0)):
            self.palmJointDValue = self.angleEDB + self.angleBDC - m.pi
        else:   
            self.palmJointDValue = m.pi - self.angleEDB - self.angleBDC 
        return self.palmJointDValue

    def palmJointB(self):
        self.vectorAD = self.getPositionA()-self.getPositionD()
        self.distanceSquareAD = np.dot(self.vectorAD,self.vectorAD)
        self.cos_alpha_AD = 1.0 - self.distanceSquareAD / 2.0
        self.alpha_AD = m.acos(self.cos_alpha_AD)
        self.angleABD = m.acos((self.cos_alpha_AD-m.cos(self.angleAB)*self.cos_alpha_BD) / (m.sin(self.angleAB)*m.sin(self.alpha_BD)))
        self.angleDBC_value = (m.cos(self.angleCD) - m.cos(self.angleBC)*self.cos_alpha_BD) / (m.sin(self.angleBC)*m.sin(self.alpha_BD))
        if (self.angleDBC_value > 1.0):
           self.angleDBC_value = 1.0
        self.angleDBC = m.acos(self.angleDBC_value)

        if ((self.palmJointA <= 0) & (self.palmJointE <= 0)):
            self.palmJointBValue = self.angleABD + self.angleDBC - m.pi
        elif((self.palmJointA > 0) & (self.palmJointE <= 0)):
            self.palmJointBValue = self.angleABD + self.angleDBC - m.pi
        else:    
            self.palmJointBValue = m.pi - self.angleABD - self.angleDBC
        return self.palmJointBValue




class HandFK (Palm):
    #parameters for the hand, unit: m
    fingerLowerLength = 0.05
    fingerUpperLength = 0.04586

    palmRadiusMiddle = 0.041
    palmRadiusSide = 0.0475

    middleFingerBaseLength = 0.020
    sideFingerBaseLength = 0.021

    deltaMiddle = m.radians(60) # for middle finger
    deltaSide = m.radians(20) # for side finger

    upperFingerPreBending = m.pi/4.
    fingerRadio = 3.0/7.0


    # for the 5 active joints of the hand, define the initial state
    def __init__(self, palmJointA = -30, palmJointE = -60 , leftFingerLower = 0, middleFingerLower = 0, rightFingerLower = 0):
        self.palmJointA = m.radians(palmJointA)
        self.palmJointE = m.radians(palmJointE)
        self.leftFingerLower = m.radians(leftFingerLower)
        self.middleFingerLower = m.radians(middleFingerLower)
        self.rightFingerLower = m.radians(rightFingerLower)
        #self.middleFingerUpper = self.middleFingerLower * self.fingerRadio - m.pi/12
        #self.leftFingerUpper = self.leftFingerLower * self.fingerRadio - m.pi/6
        #self.rightFingerUpper = self.rightFingerLower * self.fingerRadio + m.pi/6
        self.middleFingerUpper = self.middleFingerLower * self.fingerRadio + m.radians(90. * 3/7) - m.pi/16
        self.leftFingerUpper = self.leftFingerLower * self.fingerRadio + m.radians(90. * 3/7) - m.pi/4
        self.rightFingerUpper = self.rightFingerLower * self.fingerRadio + m.radians(90. * 3/7) - m.pi/4

    def middleFinger(self):
        self.middleR1 = self.rotation_y(self.deltaMiddle)
        self.middleR2 = np.array([[0 , 0, -1],[1, 0, 0],[0, -1 ,0]])
        self.middleR3 = self.rotation_z(self.middleFingerLower)
        self.middleR4 = self.rotation_z(self.middleFingerLower * self.fingerRadio + self.upperFingerPreBending)
        self.middleP1 = np.array([0, self.middleFingerBaseLength, self.palmRadiusMiddle])
        self.middleP2 = np.array([self.fingerLowerLength, 0, 0])
        self.middleP3 = np.array([self.fingerUpperLength, 0, 0])
        self.middleOrientationR123 = np.dot(np.dot(self.middleR1, self.middleR2), self.middleR3)
        self.middleOrientation = np.dot(self.middleOrientationR123, self.middleR4)
        self.middlePosition = np.dot(self.middleOrientation, self.middleP3) + np.dot(self.middleOrientationR123, self.middleP2) + np.dot(self.middleR1, self.middleP1)
        
        return self.middlePosition

    def leftFinger(self):
        # change orientation for the left finger
        self.joinC = self.palmJointC()
        self.leftPalm1 = self.rotation_z(-self.palmJointE)
        self.leftPalm2 = self.rotation_y(-self.angleDE)
        self.leftPalm3 = self.rotation_z(-self.palmJointD())
        self.leftPalm4 = self.rotation_y(-self.deltaSide)

        # calculate the forward kinematics
        self.leftR1 = np.dot(np.dot(self.leftPalm1, self.leftPalm2), np.dot(self.leftPalm3, self.leftPalm4))
        self.leftR2 = np.array([[0 , 0, -1],[1, 0, 0],[0, -1 ,0]])
        self.leftR3 = self.rotation_z(self.leftFingerLower)
        self.leftR4 = self.rotation_z(self.leftFingerLower * self.fingerRadio + self.upperFingerPreBending)
        self.leftP1 = np.array([0, self.sideFingerBaseLength, self.palmRadiusSide])
        self.leftP2 = np.array([self.fingerLowerLength, 0, 0])
        self.leftP3 = np.array([self.fingerUpperLength, 0, 0])
        self.leftOrientationR123 = np.dot(np.dot(self.leftR1, self.leftR2), self.leftR3)
        self.leftOrientation = np.dot(self.leftOrientationR123, self.leftR4)
        self.leftPosition = np.dot(self.leftOrientation, self.leftP3) + np.dot(self.leftOrientationR123, self.leftP2) + np.dot(self.leftR1, self.leftP1)
      
        return self.leftPosition

    def rightFinger(self):
        # change orientation for the right finger
        self.joinC = self.palmJointC()
        self.rightPalm1 = self.rotation_y(self.angleAE)
        self.rightPalm2 = self.rotation_z(self.palmJointA)
        self.rightPalm3 = self.rotation_y(self.angleAB)
        self.rightPalm4 = self.rotation_z(self.palmJointB())
        self.rightPalm5 = self.rotation_y(self.deltaSide)

        # calculate the forward kinematics
        self.rightR1 = np.dot(np.dot(np.dot(self.rightPalm1, self.rightPalm2), np.dot(self.rightPalm3, self.rightPalm4)), self.rightPalm5)
        self.rightR2 = np.array([[0 , 0, -1],[1, 0, 0],[0, -1 ,0]])
        self.rightR3 = self.rotation_z(self.rightFingerLower)
        self.rightR4 = self.rotation_z(self.rightFingerLower * self.fingerRadio + self.upperFingerPreBending)
        self.rightP1 = np.array([0, self.sideFingerBaseLength, self.palmRadiusSide])
        self.rightP2 = np.array([self.fingerLowerLength, 0, 0])
        self.rightP3 = np.array([self.fingerUpperLength, 0, 0])
        self.rightOrientationR123 = np.dot(np.dot(self.rightR1, self.rightR2), self.rightR3)
        self.rightOrientation = np.dot(self.rightOrientationR123, self.rightR4)
        self.rightPosition = np.dot(self.rightOrientation, self.rightP3) + np.dot(self.rightOrientationR123, self.rightP2) + np.dot(self.rightR1, self.rightP1)
        
        return self.rightPosition

    def get_joint_displacement(self):
        self.joint_displacement = [self.palmJointE + np.pi/3, -self.palmJointA - np.pi/3, -0.491072+self.palmJointD(), -self.palmJointB()+0.491072,
                                   -self.rightFingerLower, self.rightFingerUpper, self.middleFingerLower, -self.middleFingerUpper,
                                   -self.leftFingerLower, -self.leftFingerUpper]
        return self.joint_displacement

   # def leftFinger(self):

a = HandFK(-30, -30, -30, -30, -30)
print a.palmJointC()
print a.palmJointB()
print a.palmJointD()
print a.get_joint_displacement()














"""
print "-10 ----------------------------------------------------------"

b = Palm(-10, -10)
print b.palmJointC()
print b.palmJointD()
print b.palmJointB()



print "-30 ----------------------------------------------------------"
c = Palm(-30, -30)
print c.palmJointC()
print c.palmJointD()
print c.palmJointB()


print "-60   ----------------------------------------------------------"
a = Palm(-60,-60)
print a.palmJointC()
print a.palmJointD()
print a.palmJointB()


print "-30, -45 ----------------------------------------------------------"
d = Palm(-30, -45)
print d.palmJointC()
print d.palmJointD()
print d.palmJointB()


print "-60, -45 ----------------------------------------------------------"
e = Palm(-60, -45)
print e.palmJointC()
print e.palmJointD()
print e.palmJointB()
"""





