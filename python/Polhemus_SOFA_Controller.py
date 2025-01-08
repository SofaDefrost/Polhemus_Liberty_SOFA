"""
Auteur : Paul Chaillou
Contact : paul.chaillou@inria.fr
Année : 2022
Propriétaire : Université de Lille - CNRS 
License : Non définie, mais développé dans une démarche Open-Source et Logiciel Libre avec volonté de partage et de travail collaboratif. Développé dans un but non-marchand, en cas d'utilisation commerciale, merci de minimiser les prix et de favoriser le partage gratuit de tout ce qui peut l'être. A utiliser dans des buts prenant en compte les questions éthiques et morales (si possible non-militaire, ne rentrant pas dans le cadre de compétition, de monopole, ou de favorisation d'interets privés).
"""

#!/usr/bin/env python;
# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
import os
import csv
import time
import numpy
import math
import serial
import six
from spicy import *
from datetime import datetime
from . import utils as utils
from . import quat as qt
from . import PolhemusUSB as PolhemusUSB

# # alignement du quaternion = annulation de la rotation en X
def alignQuatX(q): # A METTRE DANS SPLIB ???? - From QuaternionUtils
    a = q.getEulerAngles()
    a[0]=0
    return qt.Quat.createFromEuler(a)

class PolhemusTracking_Rigid3(Sofa.Core.Controller):
        """
Fonction to upload Polhemus data in SOFA

        """
        def __init__(self,node, name,offset = [0,0,0],axis = [0,1,2],print_flag = False,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.stiffNode = node
            self.position = self.stiffNode.getObject(name)

            self.conv = 10 # conversion coefficient, to pass from cm to mm

            self.sensor = PolhemusUSB.PolhemusOvercoat(init_pos = offset,axis = axis, conversion_factor = self.conv) # only once ? 

            self.print_flag = print_flag

        def onAnimateBeginEvent(self,e):
            [pos,q2] = self.sensor.get_data()

            rigid3 = [pos[0],pos[1],pos[2],q2[0],q2[1],q2[2],q2[3]]

            if self.print_flag:
                print(rigid3)

            self.position.position = [rigid3]


class PolhemusTracking_Referential(Sofa.Core.Controller):
        """
Fonction to upload Polhemus data in SOFA

        """
        def __init__(self,node, name,axis = [0,1,2],print_flag = False,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.stiffNode = node
            self.position = self.stiffNode.getObject(name)

            self.conv = 10 # conversion coefficient, to pass from cm to mm

            offset = [ 330, -40,70]

            self.sensor = PolhemusUSB.PolhemusOvercoat_Pos(init_pos = offset,axis = axis, conversion_factor = self.conv) # only once ? 

            self.print_flag = print_flag

        def onAnimateBeginEvent(self,e):
            pos = self.sensor.get_data()
            print(pos)
            rigid3 = [pos[0],pos[1],pos[2],0,0,0,1]

            if self.print_flag:
                print(rigid3)

            self.position.position = [rigid3]


## Fonctions dans le controller polhemus utilisé pour controller Echelon (voir branche Echelon3_showroom sur Gitlab) :

    # def smoothPosition(self,inputPos,prevPos,alpha = 0.2): # utilisé pour filtrer le bruit sur le positionnement du capteur => moi je n'ai pas trop de bruit...
    #     outputPos=[0]*7
    #     for i in range(3):
    #         outputPos[i] = inputPos[i]*alpha + prevPos[i]*(1-alpha)
    #     qin = array([inputPos[3],inputPos[4],inputPos[5],inputPos[6]])
    #     qprev = array([prevPos[3],prevPos[4],prevPos[5],prevPos[6]])
    #     qout = Quaternion.slerp(qprev,qin,alpha)
    #     for i in range(4):
    #         outputPos[3+i] = qout[i]
    #     return outputPos
    #     # Exemple d'utilisation : position[sensor_i] = self.smoothPosition(newPosition,self.prevPositions[sensor_i])


    # def getInitialPosition(self): # permet de définir n'importe quelle position comme étant la position initiale du capteur à n'importe quel moment (et pas seulement à l'initialisation de la scène (malin))
    #     global scaleTranslation
    #     self.polhemus.UpdateSensors() # each time you need new data
    #     self.polhemus.UpdateSensors()
    #     self.polhemus.UpdateSensors()
    #     self.polhemus.UpdateSensors()
    #     self.polhemus.UpdateSensors()   # sale, mais UTILE
        
    #     self.initialTranslationOffset=deepcopy(sensors_calibrate_translation)
    #     self.initialRotationOffset=[array([0,0,0,1]),array([0,0,0,1])]

    #     print 'sensor target initial translation = ' + str(self.initialTranslationOffset)

    #     for sensor_i in range(2):
    #         translationSensor = self.polhemus.sensors[sensor_i].GetLastPosition()
    #         for i in range(3):
    #             self.initialTranslationOffset[sensor_i][i] -= translationSensor[i]*scaleTranslation
    #         rotation = self.polhemus.sensors[sensor_i].GetLastQuaterion()
    #         self.initialRotationOffset[sensor_i] = Quaternion.conj(array([rotation[1],rotation[2],rotation[3],rotation[0]]))
    #     print 'sensors_initial_translation = ' + str(self.initialTranslationOffset)
    #     print 'sensors_initial_rotation = ' + str(self.initialRotationOffset )

    # def onKeyPressed(self,k):  
    #     if k=='D': # on redéfinit la position de référence lorsque l'on appuie sur une touche.
    #         print 'Sensors calibration...'      
    #         self.getInitialPosition()
    #     return 0
 


## Exemple du capteur Aurora (similaire au Polhemus (UCL)) :


#            nb_module = module.nb_module
#            h_module = module.h_module
#            z_eff_pos = nb_module * h_module 
# AuroraTracking(stiffNode, mechanicalPosition, offset=[nb_mobule*h_module])
class AuroraTracking(Sofa.Core.Controller):
        """Doc string"""
        def __init__(self, child_name, name, offset=[0,0,0], *args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.RootNode = kwargs["RootNode"]        # aurora setting 
            self.settings_aurora = { "tracker type": "aurora", "ports to use" : [10]}
            self.tracker = NDITracker(self.settings_aurora)
            self.tracker.start_tracking()

            self.stiffNode = self.RootNode.getChild(child_name) # for the generic one
            self.position = self.stiffNode.getObject(name)

            # first frames are invalid so we drop a given number of them
            for frame_to_drop in range(10):
                self.tracker.get_frame()
            
            pos_raw = get_data()
            self.displacement = [-pos_raw[0]+offset[0], pos_raw[1]+offset[1], -pos_raw[2]+offset[2]]

        def get_data():
            self.aurora_frame = self.tracker.get_frame();
            data = self.aurora_frame[3][0] 
            
            x_i = data[0][3]
            y_i = data[1][3]
            z_i = data[2][3]
            return [x_i, y_i, z_i]

        def onAnimateBeginEvent(self,e):
            pos_raw = get_data()
            pos = [pos_raw[0] + self.displacement[0], pos_raw[1] + self.displacement[1],  pos_raw[2] + self.displacement[2]]
            self.position.position = [pos]

