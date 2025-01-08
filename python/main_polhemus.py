"""
Auteur : Paul Chaillou
Contact : paul.chaillou@inria.fr
Année : 2022
Propriétaire : Université de Lille - CNRS 
License : Non définie, mais développé dans une démarche Open-Source et Logiciel Libre avec volonté de partage et de travail collaboratif. Développé dans un but non-marchand, en cas d'utilisation commerciale, merci de minimiser les prix et de favoriser le partage gratuit de tout ce qui peut l'être. A utiliser dans des buts prenant en compte les questions éthiques et morales (si possible non-militaire, ne rentrant pas dans le cadre de compétition, de monopole, ou de favorisation d'interets privés).
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 25 17:23:53 2022

@author: pchaillo
"""

# import serial
# a tester !!! (sinon revenir au commit précédent)

import PolhemusUSB as tracking
import time as time # for performance estimation

# p = tracking.PolhemusUSB() # 

tracker  = tracking.PolhemusOvercoat(init_pos = [110,0,0],axis = [2,-1,0]) 

nom_fichier = "polhemus_record_03_test.txt"

f = open(nom_fichier,'a')

while 1 :
    t1 = time.time()
    [position,q] = tracker.get_data()
    t2 = time.time()
    print(t2-t1)
    print('Position  : '+str(position))
    f.write(str(position)+'\n')
    f.close()
    f = open(nom_fichier,'a')
    

    # senso = s.sensor
    # print(str(senso.GetPressure()))
    # print(str(s.sensor.GetPressure()))

# ser = serial.Serial('/dev/ttyACM0',9600) # choisir le bon port serie si windows ('com1',9600) par exemple

# f = open('fichier.txt','a')

# while 1 :
#     print(str(ser.readline()))
#     f.write(str(ser.readline())+'\n')
#     f.close()
#     f = open('fichier.txt','a')