"""
Auteur : Paul Chaillou
Contact : paul.chaillou@inria.fr
Année : 2022
Propriétaire : Université de Lille - CNRS 
License : Non définie, mais développé dans une démarche Open-Source et Logiciel Libre avec volonté de partage et de travail collaboratif. Développé dans un but non-marchand, en cas d'utilisation commerciale, merci de minimiser les prix et de favoriser le partage gratuit de tout ce qui peut l'être. A utiliser dans des buts prenant en compte les questions éthiques et morales (si possible non-militaire, ne rentrant pas dans le cadre de compétition, de monopole, ou de favorisation d'interets privés).
"""

 # coding=utf-8

import Sofa
# import SofaPython3
from math import sin,cos, sqrt, acos
import array
from Polhemus_SOFA_Controller import *

def createScene(rootNode):

    rootNode.addObject('VisualStyle', displayFlags="showCollision" )

    MeasuredPosition = rootNode.addChild('MeasuredPosition')
    MeasuredPosition.addObject('EulerImplicitSolver', firstOrder=True)
    MeasuredPosition.addObject('CGLinearSolver', iterations='1000',threshold="1e-5", tolerance="1e-5")
    MeasuredPosition.addObject('MechanicalObject', name='MeasuredPositionM0', position=[0, 0,0])
    MeasuredPosition.addObject('SphereCollisionModel', radius='2')#, group='1')
    MeasuredPosition.addObject('UncoupledConstraintCorrection')
                
    rootNode.addObject(PolhemusTracking_Rigid3(node = MeasuredPosition,name = 'MeasuredPositionM0', offset=[10,10,0]))

    return rootNode
