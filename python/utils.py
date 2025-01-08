"""
Auteur : Paul Chaillou
Contact : paul.chaillou@inria.fr
Année : 2022
Propriétaire : Université de Lille - CNRS 
License : Non définie, mais développé dans une démarche Open-Source et Logiciel Libre avec volonté de partage et de travail collaboratif. Développé dans un but non-marchand, en cas d'utilisation commerciale, merci de minimiser les prix et de favoriser le partage gratuit de tout ce qui peut l'être. A utiliser dans des buts prenant en compte les questions éthiques et morales (si possible non-militaire, ne rentrant pas dans le cadre de compétition, de monopole, ou de favorisation d'interets privés).
"""

# # from splib3.numerics import quat as qt # crée un dépendance inutile à SOFA, le fichier a finalement été copié collé
# import quat as qt # crée un dépendance inutile à SOFA, le fichier a finalement été copié collé

# from splib3.numerics import quat as qt
from . import quat as qt # fichier copié depuis splib (dans STLIB) pour ne pas avoir à télécharger les deux pour utiliser polhemus => faire un lien plus malin entre les deux pour ne pas avoir a copié coller (déployer splib comme une librairie avec pip ?)

def fit_referential(axis, q,inversion):
    a = q.getEulerAngles()
    # print("aaaaa")
    # print(a)
    b = [0,0,0]
    b[0] =  a[axis[0]] #*inversion[0]
    b[1] =  a[axis[1]] # + 1.57# #*inversion[1]
    b[2] =  a[axis[2]] #*inversion[2]
    # print(b)
    return qt.Quat.createFromEuler(b)

def pansement(axis, q,inversion): # a refaire de manière propre ! #TODO
    a = q.getEulerAngles()
    # print("aaaaa")
    # print(a)
    b = [0,0,0]

    # # V1
    # b[0] =  a[2]#*inversion[0]
    # b[1] =  -a[0] # + 1.57# #*inversion[1]
    # b[2] =  a[1]#*inversion[2]

    # V2
    b[0] =  a[2]#*inversion[0]
    b[1] =  a[0] # + 1.57# #*inversion[1]
    b[2] =  -a[1]#*inversion[2]

    # print(b)
    return qt.Quat.createFromEuler(b)