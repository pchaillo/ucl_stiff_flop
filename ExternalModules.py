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
Created on Fri Oct 21 16:57:00 2022

@author: pchaillo
"""

# importing csv module
import csv
import numpy as np
from math import sin,cos, sqrt, acos, radians, dist
from spicy import copy

import meshing_functions as mf
from splib3.topology import remeshing as rf

# def conv_tab_from_ind_tab(ind_tab): # a mettre dans Remeshing_functions.py
#     conv_tab = []
#     for i in range(len(ind_tab)):
#         # conv_tab.append([ind_tab[i],i])
#         conv_tab.append([i,ind_tab[i]])
#     return conv_tab

def AddConstrainCircles(parent,circle_tab,circle_ind_tab,conv_tab): # A mettre dans SoftRobot ?
    """
    Fonction qui ajoute les ressorts autour des cavités pour éviter les déformations latérales
    """
    # print(circle_ind_tab)
    circle_tab_old = rf.new_idx_from_conv_tab(mesh= circle_ind_tab,tab=conv_tab) # pour remettre les anciens indices, et ainsi correspondre aux noeuds du maillage complet
    ind = 0
    for u in range(len(circle_tab_old)):
        ind = ind + 1
        cercle = circle_tab[u]
        cercle_ind = circle_ind_tab[u]
        cercle_ind_old = circle_tab_old[u]
        [new_circle_pt,new_ind_tab] = rf.ordering_circle(circle = cercle,ind_tab = cercle_ind_old) # pour récupérer les indices triés dans le sens horaire
        p1 = cercle[0]
        p2 = cercle[1]
        d = dist(p1,p2) # on suppose ici que tous les points d'un cercle de la cavité sont espacés de la même distance => refaire un code qui place les ressorts 1 à 1 pour être utilisable pour toutes les géométries ?
        NoeudCercle = parent.addChild("Cercle" + str(ind))
        new_ind_tab_2 = rf.shift_tab(tab= new_ind_tab) # tableau des indices décalés d'un point, pour relier chaque point du cercle au point suivant
        NoeudCercle.addObject("MeshSpringForceField", name="Springs" ,stiffness= 10000,indices1 =new_ind_tab, indices2 = new_ind_tab_2 ,length = d)# damping="4"

def GetConstrainedCavityFromFEM(points,quads,indices,parent,axis = 0): # A mettre dans SPLIB ?
    """
    Fonction qui va trier les points et le maillage passé en argument afin d'ajouter des ressorts pour contraindre la cavité et renvoyer les points, le maillage et le tableau de conversion poiur créer le noeud qui contient la cavité.
    """
    new_points, triangles,ind_tab = CylinderMeshFromROI(points,quads,indices,axis = 0)
    conv_tab = rf.conv_tab_from_ind_tab(ind_tab)
    [circles, ind_tab] = rf.circle_detection_regular(points = new_points, pt_per_slice = 12) # circles position good # detecte les positions des cercles le long des cavités
    AddConstrainCircles(parent=parent,circle_tab = circles,circle_ind_tab=ind_tab,conv_tab = conv_tab) # Pour créer les ressorts qui constraigenent les déformations latérales 

    return [new_points, triangles,conv_tab]

def CylinderMeshFromROI(points,quads,indices,axis = 0):
    [new_points, ind_tab,new_quads] = rf.remesh_from_axis(points = points,mesh = quads,axis= 0 ,old_indices=indices)
    triangles = rf.quad_2_triangles(quads=new_quads) 
    [circles, circles_ind_tab] = rf.circle_detection_regular(points = new_points, pt_per_slice = 12)
    [new_circle_tab,new_ind_tab_full] = rf.ordering_cylinder(circles,circles_ind_tab)
    l = len(new_ind_tab_full)

    # closing_tri = mf.close_cavity_2(ind_bottom = new_ind_tab_full[0],ind_top = new_ind_tab_full[l-1])

    closing_tri = mf.close_cavity_3(ind_bottom = new_ind_tab_full[0],ind_top = new_ind_tab_full[l-1])
    # closing_tri = rf.invers_normal(closing_tri)

    triangles = triangles + closing_tri

    return [new_points, triangles,ind_tab]

def CreatePneumaticCavity(mesh,parent,module,i,j,act_flag,points = "null"):
    """

    INPUT :
    - act_flag => boolean to choose direct (act_flag = 1) or inverse control (act_flag = 0) 
    """
    Cavity = parent.addChild("Bellow" + str(j+1)+ str(i+1))
    if points != "null" :
        Cavity.addObject("TriangleSetTopologyContainer", triangles = mesh,name="meshLoader",points = points)#, position =Boite_III_K.pointsInROI.value, points = Boite_III_K.indices.value, quad = Boite_III_K.quadIndices.value )
        Cavity.addObject('MechanicalObject', name='chambreA'+str(i+1),rotation=[0, 0 , 0])#,translation = [0,0,h_module*i])
    else  :
        Cavity.addObject("TriangleSetTopologyContainer", triangles = mesh,name="meshLoader")#, position =Boite_III_K.pointsInROI.value, points = Boite_III_K.indices.value, quad = Boite_III_K.quadIndices.value )
 
    #  # 90 on y
    if act_flag == 0 :
        Cavity.addObject('SurfacePressureActuator', name='SPC', template = 'Vec3d',triangles='@chambreAMesh'+str(i+1)+'.triangles',minPressure = module.min_pression,maxPressure = module.max_pression)#,maxPressureVariation = 20)#,valueType=self.value_type)
    elif  act_flag == 1 :
        Cavity.addObject('SurfacePressureConstraint', name='SPC', triangles='@chambreAMesh'+str(i+1)+'.triangles', value=module.init_pressure_value,minPressure = module.min_pression,maxPressure = module.max_pression, valueType=module.value_type)#,maxPressureVariation = 20)#,
    return Cavity