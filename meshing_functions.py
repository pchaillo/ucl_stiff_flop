#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 21 16:57:00 2022

@author: pchaillo
"""

# importing csv module
import numpy as np
from collections import OrderedDict
from splib3.topology import Remeshing_functions as rf

import Sofa
from splib3.numerics import Vec3, Quat, sdiv

# def closeSurface(ind_tab, reccur_bool = 0):
#     """
#     crée un ensemble de triangles pour refermer une surface, dont les indices des points triés dans le sens horaire sont passés en argument
#     """
# #    if reccur_bool == 0 :
#     triangles = []
#     new_ind = []
#     for k in range(0,len(ind_tab)-1,2) :
#         # print(k)
#         ind_a = k
#         ind_b = k + 1
#         ind_c = k + 2
#         if ind_c > len(ind_tab)-1 :
#             ind_c = 0

#         triangles.append( [ ind_tab[ind_a] ,ind_tab[ind_b] ,ind_tab[ind_c] ] )
#         new_ind.append(ind_tab[ind_a] )
#         new_ind.append(ind_tab[ind_c] )
    
#     new_ind = list(OrderedDict.fromkeys(new_ind))
#     if len(new_ind) >= 3:
#         new_triangles = closeSurface(ind_tab = new_ind, reccur_bool = 1)
#         for tri in new_triangles :
#             triangles.append(tri)

#     return triangles

def getBarycenter(selectedPoints):
    poscenter = [0., 0., 0.]
    if len(selectedPoints) != 0:
            poscenter = sdiv(sum(selectedPoints), float(len(selectedPoints)))
    return poscenter

def AddBeamMapping(BeamNode,ModelNode,MixedFEM):
    DeformableParts = MixedFEM.getChild('DeformableParts')
    RigidParts = MixedFEM.getChild('RigidParts')
    # print('yy')
    # print(type(DeformableParts))
    MixedFEM.addObject('MechanicalMatrixMapper',template='Rigid3d,Rigid3d', object1='@../../DOFs', object2='@../../DOFs', nodeToParse='@./DeformableParts' )  # je ne projete que dans la structure globale => mechanical object
    # BeamNode.addObject('MechanicalMatrixMapper',template='Rigid3d,Rigid3d', object1='@DOFs', object2='@DOFs', nodeToParse='@./././DeformableParts' )  # je ne projete que dans la structure globale => mechanical object

    DeformableParts.addObject('AdaptiveBeamMapping', interpolation='@../../../BeamInterpolation', input='@../../../DOFs', output='@./dofs')# , useCurvAbs = False ) # OK => "Plus que"
    RigidParts.addObject('AdaptiveBeamMapping', interpolation='@../../../BeamInterpolation', input='@../../../DOFs', output='@./dofs')# , useCurvAbs = False ) # OK => "Plus que"



def Rigidify2(targetNode, sourceNode, sourceObject, groupIndices, frames=None, name=None):
        """ Transform a deformable object into a mixed one containing both rigid and deformable parts.

            :param targetObject: parent node where to attach the final object.
            :param sourceObject: node containing the deformable object. The object should be following
                                 the ElasticMaterialObject template. EN CHGT Pour mettre un MechanicalObject en argument
            :param list groupIndices: array of array indices to rigidify. The length of the array should be equal to the number
                                      of rigid component.
            :param list frames: array of frames. The length of the array should be equal to the number
                                of rigid component. The orientation are given in eulerAngles (in degree) by passing
                                three values or using a quaternion by passing four values.
                                [[rx,ry,rz], [qx,qy,qz,w]]
                                User can also specify the position of the frame by passing six values (position and orientation in degree)
                                or seven values (position and quaternion).
                                [[x,y,z,rx,ry,rz], [x,y,z,qx,qy,qz,w]]
                                If the position is not specified, the position of the rigids will be the barycenter of the region to rigidify.
            :param str name: specify the name of the Rigidified object, is none provided use the name of the SOurceObject.
        """
        if frames is None:
            frames = [[0., 0., 0.]]*len(groupIndices)

        assert len(groupIndices) == len(frames), "size mismatch."

        if name is None:
            # name = sourceObject.name #eul
            name = "DefaultRigidificationName"
        # sourceObject.reinit()
        ero = targetNode.addChild(name)

        # allPositions = sourceObject.container.position.value #eul
        allPositions = sourceObject.position.value #eul
        allIndices =list(range(len(allPositions)))

        allPositions = np.array(allPositions, dtype='float32')

        rigids = []
        indicesMap = []

        def mfilter(si, ai, pts):
                tmp = []
                for i in ai:
                        if i in si:
                                tmp.append(pts[i])
                return tmp
        
        # print("zz") # Pour correction automatique quand le format de données n'est pas bon
        # print(str(type(groupIndices)))
        # if type(groupIndices) == "<class 'numpy.ndarray'" :
        #     print("OKOKOK molo")
        #     groupIndices = list(groupIndices)


        # get all the points from the source.
        selectedIndices = []

        for i in range(len(groupIndices)):
                # print("zz")
                # print(type(groupIndices[1]))
                # print(type(allIndices[1]))
                # print(type(allPositions[1]))
                print(groupIndices)

                selectedPoints = mfilter(groupIndices[i], allIndices, allPositions)
                if len(frames[i]) == 3:
                        orientation = Quat.createFromEuler(frames[i], inDegree=True)
                        poscenter = getBarycenter(selectedPoints)
                elif len(frames[i]) == 4:
                        orientation = frames[i]
                        poscenter = getBarycenter(selectedPoints)
                elif len(frames[i]) == 6:
                        orientation = Quat.createFromEuler([frames[i][3], frames[i][4], frames[i][5]], inDegree=True)
                        poscenter = [frames[i][0], frames[i][1], frames[i][2]]
                elif len(frames[i]) == 7:
                        orientation = [frames[i][3], frames[i][4], frames[i][5], frames[i][6]]
                        poscenter = [frames[i][0], frames[i][1], frames[i][2]]
                else:
                        Sofa.msg_error("Do not understand the size of a frame.")

                rigids.append(poscenter + list(orientation))

                selectedIndices += map(lambda x: x, groupIndices[i])
                indicesMap += [i] * len(groupIndices[i])

        otherIndices = list(filter(lambda x: x not in selectedIndices, allIndices))
        Kd = {v: None for k, v in enumerate(allIndices)}
        Kd.update({v: [0, k] for k, v in enumerate(otherIndices)})
        Kd.update({v: [1, k] for k, v in enumerate(selectedIndices)})
        indexPairs = [v for kv in Kd.values() for v in kv]

        freeParticules = ero.addChild("DeformableParts")
        freeParticules.addObject("MechanicalObject", template="Vec3", name="dofs",
                                    position=[allPositions[i] for i in otherIndices])

        rigidParts = ero.addChild("RigidParts")
        rigidParts.addObject("MechanicalObject", template="Rigid3", name="dofs", reserve=len(rigids), position=rigids)

        rigidifiedParticules = rigidParts.addChild("RigidifiedParticules")
        rigidifiedParticules.addObject("MechanicalObject", template="Vec3", name="dofs",
                                          position=[allPositions[i] for i in selectedIndices])
        rigidifiedParticules.addObject("RigidMapping", name="mapping", globalToLocalCoords=True, rigidIndexPerPoint=indicesMap)

        # if "solver" in sourceObject.objects: #eul => maintenant on vérife à la main qu'il n'y a pas de solver sur ces noeuds
        #     sourceObject.removeObject(sourceObject.solver)
        # if "integration" in sourceObject.objects:
        #     sourceObject.removeObject(sourceObject.integration)
        # if "correction" in sourceObject.objects:
        #     sourceObject.removeObject(sourceObject.correction)

        sourceNode.addObject("SubsetMultiMapping", name="mapping", template="Vec3,Vec3",
                              input=[freeParticules.dofs.getLinkPath(),rigidifiedParticules.dofs.getLinkPath()],
                              # output=sourceObject.dofs.getLinkPath(), #eul
                              output=sourceNode.getLinkPath(),
                              indexPairs=indexPairs)

        rigidifiedParticules.addChild(sourceNode)
        freeParticules.addChild(sourceNode)
        return ero


def close_cavity(circles,ind_tab): # dirty => you may do better my boy
    """
    Fonction qui en fonction des cercles, va créer les triangles pour fermer le maillage du cylindre aux extrémités

    INPUT : 
    circles = tableau qui contient les cercles du cylindre (tableau de tous les indices des points, un ligne du tableau représentant un cercle
    ind_tab = tableau des indices 

    OUTOUT :
    new_triangles = tableau des triangles à ajouter pour fermer les cylindres
    """
    circle_bottom = circles[0]
    ind_bottom = ind_tab [0]
    # print(ind_bottom)
    l = len(circles)
    circle_top = circles[l-1]
    ind_top = ind_tab[l-1]
    print(ind_top)
    print(len(ind_top))
    
    new_triangles = []
    nb_pt_per_slices = len(ind_top)
    print(nb_pt_per_slices)
    for i in range(6):
        i = i*2
        print(i)
        ind_a = i
        ind_b = i + 2
        ind_c = i + 1
        # print([ind_a,ind_b,ind_c])
        if ind_b == np.ceil(nb_pt_per_slices):
            # print("Y ALLONS NOUS ? je vais savoir bientpot")
            ind_b = 0
        print([ind_a,ind_b,ind_c])
        
        new_triangles.append( [ ind_top[ind_a] ,ind_top[ind_b] ,ind_top[ind_c] ] )
        new_triangles.append( [ ind_bottom[ind_a] ,ind_bottom[ind_b] ,ind_bottom[ind_c] ] )
        
        ind_a = 0
        ind_b = 10
        ind_c = 2
        new_triangles.append( [ ind_top[ind_a] ,ind_top[ind_b] ,ind_top[ind_c] ] )
        new_triangles.append( [ ind_bottom[ind_a] ,ind_bottom[ind_b] ,ind_bottom[ind_c] ] )
        
        ind_a = 4
        ind_b = 8
        ind_c = 6
        new_triangles.append( [ ind_top[ind_a] ,ind_top[ind_b] ,ind_top[ind_c] ] )
        new_triangles.append( [ ind_bottom[ind_a] ,ind_bottom[ind_b] ,ind_bottom[ind_c] ] )
        
        ind_a = 4
        ind_b = 2
        ind_c = 10
        new_triangles.append( [ ind_top[ind_a] ,ind_top[ind_b] ,ind_top[ind_c] ] )
        new_triangles.append( [ ind_bottom[ind_a] ,ind_bottom[ind_b] ,ind_bottom[ind_c] ] )
        
        ind_a = 10
        ind_b = 4
        ind_c = 8
        new_triangles.append( [ ind_top[ind_a] ,ind_top[ind_b] ,ind_top[ind_c] ] )
        new_triangles.append( [ ind_bottom[ind_a] ,ind_bottom[ind_b] ,ind_bottom[ind_c] ] )
    
    return new_triangles

def close_cavity_2(ind_top,ind_bottom): # dirty => you may do better my boy
    """
    Fonction qui en fonction des cercles, va créer les triangles pour fermer le maillage du cylindre aux extrémités

    INPUT : 
    ind_top = indices du cercle à l'extrémité supérieure du cylindre
    ind_bottom = indices du cercle à l'extrémité inférieure du cylindre

    OUTOUT :
    new_triangles = tableau des triangles à ajouter pour fermer les cylindres
    """
    
    new_triangles = []
    nb_pt_per_slices = len(ind_top)
    # print(nb_pt_per_slices)
    for i in range(6):
        i = i*2
        # print(i)
        ind_a = i
        ind_b = i + 2
        ind_c = i + 1
        # print([ind_a,ind_b,ind_c])
        if ind_b == np.ceil(nb_pt_per_slices):
            # print("Y ALLONS NOUS ? je vais savoir bientpot")
            ind_b = 0
        # print([ind_a,ind_b,ind_c])
        
        new_triangles.append( [ ind_top[ind_a] ,ind_top[ind_b] ,ind_top[ind_c] ] )
        new_triangles.append( [ ind_bottom[ind_c] ,ind_bottom[ind_b] ,ind_bottom[ind_a] ] )
        
    ind_a = 0
    ind_b = 10
    ind_c = 2
    new_triangles.append( [ ind_top[ind_a] ,ind_top[ind_b] ,ind_top[ind_c] ] )
    new_triangles.append( [ ind_bottom[ind_c] ,ind_bottom[ind_b] ,ind_bottom[ind_a] ] )
    
    ind_a = 4
    ind_b = 8
    ind_c = 6
    new_triangles.append( [ ind_top[ind_a] ,ind_top[ind_b] ,ind_top[ind_c] ] )
    new_triangles.append( [ ind_bottom[ind_c] ,ind_bottom[ind_b] ,ind_bottom[ind_a] ] )
    
    ind_a = 2
    ind_b = 10
    ind_c = 4
    new_triangles.append( [ ind_top[ind_a] ,ind_top[ind_b] ,ind_top[ind_c] ] )
    new_triangles.append( [ ind_bottom[ind_c] ,ind_bottom[ind_b] ,ind_bottom[ind_a] ] )
    
    ind_a = 4
    ind_b = 10
    ind_c = 8
    new_triangles.append( [ ind_top[ind_a] ,ind_top[ind_b] ,ind_top[ind_c] ] )
    new_triangles.append( [ ind_bottom[ind_c] ,ind_bottom[ind_b] ,ind_bottom[ind_a] ] )
    
    return new_triangles


def close_cavity_3(ind_top,ind_bottom): # dirty => you may do better my boy
    """
    Fonction qui en fonction des cercles, va créer les triangles pour fermer le maillage du cylindre aux extrémités

    INPUT : 
    ind_top = indices du cercle à l'extrémité supérieure du cylindre
    ind_bottom = indices du cercle à l'extrémité inférieure du cylindre

    OUTOUT :
    new_triangles = tableau des triangles à ajouter pour fermer les cylindres
    """

    # triangles_top = closeSurface(ind_top)
    # triangles_bottom = closeSurface(ind_bottom)
    # triangles = triangles_top + triangles_bottom

    triangles = rf.closeSurface(ind_top)
    triangles = rf.invers_normal(triangles)
    triangles_bottom = rf.closeSurface(ind_bottom)

    for i in triangles_bottom :
        triangles.append(i)
    
    return triangles
