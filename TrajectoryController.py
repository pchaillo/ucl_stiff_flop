#!/usr/bin/env python;
# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
from spicy import *
# from spicy import distance
import math
import numpy as np
import six

# réunit toutes les fonctions python, permettant de réaliser des trajectoires
 ### - CONTROLLER - ###

class CircleTrajectory(Sofa.Core.Controller):
        """ Génère un trajectoire circulaire à l'effecteur """
        def __init__(self,rayon,nb_iter,node,name,circle_height,module,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.stiffNode = node # for the generic one
            self.position = self.stiffNode.getObject(name)
            self.nb_iter_d = 50 # nombre d'iteration pour réaliser le sement qui amène au bord u cercle
            self.nb_iter = nb_iter  # nb d'itération pour réaliser un cercle complet
            self.d_flag = 0 # flag, passe à 1 quand le placement sur le cercle est effectué
            self.iter = 1 # numérote les itérations
            self.r = rayon
            self.circle_height = circle_height
            self.h_effector = module.h_module * module.nb_module
            self.height_shift = self.h_effector - self.circle_height
            print(self.height_shift)

        def onAnimateBeginEvent(self,e):

            d = (self.position.position.value).copy()

            if self.d_flag == 0 :
                d[0][0] = (self.iter/self.nb_iter_d)*self.r
                d[0][2] = self.h_effector - (self.iter/self.nb_iter_d)*self.height_shift
                if self.iter >= self.nb_iter_d:
                    self.d_flag = 1
                    self.iter = 1
            else :
                # in_iter = self.iter - self.nb_iter_d
                d[0][0] = self.r*math.cos((self.iter/self.nb_iter)*2*math.pi);
                d[0][1] = self.r*math.sin((self.iter/self.nb_iter)*2*math.pi);


            self.position.position = [d[0]]

            self.iter += 1


class SquareTrajectory(Sofa.Core.Controller):
# Génère un trajectoire circulaire à l'effecteur
        def __init__(self,rayon,nb_iter,node,name,square_height,module,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.stiffNode = node # for the generic one
            self.position = self.stiffNode.getObject(name)
            self.nb_iter_d = nb_iter # nombre d'iteration pour réaliser le sement qui amène au bord u cercle
            self.nb_iter = nb_iter  # nb d'itération pour réaliser un cercle complet
            self.d_flag = 0 # flag, passe à 1 quand le placement sur le cercle est effectué
            self.iter = 1 # numérote les itérations
            self.r = rayon
            self.square_height = square_height
            self.h_effector = module.h_module * module.nb_module
            # self.height_shift = self.h_effector - self.circle_height
            # print(self.height_shift)

        def onAnimateBeginEvent(self,e):

            d = (self.position.position.value).copy()

            # if self.d_flag == 0 :
            #     d[0][0] = (self.iter/self.nb_iter_d)*self.r
            #     d[0][2] = self.h_effector - (self.iter/self.nb_iter_d)*self.height_shift
            #     if self.iter >= self.nb_iter_d:
            #         self.d_flag = 1
            #         self.iter = 1
            # else :
            #     # in_iter = self.iter - self.nb_iter_d
            #     d[0][0] = self.r*math.cos((self.iter/self.nb_iter)*2*math.pi);
            #     d[0][1] = self.r*math.sin((self.iter/self.nb_iter)*2*math.pi);
            if self.d_flag == 0 :
                d[0][0] = (self.iter/(self.nb_iter_d/2))*self.r
                if self.iter >= self.nb_iter_d/2:
                    self.d_flag = 1
                    self.iter = 0
            elif self.d_flag == 1 :
                d[0][1] = (self.iter/(self.nb_iter_d/2))*self.r
                if self.iter >= self.nb_iter_d/2:
                    self.d_flag = 2
                    self.iter = 0
            elif self.d_flag == 2 :
                d[0][0] = d[0][0] - self.r/self.nb_iter_d
                if self.iter >= self.nb_iter_d*2:
                    self.d_flag = 3
                    self.iter = 0
            elif self.d_flag == 3 :
                d[0][1] = d[0][1] - self.r/self.nb_iter_d
                if self.iter >= self.nb_iter_d*2:
                    self.d_flag = 4
                    self.iter = 0
            elif self.d_flag == 4 :
                d[0][0] = d[0][0] + self.r/self.nb_iter_d
                if self.iter >= self.nb_iter_d*2:
                    self.d_flag = 5
                    self.iter = 0
            elif self.d_flag == 5 :
                d[0][1] = d[0][1] + self.r/self.nb_iter_d
                if self.iter >= self.nb_iter_d*2:
                    self.d_flag = 2
                    self.iter = 0

            self.position.position = [d[0]]

            self.iter += 1


class PatternTrajectory(Sofa.Core.Controller):
# Génère un trajectoire circulaire à l'effecteur
        def __init__(self,rayon,nb_iter,child_name,name,square_height,module,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.RootNode = kwargs["RootNode"]
            self.stiffNode = self.RootNode.getChild(child_name) # for the generic one
            self.position = self.stiffNode.getObject(name)
            # self.stiffNode = self.RootNode.getChild('goal')
            # self.position = self.stiffNode.getObject('goalMO')
            self.nb_iter_d = 50 # nombre d'iteration pour réaliser le sement qui amène au bord u cercle
            self.nb_iter = nb_iter  # nb d'itération pour réaliser un cercle complet
            self.d_flag = 0 # flag, passe à 1 quand le placement sur le cercle est effectué
            self.iter = 1 # numérote les itérations
            self.r = rayon
            self.square_height = square_height
            self.h_effector = module.h_module * module.nb_module
            # self.height_shift = self.h_effector - self.circle_height
            # print(self.height_shift)
            # self.x = 0
            # self.y = 0
            self.ratio = 20

        def onAnimateBeginEvent(self,e):

            d = (self.position.position.value).copy()

            if self.d_flag == 0 :
                d[0][0] = -(self.iter/self.nb_iter_d)*self.r
                d[0][1] = -(self.iter/self.nb_iter_d)*self.r
                if self.iter >= self.nb_iter_d:
                    self.d_flag = 1
                    # self.iter = 1
            else :
                for x in range(2*self.r*self.ratio):
                    d[0][0] = x/self.ratio - self.r
                    for y in range(2*self.r*self.ratio):
                        if x % 2 == 0:
                            d[0][1] = y/self.ratio - self.r
                        # else :
                        #     d[0][1] = self.r - y/self.ratio



            self.position.position = [d[0]]

            self.iter += 1

class LineTrajectory(Sofa.Core.Controller):
# Génère un trajectoire circulaire à l'effecteur
        def __init__(self,nb_iter,node,name,p_begin = [-10 , 0 , 58], p_end = [10, 0 , 58 ] , *args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)

            self.stiffNode = node # for the generic one
            self.position = self.stiffNode.getObject(name)

            self.nb_iter = nb_iter  # nb d'itération pour réaliser un cercle complet
            self.iter = 0 # numérote les itérations

            self.begin = p_begin 
            self.end = p_end

        def onAnimateBeginEvent(self,e):

            if self.iter <= self.nb_iter :

                d = (self.position.position.value).copy()

                d[0][0] = ((self.nb_iter - self.iter)/self.nb_iter)*self.begin[0] + (self.iter/self.nb_iter)*self.end[0]
                d[0][1] = ((self.nb_iter - self.iter)/self.nb_iter)*self.begin[1] + (self.iter/self.nb_iter)*self.end[1]
                d[0][2] = ((self.nb_iter - self.iter)/self.nb_iter)*self.begin[2] + (self.iter/self.nb_iter)*self.end[2]

                self.position.position = [d[0]]

            self.iter += 1


class PointPerPointTrajectory(Sofa.Core.Controller):
# Génère un trajectoire circulaire à l'effecteur
        def __init__(self, point_tab, err_d, node, name, node_pos, name_pos,module, shift,beam_flag,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)

            self.stiffNode = node # for the generic one
            self.position = self.stiffNode.getObject(name)

            self.stiffNode_pos = node_pos # for the generic one
            self.position_pos = self.stiffNode_pos.getObject(name_pos)

            self.iter = 0 # numérote les itérations

            self.p_tab = point_tab 

            self.flag = 0 # to know which point in the tab will we reach

            self.nb_poutre = module.nb_poutre

            self.err_d = err_d # desired error
            self.err_nb = 10 # it will recquire 50 iteration with the good error to go to the next point
            self.err_incr = 0

            self.shift = shift

            self.beam_flag = beam_flag

        def onAnimateBeginEvent(self,e):

            if self.beam_flag == 1 :
                pos = self.position_pos.position.value[self.nb_poutre-1][0:3]
            else :
                pos = self.position_pos.position[0]

            d = (self.position.position.value).copy()

            d[0][0] = self.p_tab[self.flag][0]
            d[0][1] = self.p_tab[self.flag][1]
            d[0][2] = self.p_tab[self.flag][2]

            self.position.position = [d[0]]

            d2 = d
            d2[0][2] = self.p_tab[self.flag][2] - self.shift

            erreur = np.linalg.norm(pos-d2[0])
            print(erreur)

            if erreur < self.err_d :
                self.err_incr += 1
                if self.err_incr >= self.err_nb :
                    self.flag += 1
                    if self.flag >= len(self.p_tab):
                        self.flag =0
            else :
                self.err_incr = 0

            self.iter += 1
