#!/usr/bin/env python;
# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
from spicy import *
import math
import numpy as np
import six

try :
    import Connexion_Function_ucl as connect
except :
    import ucl_collaboration.Connexion_Function_ucl as connect

# Réunit toutes les fonctions python, permettant de réaliser des trajectoires

 ### - CONTROLLER - ###

class CircleTrajectory(Sofa.Core.Controller):
        """ 
        Génère un trajectoire circulaire à l'effecteur (plan x - y) [ Faire une version OK pour tous les plans ?]

        INPUT : 
        rayon = rayon du cercle de la trajectoire
        nb_iter : nombre d'itérations, de pas de calculs, à effectuer pour effectuer un cercle complet (donne la vitesse à laquelle est exécutée le cercle)
        node : noeud sur se trouve l'objet qui va effectuer un cercle
        name : nom de l'objet qui contient la position à faire varier
        circle_height : hauteur du cercle tracé par l'effecteur (selon l'axe z ici)
        module = variable stiff qui contient toutes les données du robot
        axis = variable qui donne le plan dans lequel on tracer le cercle ([1,1,0] = x,y,  [1,0,1] = x,z)

        Exemple : rootNode.addObject(CircleTrajectory(rayon =circle_radius, nb_iter = nb_iter_circle, node = goal2,name = 'goal2M0',circle_height = circle_height+0,module=stiff,axis = [0,1,1]))

        """
        def __init__(self,rayon,nb_iter,node,name,circle_height,module,axis = [1,1,0],*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.stiffNode = node # for the generic one
            self.position = self.stiffNode.getObject(name)
            self.nb_iter_d = 50 # nombre d'iteration pour réaliser le sement qui amène au bord u cercle
            self.nb_iter = nb_iter  # nb d'itération pour réaliser un cercle complet
            self.d_flag = 0 # flag, passe à 1 quand le placement sur le cercle est effectué
            self.iter = 1 # numérote (compte) les itérations
            self.r = rayon
            self.circle_height = circle_height
            self.h_effector = module.h_module * module.nb_module
            self.height_shift = self.h_effector - self.circle_height
            # print(self.height_shift)

            flag_temp = 0
            for k in range(len(axis)):
                if axis[k] == 0 :
                    self.perpen = k
                elif flag_temp == 0 :
                    self.p1 = k
                    flag_temp = 1
                else :
                    self.p2 = k

        def onAnimateBeginEvent(self,e):

            d = (self.position.position.value).copy()

            if self.d_flag == 0 :
                d[0][self.p1] = (self.iter/self.nb_iter_d)*self.r
                d[0][self.perpen] = self.h_effector - (self.iter/self.nb_iter_d)*self.height_shift
                if self.iter >= self.nb_iter_d:
                    self.d_flag = 1
                    self.iter = 1
            else :
                # in_iter = self.iter - self.nb_iter_d
                d[0][self.p1] = self.r*math.cos((self.iter/self.nb_iter)*2*math.pi);
                d[0][self.p2] = self.r*math.sin((self.iter/self.nb_iter)*2*math.pi);


            self.position.position = [d[0]]

            self.iter += 1

class SquareTrajectory(Sofa.Core.Controller):
    """
    Génère un trajectoire carré à l'effecteur

    INPUT : 
    length = longueur d'un coté
    nb_iter = nb d'itération pour réaliser un coté du carré 
    node : noeud sur se trouve l'objet qui va effectuer un cercle
    name : nom de l'objet qui contient la position à faire varier
    square_height : hauteur du carré tracé par l'effecteur (selon l'axe z ici)
    module = variable stiff qui contient toutes les données du robot
    axis = variable qui donne le plan dans lequel on tracer le carré ([1,1,0] = x,y,  [1,0,1] = x,z)

    Exemple : rootNode.addObject(SquareTrajectory(rayon =square_radius, nb_iter = nb_iter_square,node = goal2,name = 'goal2M0',square_height = square_height+0,module=stiff))
    """
    def __init__(self,length,nb_iter,node,name,square_height,module,axis = [1,1,0],*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        self.stiffNode = node # for the generic one
        self.position = self.stiffNode.getObject(name)
        self.nb_iter_d = nb_iter # nombre d'iteration pour réaliser le sement qui amène au bord u cercle
        self.nb_iter = nb_iter  # nb d'itération pour réaliser la trajectoire
        self.d_flag = 0 # flag, passe à 1 quand le placement sur le cercle est effectué
        self.iter = 1 # numérote les itérations
        self.r = length
        self.square_height = square_height
        self.h_effector = module.h_module * module.nb_module
        # self.height_shift = self.h_effector - self.circle_height
        # print(self.height_shift)

        flag_temp = 0
        for k in range(len(axis)):
            if axis[k] == 0 :
                self.perpen = k
            elif flag_temp == 0 :
                self.p1 = k
                flag_temp = 1
            else :
                self.p2 = k

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
            d[0][self.p1] = (self.iter/(self.nb_iter_d/2))*self.r
            if self.iter >= self.nb_iter_d/2:
                self.d_flag = 1
                self.iter = 0
        elif self.d_flag == 1 :
            d[0][self.p2] = (self.iter/(self.nb_iter_d/2))*self.r
            if self.iter >= self.nb_iter_d/2:
                self.d_flag = 2
                self.iter = 0
        elif self.d_flag == 2 :
            d[0][self.p1] = d[0][self.p1] - self.r/self.nb_iter_d
            if self.iter >= self.nb_iter_d*2:
                self.d_flag = 3
                self.iter = 0
        elif self.d_flag == 3 :
            d[0][self.p2] = d[0][self.p2] - self.r/self.nb_iter_d
            if self.iter >= self.nb_iter_d*2:
                self.d_flag = 4
                self.iter = 0
        elif self.d_flag == 4 :
            d[0][self.p1] = d[0][self.p1] + self.r/self.nb_iter_d
            if self.iter >= self.nb_iter_d*2:
                self.d_flag = 5
                self.iter = 0
        elif self.d_flag == 5 :
            d[0][self.p2] = d[0][self.p2] + self.r/self.nb_iter_d
            if self.iter >= self.nb_iter_d*2:
                self.d_flag = 2
                self.iter = 0

        self.position.position = [d[0]]

        self.iter += 1


class PatternTrajectory(Sofa.Core.Controller):
# Génère un trajectoire circulaire à l'effecteur # je ne sais plus si elle fonctionne => a verifier
        def __init__(self,rayon,nb_iter,child_name,name,square_height,module,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.RootNode = kwargs["RootNode"]
            self.stiffNode = self.RootNode.getChild(child_name) # for the generic one
            self.position = self.stiffNode.getObject(name)
            # self.stiffNode = self.RootNode.getChild('goal')
            # self.position = self.stiffNode.getObject('goalMO')
            self.nb_iter_d = 50 # nombre d'iteration pour réaliser le sement qui amène au bord u cercle
            self.nb_iter = nb_iter  # nb d'itération pour réaliser la trajectoire
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



class LineTrajectory(Sofa.Core.Controller):
    """
    Génère un trajectoire linéaire 

    INPUT : 
    p_begin = coordonnée du point à l'extrémités du début segment
    p_end = coordonnée du point à  l'extrémités de la fin du segment
    nb_iter = nb d'itération pour réaliser un coté du carré 
    node : noeud sur se trouve l'objet qui va effectuer un cercle
    name : nom de l'objet qui contient la position à faire varier
    square_height : hauteur du carré tracé par l'effecteur (selon l'axe z ici)
    module = variable stiff qui contient toutes les données du robot

    Exemple : rootNode.addObject(LineTrajectory(nb_iter=20,node = goal2,name = 'goal2M0',p_begin = [0, 0 , 55], p_end = [10, -10 , 55]))

    """
    def __init__(self,nb_iter,node,name,p_begin = [-10 , 0 , 58], p_end = [10, 0 , 58 ] , *args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)

        self.stiffNode = node # for the generic one
        self.position = self.stiffNode.getObject(name)

        self.nb_iter = nb_iter  # nb d'itération pour réaliser la trajectoire
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
    """
    Génère un trajectoire, contenant tous les points passés en argument

    INPUT : 
    point_tab = tableau des points à atteindre successivements
    err_d = erreur désirée => si elle est très grande, va juste rester le nb self.err_nb d'iteration à chaque position avant de passer à la suivante.
    nb_iter = nb d'itération pour réaliser un coté du carré 

    node : noeud sur se trouve l'objet qui va effectuer un cercle
    name : nom de l'objet qui contient la position à faire varier

    node_pos : noeud sur se trouve l'objet qui va mesure la position atteinte (en réalité et en simulation) => utilisés pour comprarer les positions désirés et les positions 
    name_pos : nom de l'objet qui contient la position 
    type_flag : pour savoir si le noeud passé en argument node_pos et name_pos est ou non une poutre (type_flag = 0 -> goal_point; type_flag = 1 -> poutre, type_flag = 2 -> fem avec indices des points a moyenner)
    indices : utilisés seulement dans le cas du FEM, pour savoir quels indices utiliser

    module = variable stiff qui contient toutes les données du robot
    shift  = décalage !!! A METTRE AU CLAIR !!! surement pour le décalage entre goal et goal2 -> vraiment utile ?

    Exemple : rootNode.addObject(PointPerPointTrajectory(node = goal,name = 'goalM0',module = stiff,point_tab = point_tab, node_pos = rigidFramesNode, name_pos = 'DOFs',err_d = 50,shift=0,beam_flag = 1))

    """
    def __init__(self, point_tab, err_d, node, name, node_pos, name_pos,module, shift,type_flag,indices = "null",*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)

        self.stiffNode = node # for the generic one
        self.position = self.stiffNode.getObject(name)

        self.stiffNode_pos = node_pos # for the generic one
        self.position_pos = self.stiffNode_pos.getObject(name_pos)

        self.iter = 1 # numérote les itérations

        self.p_tab = point_tab 

        self.flag = 0 # to know which point in the tab will we reach

        self.nb_poutre = module.nb_poutre

        self.err_d = err_d # desired error
        self.err_nb = 25 # it will recquire 50 iteration with the good error to go to the next point
        self.err_incr = 0

        self.shift = shift

        self.type_flag = type_flag

        if type_flag == 2:
            self.indices = indices

    def onAnimateBeginEvent(self,e):

        if self.type_flag == 1 :
            pos = self.position_pos.position.value[self.nb_poutre-1][0:3]
        elif self.type_flag == 0 :
            pos = self.position_pos.position[0]
        elif self.type_flag == 2 :
            [posx,posy,posz] = connect.get_mean_point(all_positions = self.position_pos,indices = self.indices)
            pos = [posx,posy,posz]

        d = (self.position.position.value).copy()


        d[0][0] = self.p_tab[self.flag][0]
        d[0][1] = self.p_tab[self.flag][1]
        d[0][2] = self.p_tab[self.flag][2]
        print('d is')
        print(d)
        self.position.position = [d[0]]

        d2 = d
        d2[0][2] = self.p_tab[self.flag][2] - self.shift

        #erreur = np.linalg.norm(pos-d2[0])
        erreur = 0
        print(erreur)

        #if self.iter % 10 == 0:
            #self.flag += 1
                #if self.flag >= len(self.p_tab):
                     #self.flag =0

        if erreur < self.err_d :
            self.err_incr += 1
            if self.err_incr >= self.err_nb :
                self.flag += 1
                self.err_incr = 0
                if self.flag >= len(self.p_tab):
                    self.flag =0
                    
        #else :
            #self.err_incr = 0

        self.iter += 1

