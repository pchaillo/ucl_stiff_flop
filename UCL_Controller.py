#!/usr/bin/env python;
# -*- coding: utf-8 -*-
import Sofa.Core
import Sofa.constants.Key as Key
from spicy import *
# from os import getcwd, chdir, mkdir
import os
from datetime import datetime
import csv
import time
import Connexion_Function_ucl as connect
import serial
import math
import numpy
import six
from sksurgerynditracker.nditracker import NDITracker # for Aurora tracking

# réunit toutes les fonctions python, permettant de :
# - controller les pressions appliqués sur le robot
# - Importer les données du capteur Aurora (pour enregistrer les positions réelles et faire la boucle fermée)

 ### - CONTROLLER - ###
class ArduinoPressure_UCL(Sofa.Core.Controller):
    # pour envoyer les pressions avec les valves connectées Arduino DUE
    # UCL SETUP
    def __init__(self,module,dt,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        if module.dyn_flag == 1 :
            self.time_step = dt
        else :
            self.time_step = 1 # pour ne pas prendre en compte l'application du time step sur l'actionnement
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.ecart = 0 # ecart entre la simulation et la réalité, en mm
        ind = -1
        self.pressure, txt_chmbre = connect.CavityConnect(RootNode=self.RootNode,module=module)
        # self.board = pyfirmata.Arduino('/dev/ttyACM0') # pyfirmata connexion
        # self.led = self.board.get_pin('d:13:o')

        ### Stefan version
        self.SerialObj1 = serial.Serial('/dev/ttyACM1', 115200, timeout=0.5) #port used by the arduino mega board
        # print("ééééé ARDUINO CONNEXION OK éééééééééé")
 

    def onAnimateBeginEvent(self, dt): 
        if self.nb_module == 1 :
            pres_tab = [(self.pressure[0].pressure.value/self.time_step),(self.pressure[1].pressure.value/self.time_step),(self.pressure[2].pressure.value/self.time_step)]
            S = "{:,.3f}".format(pres_tab[0]) + "," + "{:,.3f}".format(pres_tab[1]) + "," + "{:,.3f}".format(pres_tab[2]) + ',' + "{:,.3f}".format(0) + "," + "{:,.3f}".format(0) + "," + "{:,.3f}".format(0) +"\n"
        elif self.nb_module == 2 :
            pres_tab = [self.pressure[0].pressure.value/self.time_step,self.pressure[1].pressure.value/self.time_step,self.pressure[2].pressure.value/self.time_step,self.pressure[3].pressure.value/self.time_step,self.pressure[4].pressure.value/self.time_step,self.pressure[5].pressure.value/self.time_step]
            S = "{:,.3f}".format(pres_tab[0]) + "," + "{:,.3f}".format(pres_tab[1]) + "," + "{:,.3f}".format(pres_tab[2]) + ',' + "{:,.3f}".format(pres_tab[3]) + "," + "{:,.3f}".format(pres_tab[4]) + "," + "{:,.3f}".format(pres_tab[5]) +"\n"

        print(S)

        ByteStr = S.encode("utf-8")
        self.SerialObj1.write(ByteStr)

        # self.SerialObj1.write(bytes(S, 'utf-8')) # may be better (more stable)

class AuroraTracking(Sofa.Core.Controller):
        """Doc string"""
        def __init__(self, child_name, name, module, offset=[0,0,0], *args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.RootNode = kwargs["RootNode"]        # aurora setting 
            self.settings_aurora = { "tracker type": "aurora", "ports to use" : [10]}
            self.tracker = NDITracker(self.settings_aurora)
            self.tracker.start_tracking()

            self.stiffNode = self.RootNode.getChild(child_name) # for the generic one
            self.position = self.stiffNode.getObject(name)
            nb_module = module.nb_module
            h_module = module.h_module

            self.z_eff_pos = nb_module * h_module 
            # first frames are invalid so we drop a given number of them
            for frame_to_drop in range(10):
                self.tracker.get_frame()
            
            self.aurora_frame = self.tracker.get_frame();
            x_i = self.aurora_frame[3][0][0][3]
            y_i = self.aurora_frame[3][0][1][3]
            z_i = self.aurora_frame[3][0][2][3]
            self.displacement = [ -x_i, -y_i , - z_i ]

        def get_data():
            self.aurora_frame = self.tracker.get_frame();
            data = self.aurora_frame[3][0] 
            
            x_i = data[0][3]
            y_i = data[1][3]
            z_i = data[2][3]
            return [x_i, y_i, z_i]

        def onAnimateBeginEvent(self,e):
            self.aurora_frame = self.tracker.get_frame();

            x = self.aurora_frame[3][0][0][3]
            y = self.aurora_frame[3][0][1][3]
            z = self.aurora_frame[3][0][2][3]
            
            pos_raw = [x ,y ,z]
            print('raw position is')
            print(pos_raw)
            if ~math.isnan(pos_raw[0]):
                pos = [pos_raw[0] + self.displacement[0], pos_raw[1] + self.displacement[1],  self.z_eff_pos + (pos_raw[2] + self.displacement[2])]
            self.position.position = [pos]


class AuroraTracking_2_nodes(Sofa.Core.Controller):
        """Doc string"""
        def __init__(self, name, node,name2,node2 module, offset=[0,0,0], *args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
         #   self.RootNode = kwargs["RootNode"]        # aurora setting 
            self.settings_aurora = { "tracker type": "aurora", "ports to use" : [10]}
            self.tracker = NDITracker(self.settings_aurora)
            self.tracker.start_tracking()

            self.stiffNode = node # for the generic one
            self.position = self.stiffNode.getObject(name)
            self.stiffNode2 = node2 # for the generic one
            self.position2 = self.stiffNode.getObject(name2)
            nb_module = module.nb_module
            h_module = module.h_module

            self.z_eff_pos = nb_module * h_module 
            self.h_module = h_module
            # first frames are invalid so we drop a given number of them
            for frame_to_drop in range(10):
                self.tracker.get_frame()
            
            self.aurora_frame = self.tracker.get_frame();
            # read the first tracker
            x_i_1 = self.aurora_frame[3][0][0][3]
            y_i_1 = self.aurora_frame[3][0][1][3]
            z_i_1 = self.aurora_frame[3][0][2][3]
            self.displacement_1 = [ -x_i_1, -y_i_1, - z_i_1]
            # read the second tracker
            x_i_2 = self.aurora_frame[3][1][0][3]
            y_i_2 = self.aurora_frame[3][1][1][3]
            z_i_2 = self.aurora_frame[3][1][2][3]
            self.displacement_1 = [ -x_i_1, -y_i_1, - z_i_1]
	        self.displacement_2 = [ -x_i_2, -y_i_2, - z_i_2]
            
        def get_data():
            self.aurora_frame = self.tracker.get_frame();
            data = self.aurora_frame[3][0] 
            
            x_i = data[0][3]
            y_i = data[1][3]
            z_i = data[2][3]
            return [x_i, y_i, z_i]

        def onAnimateBeginEvent(self,e):
            self.aurora_frame = self.tracker.get_frame();

            x_1 = self.aurora_frame[3][0][0][3]
            y_1 = self.aurora_frame[3][0][1][3]
            z_1 = self.aurora_frame[3][0][2][3]
            x_2 = self.aurora_frame[3][1][0][3]
            y_2 = self.aurora_frame[3][1][1][3]
            z_2 = self.aurora_frame[3][1][2][3]
            pos_raw_1 = [x_1 ,y_1 ,z_1]
            pos_raw_2 = [x_2 ,y_2 ,z_2]
            print('raw position is')
            print(pos_raw_1)
            if ~math.isnan(pos_raw_1[0]):
                pos_1 = [pos_raw_1[0] + self.displacement_1[0], pos_raw_1[1] + self.displacement_1[1],  self.z_eff_pos + (pos_raw_1[2] + self.displacement_1[2])]
                pos_2 = [pos_raw_2[0] + self.displacement_2[0], pos_raw_2[1] + self.displacement_2[1],  self.self.h_module + (pos_raw_2[2] + self.displacement_2[2])]
            
            self.position.position = [pos_1]
            self.position2.position = [pos_2]
