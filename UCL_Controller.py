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
    def __init__(self,module,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        # self.step = step
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
            pres_tab = [(self.pressure[0].pressure.value),(self.pressure[1].pressure.value),(self.pressure[2].pressure.value)]
            S = "{:,.3f}".format(pres_tab[0]) + "," + "{:,.3f}".format(pres_tab[1]) + "," + "{:,.3f}".format(pres_tab[2]) + ',' + "{:,.3f}".format(0) + "," + "{:,.3f}".format(0) + "," + "{:,.3f}".format(0) +"\n"
        elif self.nb_module == 2 :
            pres_tab = [self.pressure[0].pressure.value,self.pressure[1].pressure.value,self.pressure[2].pressure.value,self.pressure[3].pressure.value,self.pressure[4].pressure.value,self.pressure[5].pressure.value]
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
