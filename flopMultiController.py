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
import serial
import math
import numpy
import six
# from sksurgerynditracker.nditracker import NDITracker # for Aurora tracking
# import polhemus_liberty.python.PolhemusUSB as PolhemusUSB # for Polhemus tracking

try :
    import Connexion_Function_ucl as connect
except :
    import ucl_collaboration.Connexion_Function_ucl as connect

# réunit toutes les fonctions python, permettant de :
# - controller le robot en pression/volume
# - imprimer les positions de l'effecteur dans le terminal
# - enregistrer les temps, pressions et positions correspondantes dans des fichiers txt ou csv

 ### - CONTROLLER - ###
class StiffController(Sofa.Core.Controller):

    # def __init__(self,pas,max_pression,nb_module,nb_cavity,*args, **kwargs):
    def __init__(self,pas,module,*args, **kwargs):

            Sofa.Core.Controller.__init__(self,args,kwargs)
            # self.RootNode = args[0]
            self.RootNode = kwargs["RootNode"]
            self.pressure, txt_chmbre = connect.CavityConnect(RootNode=self.RootNode,module=module)
            self.flag = 0;
            self.pas = pas
            self.max_pression = module.max_pression
            self.nb_module = module.nb_module
            self.nb_cavity = module.nb_cavity
            if module.dyn_flag == 1:
                self.time_step = module.dt
            else :
                self.time_step = 1



    def onKeypressedEvent(self,e):
        
            if e["key"] == Key.T: # switche d'un module à un autre pour l'actionnement
                if self.flag < self.nb_module - 1:
                    self.flag = self.flag + 1
                    print('Switch au mondule n° : ',self.flag+1)
                    # print(self.flag)
                else:
                    self.flag = 0
                    print('Switch au mondule n° : ',self.flag+1)
                    # print(self.flag)

            pressureValue = numpy.zeros(self.nb_cavity)

            index = self.flag*self.nb_cavity
            for i in range(self.nb_cavity):
                pressureValue[i] = self.pressure[index+i].value.value[0]

            if e["key"] == Key.D:
                pressureValue[0] += self.pas
                # print('===========D')
                if pressureValue[0] > self.max_pression:
                    pressureValue[0]= self.max_pression
            if e["key"] == Key.C:
                pressureValue[0] -= self.pas
                if pressureValue[0] < 0:
                    pressureValue[0] = 0

            if e["key"] == Key.F:
                # print('===========F')
                pressureValue[1] += self.pas
                if pressureValue[1] > self.max_pression:
                    pressureValue[1] = self.max_pression
            if e["key"] == Key.V: # S déjà pris, je met F à la place
                pressureValue[1] -= self.pas
                if pressureValue[1] < 0:
                    pressureValue[1] = 0

            if e["key"] == Key.G:
                # print('==========G')
                pressureValue[2] += self.pas
                if pressureValue[2] > self.max_pression:
                    pressureValue[2] = self.max_pression
            if e["key"] == Key.B:
                pressureValue[2] -= self.pas
                if pressureValue[2] < 0:
                    pressureValue[2] = 0

            if self.nb_cavity > 3:
                if e["key"] == Key.H:
                    # print('==========G')
                    pressureValue[3] += self.pas
                    if pressureValue[3] > self.max_pression:
                        pressureValue[3] = self.max_pression
                if e["key"] == Key.N:
                    pressureValue[3] -= self.pas
                    if pressureValue[3] < 0:
                        pressureValue[3] = 0
                        
            print('         ****       ')
            print('Control du mondule n° : ',self.flag+1)
            for i in range(self.nb_cavity): # remise valeurs au bon endroit
                self.pressure[index+i].value = [pressureValue[i]]
                print('Pression chambre ',i,' : ',pressureValue[i]/self.time_step)
            print('         ****       ')

### - VIEWER - ###
class PositionViewer(Sofa.Core.Controller):
    def __init__(self,nb_poutre,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        self.node = args[0]
        self.stiffNode=self.node.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = nb_poutre

    def onKeypressedEvent(self,e):
        pos = self.position.position.value[self.nb_poutre-1][0:3]
        print('         ----       ')
        print('Position effecteur : ',pos)
        print('         ----       ')

class GoalKeyboardController(Sofa.Core.Controller):
    # pour controller la position de l'effecteur désiré avec le clavier
    # Controle le point avec décalage
        def __init__(self,goal_pas,node,name,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.stiffNode = node # for the generic one
            self.position = self.stiffNode.getObject(name)

            self.pas = goal_pas

        def onKeypressedEvent(self,e):

            d = (self.position.position.value).copy

            if e["key"] == Key.D:
                d[0][0] += self.pas  
            if e["key"] == Key.C:
                d[0][0] -= self.pas  

            if e["key"] == Key.F:
                d[0][1] += self.pas  
            if e["key"] == Key.V:
                d[0][1] -= self.pas  

            if e["key"] == Key.G:
                d[0][2] += self.pas  
            if e["key"] == Key.B:
                d[0][2] -= self.pas 

            self.position.position = [d[0]]

class ArduinoPressure(Sofa.Core.Controller): # à déporter dans le fichier pressure_actuation
    # pour envoyer les pressions avec les valves connectées à la MegaPi 
    # DEFROST SETUP
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

        if module.dyn_flag == 1:
            self.time_step = module.dt
        else :
            self.time_step = 1

        # self.board = pyfirmata.Arduino('/dev/ttyACM0') # pyfirmata connexion
        # self.led = self.board.get_pin('d:13:o')

        ### Stefan version
        self.SerialObj1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5) #port used by the arduino mega board
        # print("ééééé ARDUINO CONNEXION OK éééééééééé")
 

    def onAnimateBeginEvent(self, dt): 
        pres_tab = [copy(self.pressure[0].pressure.value)/self.time_step,copy(self.pressure[1].pressure.value)/self.time_step,copy(self.pressure[2].pressure.value)/self.time_step]
        # print(pres_tab)

        bar_tab = connect.kPa_to_bar(pres_tab)

        S = "{:,.3f}".format(bar_tab[0]) + "," + "{:,.3f}".format(bar_tab[1]) + "," + "{:,.3f}".format(bar_tab[2]) + "\n"

        print(S)
        ByteStr = S.encode("utf-8")

        self.SerialObj1.write(ByteStr)
        # time.sleep(0.2) #usefull ?
        # print("Step: " + str(i) + ", Bytes sent: " + S)
        # print("  Bytes sent: " + S)

        # # pour le test avec la led
        # pres = pres_tab[0]
        # if pres > 50 :
        #     self.led.write(1)
        # else :
        #     self.led.write(0)

class PressurePrinter_local(Sofa.Core.Controller):
    # Pour print les pressions dans le terminal 
    def __init__(self,module,node,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        # self.stiffNode = node
        # self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        # self.step = step
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.ecart = 0 # ecart entre la simulation et la réalité, en mm
        ind = -1
        self.pressure, txt_chmbre = connect.CavityConnect2(node=node,module=module)
        if module.dyn_flag == 1:
            self.time_step = module.dt
        else :
            self.time_step = 1
 

    def onAnimateBeginEvent(self, dt): 
        # pres_tab = [copy(self.pressure[0].pressure.value),copy(self.pressure[1].pressure.value),copy(self.pressure[2].pressure.value),copy(self.pressure[3].pressure.value),copy(self.pressure[4].pressure.value),copy(self.pressure[5].pressure.value)]
        if self.nb_module == 1 :
            pres_tab = [self.pressure[0].pressure.value/self.time_step,self.pressure[1].pressure.value/self.time_step,self.pressure[2].pressure.value/self.time_step]
        elif self.nb_module == 2:
            pres_tab = [self.pressure[0].pressure.value/self.time_step,self.pressure[1].pressure.value/self.time_step,self.pressure[2].pressure.value/self.time_step,self.pressure[3].pressure.value/self.time_step,self.pressure[4].pressure.value/self.time_step,self.pressure[5].pressure.value/self.time_step]
        print(str(pres_tab))
        # print(str(pres_tab.to_list()))

class GoalShift(Sofa.Core.Controller):
        """ pour controller la position de l'effecteur désiré avec le clavier """
        def __init__(self,node_follow ,object_follow,node_master,object_master,shift_tab = [0,0,5],*args, **kwargs):
            Sofa.Core.Controller.__init__(self,args,kwargs)
            self.stiffNode = node_follow
            self.position = self.stiffNode.getObject(object_follow)
            self.stiffNode = node_master
            self.position2 = self.stiffNode.getObject(object_master)
            self.shift_tab = shift_tab

        def onAnimateBeginEvent(self,e):

            d = (self.position.position.value).copy()

            d2 = (self.position2.position.value).copy()

            d[0][0] = d2[0][0] - self.shift_tab[0]
            d[0][1] = d2[0][1] - self.shift_tab[1]
            d[0][2] = d2[0][2] - self.shift_tab[2]

            self.position.position = [d[0]]

### - CSV PRINTER - ###
class PositionPrinterCsv(Sofa.Core.Controller):
    def __init__(self,module,child_name,name,nom_dossier,beam_ind = 'null',*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        # self.node = args[0]
        self.RootNode = kwargs['RootNode']
        self.stiffNode = self.RootNode.getChild(child_name) # for the generic one
        self.position = self.stiffNode.getObject(name)
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity

        path = os.getcwd()
        if not (os.path.exists(path + '/record/'+ nom_dossier)) :
            os.mkdir(path + '/record/'+ nom_dossier)

        if beam_ind != 'null':
            child_name = child_name + '_' + str(beam_ind)

        self.nf, self.fichier_csv = connect.OpenPrintFile2('.csv',child_name,nom_dossier)
        self.start = time.time()

        self.beam_ind = beam_ind

    def onAnimateBeginEvent(self, dt): 
    # def onKeypressedEvent(self,e):
        # print(self.nf)
        if self.beam_ind == "null":
            pos = numpy.array(self.position.position[0])
            # print(pos)
        else : 
            if self.beam_ind == ceil(self.beam_ind):
                pos = self.position.position.value[self.beam_ind][0:3]
            else : 
                # ### si la valeur n'est pas entière, que le milieu des deux modules tombe entre deux poutres, on prend la moyenne de ces poutres # INUTILE FINALEMENT, LORS DES TESTS JE VOIS QUE POS_A EST TOUJOURS BON
                # print("77777777777777777777777777__0")
                # print([ self.beam_ind , ceil(self.beam_ind)])
                # [pos_a_x,pos_a_y,pos_a_z]  = self.position.position.value[int(ceil(self.beam_ind))][0:3]
                # [pos_b_x,pos_b_y,pos_b_z]  = self.position.position.value[int(ceil(self.beam_ind)+1)][0:3]
                # pos = [ (pos_a_x+pos_b_x)/2, (pos_a_y+pos_b_y)/2, (pos_a_z+pos_b_z)/2 ]
                # print([pos_a_x,pos_a_y,pos_a_z])
                # print([pos_b_x,pos_b_y,pos_b_z])
                # print(pos)
                # print("77777777777777777777777777__1")
                # ###
                pos = self.position.position.value[int(ceil(self.beam_ind))][0:3]
                # print("44444444")
                # print(pos)

        ind = 0
        # pres = []
        print(str(time.time() - self.start)) 
        time_txt = ", [" + str(time.time() - self.start) + "]"

        self.fichier_csv.write(str(pos) + time_txt +'\n')

        self.fichier_csv.close()
        print("%%%% Positions Enregistrées en Csv %%%%")
        self.fichier_csv = open(self.nf,'a')

class PressurePrinterCsv(Sofa.Core.Controller):
    def __init__(self,module,nom_dossier,act_flag,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        # self.node = args[0]
        self.RootNode = kwargs['RootNode']
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        self.pressure, txt_chmbre = connect.CavityConnect(RootNode=self.RootNode,module=module)
        # nom_fichier = child_name 

        path = os.getcwd()
        if not (os.path.exists(path + '/record/'+ nom_dossier)) :
            os.mkdir(path + '/record/'+ nom_dossier)

        self.nf, self.fichier_csv = connect.OpenPrintFile2('.csv','Pressure',nom_dossier)
        self.start = time.time()

        self.act_flag = act_flag

    def onAnimateBeginEvent(self, dt): 
    # def onKeypressedEvent(self,e):
        # print(self.nf)
        # if self.beam_flag == 1 : 
        #     pos = self.position.position.value[self.nb_poutre-1][0:3]
        # else :
        #     pos = array(self.position.position[0])
            
        ind = 0
        # pres = []
        print(str(time.time() - self.start)) 
        time_txt = ", [" + str(time.time() - self.start) + "]"
        pres_txt = ""
        for i in range(self.nb_module):
            pres_txt = pres_txt + "["
            i0 = ind
            for j in range(self.nb_cavity):
                if self.act_flag == 1 :
                    pres_txt = pres_txt + ' ' + str(self.pressure[ind].value.value[0]) # for controller 
                elif self.act_flag == 0 :
                    pres_txt = pres_txt + ' ' + str(self.pressure[ind].pressure.value) 
                    # print(self.pressure[ind].pressure.value)
                ind = ind + 1
            pres_txt = pres_txt + "]"
            ind = i0
            pres_txt = pres_txt + ",["
            for j in range(self.nb_cavity):
                pres_txt = pres_txt + ' ' + str(self.pressure[ind].cavityVolume.value)
                ind = ind + 1
            pres_txt = pres_txt + "]"

        self.fichier_csv.write(pres_txt +time_txt + '\n')
        # self.fichier_csv.write(str(pos) + time_txt +'\n')

        self.fichier_csv.close()
        print("%%%% Positions Enregistrées en Csv %%%%")
        self.fichier_csv = open(self.nf,'a')

class ParameterPrinterCsv(Sofa.Core.Controller):
    def __init__(self,module,nom_dossier,K_I,K_P,dt,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        path = os.getcwd()
        if not (os.path.exists(path + '/record/'+ nom_dossier)) :
            os.mkdir(path + '/record/'+ nom_dossier)

        connect.RecordParameters(module,'.csv',nom_dossier,K_P,K_I,dt)

## OLD FUNCTION / CONTROLLER - KEEP JUST IN CASE :

 ### - TXT PRINTER - ###
class PositionPrinterTxt(Sofa.Core.Controller): # utile ????
    def __init__(self,module,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,args,kwargs)
        self.node = args[0]
        self.stiffNode = self.node.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        self.pressure, txt_chmbre = connect.CavityConnect(RootNode=self.node,module=module)
        self.nf, self.fichier_txt = connect.OpenPrintFile(module,txt_chmbre,'.txt','pos_stiff_record_')
        self.start = time.time()

    def onKeypressedEvent(self,e):
        # print(self.nf)
        # ATTENTION INCOMPLET => REGARDER CSV
        pos = self.position.position.value[self.nb_poutre-1][0:3]
        ind = 0
        # pres = []
        pres_txt = ""
        for i in range(self.nb_module):
            pres_txt = pres_txt + " - ["
            for j in range(self.nb_cavity):
                pres_txt = pres_txt + ' ' + str(self.pressure[ind].value.value[0]) # for controller
                # pres_txt = pres_txt + ' ' + str(self.pressure[ind].pressure.value) # for qp
                ind = ind + 1
            pres_txt = pres_txt + " ]"

        self.fichier_txt.write(str(pos) + pres_txt +'\n')
        self.fichier_txt.close()
        print("%%%% Positions Enregistrées en Txt %%%%")
        self.fichier_txt = open(self.nf,'a')

class PrintBox(Sofa.Core.Controller) :
    def __init__(self,noeud,*args, **kwargs):


        Boite_III_K = noeud.getObject('boxROI_III_K1')
        print("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU \n \n \n")
        print(copy(Boite_III_K.pointsInROI.value))
        print(copy(Boite_III_K.quadInROI.value))

        print("\n \n \n UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU")