import Sofa
# import SimuVal as Sim
from spicy import *
from os import getcwd, chdir, mkdir
from datetime import datetime
import csv
import time

try :
    import Connexion_Function_ucl as connect
except :
    import ucl_collaboration.Connexion_Function_ucl as connect

""" Controller for simulation """
class simu_controller(Sofa.Core.Controller):
# permet d'arrêter la simulation lorsqu'elle est finie
    def __init__(self,step,data_exp,*args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.RootNode = kwargs["RootNode"]
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.MaxIter = data_exp[1][step] # Number of dt steps before stopping simulation
        self.FinSimu = False # Variable to store if Simulation is done
        
    def onAnimateBeginEvent(self, dt): 
        # Compute error when simu is done            
        if (self.IterSimu >= self.MaxIter-1) :
            # self.compute_error()
            # with open(constants.OptimPath + "logs.txt","a") as logs:
            #     logs.write("\n")
            self.FinSimu = True 
        self.IterSimu += 1

class ProgressivePressure(Sofa.Core.Controller):
# fait atteindre porgressivement une pression déterminée en un nombre de pas determiné dans SimuVal 
    def __init__(self, nb_module,step,nb_cavity,*args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.RootNode = kwargs["RootNode"]
        self.MaxIter = Sim.nb_iter[step] # Number of dt steps before stopping simulation
        self.pression_d = Sim.pression_d
        self.pas_pres = self.pression_d / self.MaxIter
        self.stiffNode=self.RootNode.getChild('RigidFrames')
        ind = -1
        self.pressure = []
        print('-------')
        print('SIMU : Noeuds connectés au Controller : ')
        for i in range(nb_module):
            for j in range(nb_cavity):
                ind = ind + 1
                node_name = "Bellow" + str(j+1) + str(i+1)            
                self.noeud = self.stiffNode.getChild(node_name)
                # self.noeud.append(self.stiffNode.getChild(node_name)) # finalement pas de liste de noeuds
                print(node_name)
                self.pressure.append(self.noeud.getObject('SPC'))

        print('-------')
        print(' ')
        self.flag = 0
        self.nb_cavity = nb_cavity
        self.nb_module = nb_module
    
        
    def onAnimateBeginEvent(self, dt): 
        pressureValue = zeros(self.nb_cavity)
        index = self.flag*self.nb_cavity
        for i in range(self.nb_cavity):
            pressureValue[i] = self.pressure[index+i].value.value[0]

        pressureValue[0] += self.pas_pres
        print('         ****       ')
        print('SIMU : Control du mondule n° : ',self.flag+1)
        for i in range(self.nb_cavity): # remise valeurs au bon endroit
            self.pressure[index+i].value = [pressureValue[i]]
            print('SIMU : Pression chambre ',i,' : ',pressureValue[i])
        print('         ****       ')


class CsvPressureController(Sofa.Core.Controller):
    # récupère les pressions dans les csv puis on l'applique dans la chambre
    def __init__(self, module, step,data_exp,*args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        
        self.RootNode = kwargs["RootNode"]
        self.step = step
        
        # self.MaxIter = Sim.nb_iter[step] # Number of dt steps before stopping simulation
        self.IterSimu = 0 # Counter for dt steps before stopping simulation

        self.pressure , txt_chmbre = connect.CavityConnect(RootNode = self.RootNode,module = module)
        self.flag = 0
        self.nb_cavity = module.nb_cavity
        self.nb_module = module.nb_module
        self.ind_pres = data_exp[4]
        self.data_tab = data_exp[0]
    
        
    def onAnimateBeginEvent(self, dt): 
        pressureValue = zeros(self.nb_cavity)
        index = self.flag*self.nb_cavity
        for i in range(self.nb_cavity):
            pressureValue[i] = self.pressure[index+i].value.value[0]

        pressureValue[0] = self.data_tab[self.step][self.IterSimu][self.ind_pres]# pression a droite dans le
        # print(pressureValue)
        # print('         ****       ')
        # print('SIMU : Control du mondule n° : ',self.flag+1)
        for i in range(self.nb_cavity): # remise valeurs au bon endroit
            self.pressure[index+i].value = [pressureValue[i]*100]
            # print('SIMU : Pression chambre ',i,' : ',pressureValue[i])
        # print('         ****       ')
        self.IterSimu += 1

class PositionComparator_2d(Sofa.Core.Controller):
    # ne marche que pour le tableau contenant deux valeurs => écart et pression relative 
    def __init__(self,step,module,data_exp,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        self.step = step
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.ecart = 0 # ecart entre la simulation et la réalité, en mm
        self.ind_z = data_exp[3]
        self.data_tab = data_exp[0]


    def onAnimateBeginEvent(self, dt): 
         # je recupère l'extrémité de la dernière poutre comme position de référence pour l'effecteur, et seulement les 3ere valeurs car les 4 suivantes sont les quaternions caractérisant l'orientation
        pos_raw = self.position.position.value[self.nb_poutre-1][0:3]
        # pos = pos_raw - Sim.pos_i # on ne remet pas dans le referentiel de l'effecteur du robot
        pos = pos_raw 

        ## dirty code ##

        pos_z = pos[2] - 50 ;
        distance = sqrt(pos[0]*pos[0]+pos_z*pos_z)
        ##            ##
        
        # distance = sqrt(pos[0]*pos[0]+pos[2]*pos[2])
        distance_d = self.data_tab[self.step][self.IterSimu][self.ind_z] # distance désiré entre la position de repos et la position actuelle du robot, enregistrés dans les csv
        self.ecart = abs(float(distance_d)-distance)
        self.IterSimu += 1
        # print('Ecart entre les 2 : ' + str(self.ecart))

class InversePositionController(Sofa.Core.Controller):
    # récupère les pressions dans les csv puis on l'applique dans la chambre
    def __init__(self, nb_module,nb_cavity,step,*args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        # self.RootNode = args[0]
        self.RootNode = kwargs["RootNode"]
        self.step = step    
        self.MaxIter = Sim.nb_iter[step] # Number of dt steps before stopping simulation
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.flag = 0
        self.nb_cavity = nb_cavity
        self.nb_module = nb_module
        self.goalNode = self.RootNode.getChild('goal')
        self.position = self.goalNode.getObject('goalMO')
    
        
    def onAnimateBeginEvent(self, dt): 
        d = copy(self.position.position.value)

        # attention : inversion x et y # décalage entre la simu et l'acquisition
        d[0][0] = Sim.tab[self.step][self.IterSimu][1] # les trois positions sont à droite, quand la pression est dans la dernière case
        d[0][1] = Sim.tab[self.step][self.IterSimu][0] # les trois positions sont à droite, quand la pression est dans la dernière case
        d[0][2] = Sim.tab[self.step][self.IterSimu][2] # les trois positions sont à droite, quand la pression est dans la dernière case

        self.position.position = [d[0]]
        self.IterSimu += 1

class PressureComparator(Sofa.Core.Controller):
    def __init__(self,step,module,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        self.step = step
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.ecart = 0 # ecart entre la simulation et la réalité, en mm
        self.pressure , txt_chmbre = connect.CavityConnect(RootNode = self.RootNode,module = module)

    def onAnimateBeginEvent(self, dt): 

        # # fonctionne => a tester plus emplement puis à utiliser # a mettre dans Function_Connexion
        # ind = 0
        # # print('a')
        # for i in range(self.nb_module):
        #     # print('b')
        #     # i0 = ind # utile seulement quand on extrait plusieurs caractéristique sur une seule chambre => ex : volume et pression
        #     pres_tab = []
        #     for j in range(self.nb_cavity):
        #         # print('c')
        #         # print(self.pressure[ind].pressure.value)
        #         # print('cc')
        #         pres = copy(self.pressure[ind].pressure.value)
        #         # print(ind, pres)
        #         pres_tab[ind].append(pres)
        #         ind  = ind + 1
        #         # print('d')

        pres_tab = [copy(self.pressure[0].pressure.value),copy(self.pressure[1].pressure.value),copy(self.pressure[2].pressure.value)]
        print(pres_tab)

        pres = pres_tab[0]

        pres_csv = copy(Sim.tab[self.step][self.IterSimu][Sim.ind_pres]) # 4e valeur, les 3 premières c'est les position cartésienne

        self.ecart = [float(pres_csv) - float(pres)]
        # self.ecart = [pres_csv , pres_tab] # solution temporaire

        self.IterSimu += 1


class PositionComparator_3d(Sofa.Core.Controller):
    # ne marche que pour le tableau contenant deux valeurs => écart et pression relative 
    def __init__(self,step,module,data_exp,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        self.step = step
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.ecart = 0 # ecart entre la simulation et la réalité, en mm
        self.data_tab = data_exp[0]
        self.pos_ind = data_exp[5]


    def onAnimateBeginEvent(self, dt): 
         # je recupère l'extrémité de la dernière poutre comme position de référence pour l'effecteur, et seulement les 3ere valeurs car les 4 suivantes sont les quaternions caractérisant l'orientation
        pos_raw = self.position.position.value[self.nb_poutre-1][0:3]

        # print('a')

        pos_csv = [float(copy(self.data_tab[self.step][self.IterSimu][self.pos_ind[0]])), float(copy(self.data_tab[self.step][self.IterSimu][self.pos_ind[1]])), float(copy(self.data_tab[self.step][self.IterSimu][self.pos_ind[2]]))] 

        # print('b')
        # print(pos_raw)
        # print(pos_csv)
        # print('bb')

        # ###########" ATTENTION : INVERSION X ET Y !!!!!!!!!!!!"
        # self.ecart_x = float(pos_raw[0])-float(pos_csv[1])
        # self.ecart_y = float(pos_raw[1])-float(pos_csv[0])
        # self.ecart_z = float(pos_raw[2])-float(pos_csv[2])

        ###########" ATTENTION : INVERSION X ET Y !!!!!!!!!!!!"
        self.ecart_x = (float(pos_raw[0])-float(pos_csv[1]))**2
        self.ecart_y = (float(pos_raw[1])-float(pos_csv[0]))**2
        self.ecart_z = (float(pos_raw[2])-float(pos_csv[2]))**2

        self.ecart = sqrt(self.ecart_x+self.ecart_y+self.ecart_z)

        self.IterSimu += 1
        # print('Ecart entre les 2 : ' + str(self.ecart))

        # print('c')


class PressurePrinter(Sofa.Core.Controller):
        def __init__(self,step,module,*args, **kwargs):
            Sofa.Core.Controller.__init__(self,*args,**kwargs)
            self.RootNode = kwargs["RootNode"]
            self.step = step
            self.IterSimu = 0 # Counter for dt steps before stopping simulation
            self.ecart = 0 # ecart entre la simulation et la réalité, en mm
            self.pressure , txt_chmbre = connect.CavityConnect(RootNode = self.RootNode,module = module)
            txt_chmbre = "" # pour ne pas avoir d'entête fausse
            self.nf, self.fichier_csv = connect.OpenPrintFile(module,txt_chmbre,'.csv','pressure_record_')


        def onAnimateBeginEvent(self, dt): 
            pres_tab = [float(copy(self.pressure[0].pressure.value)),float(copy(self.pressure[1].pressure.value)),float(copy(self.pressure[2].pressure.value))]
            print(pres_tab)
            pres_csv = copy(Sim.tab[self.step][self.IterSimu][3])
            self.fichier_csv.write(str(pres_tab)+str(pres_csv) +'\n')
            self.fichier_csv.close()
            print("%%%% Pressions Enregistrées en Csv %%%%")
            self.fichier_csv = open(self.nf,'a')
            self.IterSimu += 1

class StaticPressure(Sofa.Core.Controller):
    # récupère les pressions dans les csv puis on l'applique dans la chambre
    def __init__(self, module,data_exp,*args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        
        self.RootNode = kwargs["RootNode"]
        
        # self.MaxIter = Sim.nb_iter[step] # Number of dt steps before stopping simulation
        self.IterSimu = 0 # Counter for dt steps before stopping simulation

        self.pressure , txt_chmbre = connect.CavityConnect(RootNode = self.RootNode,module = module)
        self.flag = 0
        self.nb_cavity = module.nb_cavity
        self.nb_module = module.nb_module    

        self.pressure_app = data_exp[2]
        
    def onAnimateBeginEvent(self, dt): 
        pressureValue = zeros(self.nb_cavity)
        index = self.flag*self.nb_cavity
        for i in range(self.nb_cavity):
            pressureValue[i] = self.pressure[index+i].value.value[0]

        pressureValue[0] = self.pressure_app[0] #  pression a droite dans le
        pressureValue[1] = self.pressure_app[1]
        pressureValue[2] = self.pressure_app[2]

        # print(pressureValue)
        # print('         ****       ')
        # print('SIMU : Control du mondule n° : ',self.flag+1)
        for i in range(self.nb_cavity): # remise valeurs au bon endroit
            self.pressure[index+i].value = [pressureValue[i]] # *100] pour le passage de bar en kPa si nécessaire
            # print('SIMU : Pression chambre ',i,' : ',pressureValue[i])
        # print('         ****       ')
        self.IterSimu += 1

class Static_3d_Comparator(Sofa.Core.Controller):
    # ne marche que pour le tableau contenant deux valeurs => écart et pression relative 
    def __init__(self,module,data_exp,*args, **kwargs):
        Sofa.Core.Controller.__init__(self,*args,**kwargs)
        self.RootNode = kwargs["RootNode"]
        self.stiffNode = self.RootNode.getChild('RigidFrames')
        self.position = self.stiffNode.getObject('DOFs')
        self.nb_poutre = module.nb_poutre
        self.nb_module = module.nb_module
        self.nb_cavity = module.nb_cavity
        self.IterSimu = 0 # Counter for dt steps before stopping simulation
        self.ecart = 0 # ecart entre la simulation et la réalité, en mm
        self.pos_d = data_exp[3]


    def onAnimateBeginEvent(self, dt): 
         # je recupère l'extrémité de la dernière poutre comme position de référence pour l'effecteur, et seulement les 3ere valeurs car les 4 suivantes sont les quaternions caractérisant l'orientation
        pos_raw = self.position.position.value[self.nb_poutre-1][0:3]

        pos_csv = self.pos_d

        ###########" ATTENTION : PAS PAS PAS d'INVERSION X ET Y !!!!!!!!!!!!"
        self.ecart_x = (float(pos_raw[0])-float(pos_csv[0]))**2
        self.ecart_y = (float(pos_raw[1])-float(pos_csv[1]))**2
        self.ecart_z = (float(pos_raw[2])-float(pos_csv[2]))**2

        # ###########" ATTENTION : INVERSION X ET Y !!!!!!!!!!!!"
        # self.ecart_x = (float(pos_raw[0])-float(pos_csv[1]))**2
        # self.ecart_y = (float(pos_raw[1])-float(pos_csv[0]))**2
        # self.ecart_z = (float(pos_raw[2])-float(pos_csv[2]))**2

        self.ecart = sqrt(self.ecart_x+self.ecart_y+self.ecart_z)

        self.IterSimu += 1
        # print('Ecart entre les 2 : ' + str(self.ecart))

        # print('c')
# # ### - CSV PRINTER - ###
# class SimuPrinterCsv(Sofa.Core.Controller): # pourquoi en avoir fait un autre ? # utiliser celui de flopMultiController plutôt
#     def __init__(self,module,*args, **kwargs):
#         Sofa.Core.Controller.__init__(self,args,kwargs)
#         self.node = args[0]
#         self.stiffNode = self.node.getChild('RigidFrames')
#         self.position = self.stiffNode.getObject('DOFs')
#         self.nb_poutre = module.nb_poutre
#         self.nb_module = module.nb_module
#         self.nb_cavity = module.nb_cavity

#         txt_chmbre = ""

#         ind = -1
#         self.pressure = []
#         print('-------')
#         print('SIMU : Noeuds connectés au Printer Csv : ')
#         for i in range(self.nb_module):
#             txt_chmbre = txt_chmbre + ' , [Pression module n' + str(i+1) + '] , [Volume module n' + str(i+1) + ']'
#             for j in range(self.nb_cavity):
#                 ind = ind + 1
#                 node_name = "Bellow" + str(j+1) + str(i+1)            
#                 self.noeud = self.stiffNode.getChild(node_name)
#                 # self.noeud.append(self.stiffNode.getChild(node_name)) # finalement pas de liste de noeuds
#                 print(node_name)
#                 self.pressure.append(self.noeud.getObject('SPC'))
#                 # self.pressure.append(self.noeud[ind].getObject('SPC'))
#         print('-------')
#         print(' ')

#         self.path = getcwd()
#         d_et_h = str(datetime.now())
#         self.nf = self.path+'/record/pos_stiff_record_' + d_et_h[0:19] + '.csv'
#         self.fichier_csv = open(self.nf,'a')

#         self.fichier_csv.write('Caracteristiques des modules de la simulation : ')
#         self.fichier_csv.write('\n Hauteur du module (en mm) : ,' + str(module.h_module))
#         self.fichier_csv.write('\n Pression initiale : ,' + str(module.init_pressure_value))
#         # self.fichier.write('Hauteur du module, en mm :', str(module.value_type))
#         self.fichier_csv.write('\n Module de Young des parties souples : ,' + str(module.YM_soft_part))
#         self.fichier_csv.write('\n Module de Young des parties rigides : ,' + str(module.YM_stiff_part))
#         self.fichier_csv.write('\n Coefficient de Poisson : ,' + str(module.coef_poi))
#         self.fichier_csv.write('\n Nombre de modules robotiques : ,'+ str(module.nb_module))
#         self.fichier_csv.write('\n Nombre de paires de cavites par module : ,'+ str(module.nb_cavity))
#         self.fichier_csv.write('\n Nombre de poutres le long de l ensemble des modules : ,'+ str(module.nb_poutre))
#         self.fichier_csv.write('\n Pression maximale : ,'+ str(module.max_pression))
#         self.fichier_csv.write('\n Masse d un module (en kg) : ,'+ str(module.masse_module))
#         if module.rigid_bool == 1 :
#             self.fichier_csv.write('\n Application des parties rigides : ,'+ str(module.rigid_bool))
#         self.fichier_csv.write('\n Modele 3D des modules : ,'+ str(module.module_model))
#         self.fichier_csv.write('\n Modele 3D des chambres : ,'+ str(module.chamber_model))
#         # self.fichier_csv.write('\n')
#         self.fichier_csv.write('\n [Positions effecteur] , [Temps relatif] ' + txt_chmbre + '\n')

#         self.start = time.time()

#     def onAnimateBeginEvent(self, dt): 
#         # print(self.nf)
#         pos = self.position.position.value[self.nb_poutre-1][0:3]
#         ind = 0
#         # pres = []
#         # print(str(time.time() - self.start)) 
#         time_txt = ", [" + str(time.time() - self.start) + "]"
#         pres_txt = ""
#         for i in range(self.nb_module):
#             pres_txt = pres_txt + ",["
#             i0 = ind
#             for j in range(self.nb_cavity):
#                 pres_txt = pres_txt + ' ' + str(self.pressure[ind].value.value[0]) # attention, ne marche pas pour qp (SPActuator), voir PressureComparator
#                 ind = ind + 1
#             pres_txt = pres_txt + "]"
#             ind = i0
#             pres_txt = pres_txt + ",["
#             for j in range(self.nb_cavity):
#                 pres_txt = pres_txt + ' ' + str(self.pressure[ind].cavityVolume.value)
#                 ind = ind + 1
#             pres_txt = pres_txt + "]"

#         self.fichier_csv.write(str(pos) + time_txt + pres_txt +'\n')

#         # print('testpass')

#         # if e["key"] == Key.Q :
#         # if self.end == True:
#         self.fichier_csv.close()
#         print("%%%% Positions Enregistrées en Csv %%%%")

#         self.fichier_csv = open(self.nf,'a')

#         # print("testagain")
