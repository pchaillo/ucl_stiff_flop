from os import getcwd, chdir, mkdir
from datetime import datetime
import csv
import time
import numpy as np

# Contient des fonctions partagés dans plusieurs fichier => Trajectory_Controller.py / flopMultiController.py / SimuController.py

        ## Pour sortir des datas de SOFA et les mettre dans un csv => en faire une fonction ???
        # nf = getcwd() + '/record/' + 'points.csv' # '.csv' or '.txt' # pour record les trucs et faire des tests en python
        # fichier = open(nf,'x')
        # fichier.write("si tu veux on fait comme \n")
        # fichier.write(str(copy(Boite_III_K.pointsInROI.value)))
        # fichier.close() # 

""" Fonction utilitaires """

def get_mean_point(all_positions,indices) : 
    """
    Donne la moyenne des positions des points contenus dans le tableau "all_positions", aux positions "indices"

    INPUT : 
    all_positions = positions de tous les points
    indices = indices des points du tableau précédent dont on veut faire la moyenne

    OUTPUT :
    [posx,posy,posz] = mean position on x, y and z axis
    """
    tabx = []
    taby = []
    tabz = []
    for i in range(len(indices)):
        tabx.append( all_positions.position.value[ indices[i] ][0] )
        taby.append( all_positions.position.value[ indices[i] ][1] )
        tabz.append( all_positions.position.value[ indices[i] ][2] )
    # print(tabx)
    posx = np.mean(tabx)
    posy = np.mean(taby)
    posz = np.mean(tabz)

    return [posx,posy,posz]

def kPa_to_bar(tab_in):
    """
    # Conversion  d'un tableau 1d de kPa en bar (titre relativement explicite)
    """
    tab_out = [] # pour appliquer la même transformation à tout le tableau python
    for i in range(len(tab_in)):
        tab_out.append(tab_in[i]/100) # on divise par 100 pcq 1 bar = 100 kPa
    # print(tab_out)
    return tab_out

""" Fonction connection à la simulation """

def CavityConnect(node,module): 
    """
    Variable pour connecter les cavités du modules au Controller SOFA

    INPUT : 
    node = noeud parent des noeuds avec les cavités
    module = variable stiff qui contient toutes les données du robot

    OUTPUT : 
    pressure = objet pression ("SPC") pour pouvoir modifier la valeur de pression
    """
    stiffNode = node # argument : parent node of the cavity
    # position = stiffNode.getObject('DOFs')
    ind = -1
    pressure = []
    txt_chmbre = ""
    print('-------')
    print('Noeuds connectés au PressurePrinter : ') # ajouter SIMU pour qu'o sache dans quel contexte c'est connecté ?
    for i in range(module.nb_module):
        txt_chmbre = txt_chmbre + ' , [Pression module n' + str(i+1) + '] , [Volume module n' + str(i+1) + ']'
        for j in range(module.nb_cavity):
            ind = ind + 1
            node_name = "Bellow" + str(j+1) + str(i+1)            
            noeud = stiffNode.getChild(node_name)
            print(node_name)
            pressure.append(noeud.getObject('SPC'))
    print('-------')
    print(' ')
    return pressure, txt_chmbre

def CavityConnect_2parents(node,node2,module): # Version avec les chambres des deux modules qui ont des parents différents (pas propres mais qui fonctionne)
    stiffNode = node # argument : parent node of the cavity
    # position = stiffNode.getObject('DOFs')
    ind = -1
    pressure = []
    txt_chmbre = ""
    print('-------')
    print('Noeuds connectés au PressurePrinter : ') # ajouter SIMU pour qu'o sache dans quel contexte c'est connecté ?
    for i in range(module.nb_module):
        if i == 0 :
            txt_chmbre = txt_chmbre + ' , [Pression module n' + str(i+1) + '] , [Volume module n' + str(i+1) + ']'
            for j in range(module.nb_cavity):
                ind = ind + 1
                node_name = "Bellow" + str(j+1) + str(i+1)            
                noeud = node2.getChild(node_name)
                print(node_name)
                pressure.append(noeud.getObject('SPC'))
        else :
            txt_chmbre = txt_chmbre + ' , [Pression module n' + str(i+1) + '] , [Volume module n' + str(i+1) + ']'
            for j in range(module.nb_cavity):
                ind = ind + 1
                node_name = "Bellow" + str(j+1) + str(i+1)            
                noeud = stiffNode.getChild(node_name)
                print(node_name)
                pressure.append(noeud.getObject('SPC'))
    print('-------')
    print(' ')
    return pressure, txt_chmbre

def OpenPrintFile(module,txt_chmbre,file_type,nom_fichier):
        path = getcwd()
        d_et_h = str(datetime.now())
        nf = path + '/record/' + nom_fichier + d_et_h[0:19] + file_type # '.csv' or '.txt'
        fichier = open(nf,'x')

        fichier.write('Caracteristiques des modules de la simulation : ')
        fichier.write('\n Hauteur du module (en mm) : , ' + str(module.h_module))
        fichier.write('\n Pression initiale : , ' + str(module.init_pressure_value))
        # self.fichier.write('Hauteur du module, en mm :', str(module.value_type))
        fichier.write('\n Module de Young des parties souples : , ' + str(module.YM_soft_part))
        fichier.write('\n Module de Young des parties rigides : , ' + str(module.YM_stiff_part))
        fichier.write('\n Coefficient de Poisson : , ' + str(module.coef_poi))
        fichier.write('\n Nombre de modules robotiques : ,'+ str(module.nb_module))
        fichier.write('\n Nombre de paires de cavites par module : , '+ str(module.nb_cavity))
        fichier.write('\n Nombre de poutres le long de l ensemble des modules : , '+ str(module.nb_poutre))
        fichier.write('\n Pression maximale : , '+ str(module.max_pression))
        fichier.write('\n Masse d un module (en kg) : , '+ str(module.masse_module))
        # if module.rigid_bool == 1 : # pertinent ?
        fichier.write('\n Application des parties rigides : , '+ str(module.rigid_bool))
        fichier.write('\n Modele 3D des modules : , '+ str(module.module_model))
        fichier.write('\n Modele 3D des chambres : , '+ str(module.chamber_model))
        fichier.write('\n')
        fichier.write('\n [Positions effecteur] , [Temps relatif] ' + txt_chmbre + '\n')

        return nf, fichier

def OpenPrintFile2(file_type,nom_fichier,nom_dossier):
        path = getcwd()
        # d_et_h = str(datetime.now())
        nf = path + '/record/'+ nom_dossier + '/' + nom_fichier + file_type # '.csv' or '.txt'
        fichier = open(nf,'x')

        return nf, fichier

def RecordParameters(module,file_type,nom_dossier,K_I,K_P,dt):
    """
    Pour enregistrer les paramètres de la simulation => permet de faire le lien entre les données et les paramètres, afin de pouvoir réutiliser les données plus tard (et ne pas les mélanger)

    """
    path = getcwd()
    # d_et_h = str(datetime.now())
    nom_fichier = 'Parameters'
    nf = path + '/record/'+ nom_dossier + '/' + nom_fichier + file_type # '.csv' or '.txt'
    fichier = open(nf,'x')

    fichier.write('Caracteristiques des modules de la simulation : ')
    fichier.write('\n Hauteur du module (en mm) : , ' + str(module.h_module))
    fichier.write('\n Pression initiale : , ' + str(module.init_pressure_value))
    # self.fichier.write('Hauteur du module, en mm :', str(module.value_type))
    fichier.write('\n Module de Young des parties souples : , ' + str(module.YM_soft_part))
    fichier.write('\n Module de Young des parties rigides : , ' + str(module.YM_stiff_part))
    fichier.write('\n Coefficient de Poisson : , ' + str(module.coef_poi))
    fichier.write('\n Nombre de modules robotiques : ,'+ str(module.nb_module))
    fichier.write('\n Nombre de paires de cavites par module : , '+ str(module.nb_cavity))
    fichier.write('\n Nombre de poutres le long de l ensemble des modules : , '+ str(module.nb_poutre))
    fichier.write('\n Pression maximale : , '+ str(module.max_pression))
    fichier.write('\n Masse d un module (en kg) : , '+ str(module.masse_module))
    # if module.rigid_bool == 1 : # pertinent ?
    fichier.write('\n Application des parties rigides : , '+ str(module.rigid_bool))
    fichier.write('\n Modele 3D des modules : , '+ str(module.module_model))
    fichier.write('\n Modele 3D des chambres : , '+ str(module.chamber_model))
    fichier.write('\n K_I : , '+ str(K_I))
    fichier.write('\n K_P: , '+ str(K_P))
    fichier.write('\n Pas de temps : , '+ str(dt))

    # fichier.write('\n')
    # fichier.write('\n [Positions effecteur] , [Temps relatif] ' + txt_chmbre + '\n')
    fichier.close()

        # return nf, fichier