 # coding=utf-8

import Sofa
# import SofaPython3
from math import sin,cos, sqrt, acos
import array

from Stiff_Function import *
from flopMultiController import *
from SimuController_ucl import *
from TrajectoryController import *
from CloseLoopController import *
# from polhemus_liberty.python.Polhemus_SOFA_Controller import *

# try : ## pour le fonctionnement classique # ATTENTION AUX ERREURS 
#     from Stiff_Function2 import Stiff_Flop1
#     from flopMultiController import *
#     from SimuController_ucl import *
#     from TrajectoryController import *
#     from CloseLoopController import *
#     from polhemus_liberty.python.Polhemus_SOFA_Controller import *
#     print("************** CLASSIC ****************")
# except : # pour pouvoir lancer le script depuis un fichier externe
#     print("*************** EXCEPTION **************")
#     from ucl_collaboration.Stiff_Function import *
#     from ucl_collaboration.flopMultiController import *
#     from ucl_collaboration.SimuController_ucl import *

from stlib3.physics.mixedmaterial import Rigidify
from stlib3.physics.deformable import ElasticMaterialObject
#from tutorial import *
import time

############## PARAM7TRES A FIXER ####################

act_flag = 1 # set 0 for IP and 1 for direct control
version = 2 # v1 d=14mm // v2 d=11.5mm
record = 0 # 0 => no record // 1 => record

close_loop = 1 # 0 => no close loop
if close_loop == 0 :
    K_P = 0
    K_I = 0
    shift = 5 # shift in mm between the goal and the goal2 (for grabbing) points
else :    
    K_P = 0 #0.1
    # K_I = 0.0001
    K_I = 0.11

dt = 0.001

# Paramètres controller
pas = 20 # pour argument controller (attention aux unités !)
max_pression = 150 # en kPa
min_pression = 0
init_pressure_value = 0
# value_type = "1" # pour commande en volume (avec beam6)
value_type = "pressure" # pour commande en pression (avec beam5)
# value_type = "volumeGrowth" # pour commande en pression (avec beam5)


# Paramètres ROBOT
nb_module = 1 # nombre de modules
# module
masse_module = 0.01 # en kg, soit 10g
# soft part
coef_poisson = 0.45 # coefficient de poisson
# stiff parts
rigid_bool = 0 # 0 => no rigid parts (pas de partie rigide) // 1 => rigids parts  
YM_stiff_part = 1875 # young modulus of the stiff part
rigid_base = 4 # hauteur de rigidification de la base des modules en mm
rigid_top = 2 # hauteur de rigidification de l'extrémité des modules en mm


### Goal contoller
goal_pas = 5

if version == 1 : # V1

    h_module = 58 # hauteur du module en mm # Mieux parce que prend en compte la taille de l'effecteur ?
    chamber_model =  'stiff_flop_58_chamber_normal.stl'
    module_model = 'stiff_flop_58.vtk'
    YM_soft = 30 #30 # young modulus of the soft part (kPa)


    # h_module = 56 # hauteur du module en mm
    # chamber_model =  'model_chambres_55_simp.stl'
    # module_model = 'model_module_55.vtk'
    # YM_soft = 30 # young modulus of the soft part (kPa)

 
    radius = 7
    nb_cavity = 3  # nombre de cavités (ou paires de cavités)

elif version == 2 : # V2 module
    h_module = 50 # hauteur du module en mm
    # chamber_model =  'model_chambres_v2_reg.stl'
    chamber_model =  'model_chambre_regulier_cutted.stl'
    # module_model = 'coeur_module01.vtk'
    # module_model = 'stiff_flop_indicesOK_flip05.vtk'
    module_model = 'stiff_flop_indicesOK_flip.obj'
    # module_model = 'model_module_v2_90.vtk'
    radius = 5.75 
    YM_soft = 15 # young modulus of the soft part (kPa)
    nb_cavity = 3  # nombre de cavités (ou paires de cavités)

elif version == 3 : # V3 module
    h_module = 60 # hauteur du module en mm
    chamber_model =  'model_chambres_generic_60_simp.stl'
    # chamber_model =  'model_chambres_v2_90_test.stl'
    module_model = 'model_module_v3_canule_ext_60.vtk'
    radius = 5.75 
    YM_soft = 15 # young modulus of the soft part (kPa)
    nb_cavity = 3  # nombre de cavités (ou paires de cavités)

elif version == 4 : # V4 module
    h_module = 45 # hauteur du module en mm
    chamber_model =  'model_chambres_v4_simp.stl'
    # chamber_model =  'model_chambres_v2_90_test.stl'
    module_model = 'model_module_v4.vtk'
    radius = 5.75 
    YM_soft = 15 # young modulus of the soft part (kPa)
    nb_cavity = 4  # nombre de cavités (ou paires de cavités)

# Paramètres Simulation
name_module = 'Module'
name_cavity = 'Bellow'
nb_poutre = nb_module*4
h_effector = h_module * nb_module

d_et_h = str(datetime.now())
nom_dossier = d_et_h[0:19]

position = [0,0,h_effector]

## Trajectory parameters :
circle_radius = 25
nb_iter_circle = 600 # 600 eq to 1min/tour approximately
circle_height = h_effector

nb_iter_square = 150 # 600 eq 10min/tour /// 
square_height = circle_height
square_radius = 20

point_tab = [ [10,10,60],[10,5,60],[10,0,60],[10,-5,60],[10,-10,60],[5,-10,60],[5,-5,60],[5,0,60]] # once the robot will have reach all the positions, he will start again with the 1st position

############## PARAM7TRES -- FIN ####################

# stiff = Stiff_Flop(h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model,nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool)

def MyScene(rootNode, out_flag,step,YM_soft_part,coef_poi,act_flag,data_exp):

    print("0000000000000000000000000000")
    print("0000000000000000000000000000")
    print("0000000000000000000000000000")
    print("0000000000000000000000000000")

    stiff = Stiff_Flop(h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model,nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool,min_pression)
    
    rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/SoftRobots/lib/') #libSoftRobots.so 1.0
    rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/ModelOrderReduction/lib/') #libSoftRobots.so 1.0
    rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/BeamAdapter/lib')#/libBeamAdapter.so 1.0

    # required plugins:
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    rootNode.addObject('RequiredPlugin', name='SofaConstraint')
    rootNode.addObject('RequiredPlugin', name='SofaDeformable')
    rootNode.addObject('RequiredPlugin', name='SofaGeneralAnimationLoop')
    rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
    rootNode.addObject('RequiredPlugin', name='SofaLoader')
    rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaSimpleFem')
    rootNode.addObject('RequiredPlugin', name='SofaSparseSolver')
    rootNode.addObject('RequiredPlugin', name='SofaEngine')
    rootNode.addObject('RequiredPlugin', name='SofaGeneralLoader')
    rootNode.addObject('RequiredPlugin', name='SofaGeneralEngine')
    #rootNode.addObject('RequiredPlugin', name='SofaPython')

    # rootNode.findData('gravity').value=[0, 0, 9810];
    rootNode.findData('gravity').value=[0, 0, 0];

        #visual dispaly
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
    #rootNode.createObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    
    rootNode.findData('dt').value= dt;
    
    rootNode.addObject('FreeMotionAnimationLoop')
    rootNode.addObject('DefaultVisualManagerLoop')   

    rigidFramesNode  = rootNode.addChild('RigidFrames')
    rigidFramesNode.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)
    rigidFramesNode.addObject('SparseLDLSolver', name='ldlsolveur',template="CompressedRowSparseMatrixd")    
    # rigidFramesNode.addObject('SofaDenseSolver', name='ldlsolveur')  # test  
    rigidFramesNode.addObject('GenericConstraintCorrection')

    rigidFramesNode.addObject('RegularGridTopology',  name='meshLinesCombined',  nx=nb_poutre, ny='1', nz='1', xmax=h_module*nb_module, xmin='0.0', ymin='0', ymax='0',zmin='0',zmax='0')
    rigidFramesNode.addObject('MechanicalObject',  name='DOFs', template='Rigid3d', showObject='1', showObjectScale='1', rotation=[0, -90 ,0], translation = [0,0,0])
    rigidFramesNode.addObject('BeamInterpolation', name='BeamInterpolation', printLog = '1', defaultYoungModulus=YM_soft_part, dofsAndBeamsAligned='true', straight='1', crossSectionShape='circular', radius=radius)#, radius=5*radius)
    # rigidFramesNode.addObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', computeMass='0', massDensity=0.001)
    rigidFramesNode.addObject('RestShapeSpringsForceField', name='anchor', points='0', stiffness='1e12', angularStiffness='1e12')
      

    stiff_flop = stiff.createRobot(parent = rigidFramesNode, name = "MyStiffFlop",out_flag = out_flag, act_flag = act_flag)

    # MeasuredPosition = EffectorGoal(node=rootNode, position = [0,0,0],name = 'MeasuredPosition',taille = 4)
    # DesiredPosition = EffectorGoal(node=rootNode, position = [0,0,h_effector],name = 'DesiredPosition',taille = 0.5)

    if act_flag == 0 :
        rootNode.addObject('QPInverseProblemSolver', name="QP", printLog='0', saveMatrices = True ,epsilon = 0.01) # initialement epsilon = 0.001

        goal = EffectorGoal(node=rootNode, position = [0,0,h_effector],name = 'goal',taille = 0.5)

        # goal = rootNode.addChild('Goal')
        # goal.addObject('EulerImplicitSolver', firstOrder=True)
        # goal.addObject('CGLinearSolver', iterations=100, threshold=1e-12, tolerance=1e-10)
        # goal.addObject('MechanicalObject', name='goalMO', template='Rigid3', position=position+[0., 0., 0., 1.], showObject=True, showObjectScale=10)
        # # goal.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1e5, stiffness=1e5)
        # goal.addObject('UncoupledConstraintCorrection')

         ## 2eme goal poitn pour décalage, permet de mieux attraper à la souris
        if close_loop == 0 :
            goal2 = EffectorGoal(node=rootNode, position = [0,0,h_effector+shift+3],name = 'goal2',taille = 0.5)


        ## CLASSIC 
        controlledPoints = stiff_flop.addChild('controlledPoints')
        controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Vec3",position=[h_effector, 0, 0])#,rotation=[0, 90 ,0]) # classic
        controlledPoints.addObject('PositionEffector', template="Vec3d", indices='0', effectorGoal="@../../../goal/goalM0.position") # classic
        controlledPoints.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

        # ## TESTS
        # controlledPoints = rigidFramesNode.addChild('controlledPoints')
        # controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Rigid3",position=[0, 0, h_effector]+[0., 0., 0., 1.])#,rotation=[0, 90 ,0]) # rigid pour l'orientation
        # controlledPoints.addObject('PositionEffector', template="Rigid3", indices= 0, effectorGoal="@../../../goal/goalM0.position",useDirections=[1, 1, 1, 1, 1, 1]) # rigid pour l'orientation
        # # controlledPoints.addObject('PositionEffector', template="Vec3d", indices='0', effectorGoal="@../../../goal/goalMO.position",maxShiftToTarget=-2.0,limitShiftToTarget = True)
        # controlledPoints.addObject('BarycentricMapping', mapForces=False, mapMasses=False)
        # # controlledPoints.addObject("RigidMapping", index=0) #test

    elif act_flag == 1:
        rootNode.addObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')

    if out_flag == 1: # CONTROLLER SIMU AUTOMATIQUE
        if data_exp[0] == "lonely" : # Dans le cas ou on ne teste qu'une valeur
            cam_pos_tab = data_exp[4]
            rootNode.addObject("InteractiveCamera", name="camera", position=cam_pos_tab)#, zNear=0.1, zFar=500, computeZClip = False,  projectionType=0)
            rootNode.addObject(simu_controller(name="simu_controller",step = 0,RootNode=rootNode,data_exp=data_exp))#, Step=step))
            rootNode.addObject(StaticPressure(name="simu_static_pressure",module =stiff,RootNode=rootNode,data_exp=data_exp))#, Step=step))
            rootNode.addObject(Static_3d_Comparator(name="simu_pos_compar",module = stiff,RootNode = rootNode,data_exp = data_exp))#, Step=step)) # ne fonctionne pas pour inverse
        else :
            # print("fete à la saucisse j'invite personne")
            cam_pos_tab = data_exp[7]
            rootNode.addObject("InteractiveCamera", name="camera", position=cam_pos_tab)#, zNear=0.1, zFar=500, computeZClip = False,  projectionType=0)
            rootNode.addObject(simu_controller(name="simu_controller",step = step,RootNode=rootNode,data_exp=data_exp))#, Step=step))
            # rootNode.addObject(SimuPrinterCsv(stiff,rootNode))
            rootNode.addObject(PositionComparator_2d(name="simu_pos_compar", step=step,module = stiff,RootNode = rootNode,data_exp = data_exp))#, Step=step)) # ne fonctionne pas pour inverse
            # rootNode.addObject(PressureComparator(name="simu_pres_compar", step=step,module = stiff,RootNode = rootNode))#, Step=step))
            # rootNode.addObject(PositionComparator_3d(name="simu_pos_compar", step=step,module = stiff,RootNode = rootNode,data_exp=data_exp))#, Step=step)) # ne fonctionne pas pour inverse
            # rootNode.addObject(PressurePrinter(name="printer", step=step,module = stiff,RootNode = rootNode))#, Step=step)) # ne fonctionne pas pour inverse
            if act_flag == 0 :
                rootNode.addObject(InversePositionController(name="inverse_csv_controller", nb_module = nb_module,nb_cavity = nb_cavity,step=step,RootNode=rootNode,data_exp=data_exp))
            elif act_flag == 1 :
                # rootNode.addObject(CsvPressureController(name="simu_csv_controller", nb_module = nb_module,nb_cavity = nb_cavity,step=step,RootNode=rootNode))#, Step=step))
                rootNode.addObject(CsvPressureController(name="simu_csv_controller",module =stiff,step=step,RootNode=rootNode,data_exp=data_exp))#, Step=step))
                # rootNode.addObject(ProgressivePressure(name="simu_pos_controller", nb_module = nb_module,nb_cavity = nb_cavity,step=step,RootNode=rootNode))#, Step=step))


    else : # controller runSofa
        if record == 1:
            rootNode.addObject(ParameterPrinterCsv(module =stiff,nom_dossier = nom_dossier,RootNode=rootNode,K_I = K_I, K_P = K_P,dt=dt))
            rootNode.addObject(PositionPrinterCsv(child_name = 'RigidFrames',name = 'DOFs',module =stiff,nom_dossier = nom_dossier,beam_flag = 1,RootNode=rootNode))
            rootNode.addObject(PositionPrinterCsv(child_name = 'goal',name = 'goalM0',module =stiff,nom_dossier = nom_dossier,beam_flag = 0,RootNode=rootNode))
            rootNode.addObject(PressurePrinterCsv(module =stiff,nom_dossier = nom_dossier,RootNode=rootNode,act_flag=act_flag))
            rootNode.addObject(PositionPrinterCsv(child_name = 'MeasuredPosition',name = 'MeasuredPositionM0',module =stiff,nom_dossier = nom_dossier,beam_flag = 0,RootNode=rootNode))
            if close_loop == 1 :
                rootNode.addObject(PositionPrinterCsv(child_name = 'DesiredPosition',name = 'DesiredPositionM0',module =stiff,nom_dossier = nom_dossier,beam_flag = 0,RootNode=rootNode))
            else :
                rootNode.addObject(PositionPrinterCsv(child_name = 'goal2',name = 'goal2M0',module =stiff,nom_dossier = nom_dossier,beam_flag = 0,RootNode=rootNode))

        # rootNode.addObject(ArduinoPressure(module = stiff,RootNode = rootNode)) # pour envoyer les pressions calculées par le modèle inverse au robot (hardware) # (mettre après le if suivant !)
        # rootNode.addObject(ArduinoPressure_UCL(module = stiff,RootNode = rootNode)) # pour envoyer les pressions calculées par le modèle inverse au robot (hardware)
        # rootNode.addObject(PressurePrinter_local(module = stiff,RootNode = rootNode)) 
        
        if act_flag == 0 :
            if close_loop == 1 :
                # test = 2
                # rootNode.addObject(GoalKeyboardController(goal_pas = goal_pas,child_name = 'DesiredPosition',name = 'DesiredPositionM0', RootNode = rootNode)) # for goal without shift
                # rootNode.addObject(CloseLoopController(name="CloseLoopController",RootNode=rootNode, K_P = K_P, K_I = K_I))
                rootNode.addObject(CloseLoopController2(name="CloseLoopController",RootNode=rootNode, K_P = K_P, K_I = K_I))


                rootNode.addObject(PolhemusTracking(node = MeasuredPosition,name = 'MeasuredPositionM0',offset = [0,0,h_effector]) )

                # rootNode.addObject(CircleTrajectory(rayon =circle_radius, nb_iter = nb_iter_circle,node = DesiredPosition,name = 'DesiredPositionM0',circle_height = circle_height,module=stiff))
                rootNode.addObject(PointPerPointTrajectory(node = DesiredPosition,name = 'DesiredPositionM0',module = stiff,point_tab = point_tab, node_pos = MeasuredPosition, name_pos = 'MeasuredPositionM0',err_d = 0.2,shift=0,beam_flag = 0))

                # rootNode.addObject(SquareTrajectory(rayon =square_radius, nb_iter = nb_iter_square,node = DesiredPosition,name = 'DesiredPositionM0',square_height = square_height,module=stiff))
                # rootNode.addObject(SquareTrajectory(RootNode = rootNode, rayon =square_radius, nb_iter = nb_iter_circle,child_name = 'DesiredPosition',name = 'DesiredPositionM0',square_height = square_height,module=stiff))
                
                # rootNode.addObject(PrintGoalPos(name="CloseLoopController",RootNode=rootNode))
                # rootNode.addObject(PatternTrajectory(RootNode = rootNode, rayon =square_radius, nb_iter = nb_iter_circle,child_name = 'DesiredPosition',name = 'DesiredPositionM0',square_height = square_height,module=stiff))
            else : # open loop ( close_loop == 0 )

                # rootNode.addObject(LineTrajectory(nb_iter=20,node = goal2,name = 'goal2M0',p_begin = [-10 , 10 , 58+shift], p_end = [10, -10 , 58 +shift]))
                # rootNode.addObject(PointPerPointTrajectory(node = goal2,name = 'goal2M0',module = stiff,point_tab = point_tab, node_pos = rigidFramesNode, name_pos = 'DOFs',err_d = 0.1,shift=shift,beam_flag = 1))
                rootNode.addObject(PointPerPointTrajectory(node = goal,name = 'goalM0',module = stiff,point_tab = point_tab, node_pos = rigidFramesNode, name_pos = 'DOFs',err_d = 0.7,shift=0,beam_flag = 1))

                # rootNode.addObject(CircleTrajectory(rayon =circle_radius, nb_iter = nb_iter_circle, node = goal2,name = 'goal2M0',circle_height = circle_height+5,module=stiff))

                # rootNode.addObject(SquareTrajectory(rayon =square_radius, nb_iter = nb_iter_circle,node = goal2,name = 'goal2M0',square_height = square_height+5,module=stiff))

                # rootNode.addObject(PolhemusTracking(node = MeasuredPosition,name = 'MeasuredPositionM0',offset = [0,0,h_effector]) )

                rootNode.addObject(GoalKeyboardController(goal_pas = goal_pas,node = goal2,name = 'goal2M0')) # for goal with shift
                # rootNode.addObject(GoalShift(node_follow= goal ,object_follow = 'goalM0',node_master = goal2,object_master = 'goal2M0',shift_tab = [0,0,shift]))

        elif act_flag == 1 :
            rootNode.addObject(StiffController(pas=pas,module = stiff,RootNode = rootNode))

            # rootNode.addObject(AuroraTracking(child_name = 'MeasuredPosition',name = 'MeasuredPositionM0',module =stiff,RootNode=rootNode))
            # rootNode.addObject(PolhemusTracking(node = MeasuredPosition,name = 'MeasuredPositionM0',offset = [0,0,h_effector]) )

            # rootNode.addObject(ArduinoPressure(module = stiff,RootNode = rootNode)) # pour envoyer les pressions calculées par le modèle inverse au robot (hardware)


    return rootNode

def createScene(rootNode):
    MyScene(rootNode,out_flag = 0,step = 0,YM_soft_part=YM_soft,coef_poi = coef_poisson,data_exp = 0,act_flag = act_flag) # act_flag = 0- IP ; 1- Direct Control


    
    
   

    
   