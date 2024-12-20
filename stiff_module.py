"""
Auteur : Paul Chaillou
Contact : paul.chaillou@inria.fr
Année : 2022
Propriétaire : Université de Lille - CNRS 
License : Non définie, mais développé dans une démarche Open-Source et Logiciel Libre avec volonté de partage et de travail collaboratif. Développé dans un but non-marchand, en cas d'utilisation commerciale, merci de minimiser les prix et de favoriser le partage gratuit de tout ce qui peut l'être. A utiliser dans des buts prenant en compte les questions éthiques et morales (si possible non-militaire, ne rentrant pas dans le cadre de compétition, de monopole, ou de favorisation d'interets privés).
"""

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
from UCL_Controller import *
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

#from stlib3.physics.mixedmaterial import Rigidify
#from stlib3.physics.deformable import ElasticMaterialObject
#from tutorial import *
import time


############## PARAM7TRES A FIXER ####################
## FLAG ##
act_flag = 0 # set 0 for IP (Inverse Problem resolution with QP) and 1 for direct control
version = 2 # v1 d=14mm // v2 d=11.5mm // v3 d = 10mm // v4 d = 8mm but with 4 cavities
record = 0 # 0 => no record // 1 => record
setup = 0 # 0 => no hardware connected // 1 => UCL JILAEI SETUP // 2 => INRIA DEFROST SETUP
force_field = 1 # 0 => Tetrahedron FEM force fiels // 1 => Hexahedron FEM force field (Be crafull => you have to be coherent with the mesh files)
auto_stl = 1 # 0 = > no automatic stl completion for chamber // 1 => with automatic settings
dynamic = 1 # 0 => static (first order) // 1 => dynamic

close_loop = 0 # 0 => no close loop
if close_loop == 0 :
    K_P = 0
    K_I = 0
    shift = 0 #5 # shift in mm between the goal and the goal2 (for grabbing) points
else :    
    K_P = 0.001 #0.1
    # K_I = 0.0001
    K_I = 0.02

dt = 0.1

## Actuation Parameters ## 
pas = 10 # Pressure step for direct control (kPa) // pour argument controller (attention aux unités !) (*dt pour dyn)
max_pression = 200 # Maximal pressure in kPa
min_pression = 0
init_pressure_value = 0
# value_type = "1" # pour commande en volume (avec beam6)
value_type = "pressure" # pour commande en pression (avec beam5)
# value_type = "volumeGrowth" # pour commande en pression (avec beam5)
if dynamic == 1 :
    pas = pas*dt
    max_pression = max_pression*dt # Maximal pressure in kPa
    min_pression = min_pression*dt
    init_pressure_value = init_pressure_value*dt

## Robot Parameters ##
nb_module = 1 # nombre de modules
# module
masse_module = 0.01 # in kg, equivalent to 10g
# soft part
coef_poisson = 0.15 # 0.4 # coefficient de poisson
# stiff parts
rigid_bool = 0 # 0 => no rigid parts (pas de partie rigide) // 1 => rigids parts  
YM_stiff_part = 1875 # young modulus of the stiff part
rigid_base = 4 # hauteur de rigidification de la base des modules en mm
rigid_top = 2 # hauteur de rigidification de l'extrémité des modules en mm
r_disk_chamber = 4 + 1 # +1 for the box # radius of the disk, were are put the cavities => the distance between the center of the module and the center of the cavity
r_cavity = 0.75 # radius of the cavity

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
    if auto_stl == 0:
        # chamber_model =  'chambres_55_4it.stl'  # 55 mm
        chamber_model =  'model_chambres_v2_reg.stl' ### 
        # chamber_model =  'chambres_55cutted.stl'
        # chamber_model =  'model_chambre_regulier_cutted.stl'
    elif auto_stl == 1 :
        # stl_base = "cutted_chamber_v2_" # BE CAREFULL -  IT WILL ONLY WORK FOR SIZE BETWEEN 50 and 55mm (for the moment)
        stl_base = "bullshit" # RENDRE CA CLAIR ET PROPRE
        chamber_model = stl_base + str(h_module) + '.stl'
        # chamber_model = auto_stl_choice(h_module,stl_base)
    # module_model = 'coeur_module01.vtk'
    # module_model = 'stiff_flop_indicesOK_flip05.vtk'
    module_model = 'stiff_flop_indicesOK_flip.obj'
    # module_model = 'model_module_v2_90.vtk' # h_module = 50 du coup
    radius = 5.75 
    YM_soft = 90# 22.5 # young modulus of the soft part (kPa)
    nb_cavity = 3  # nombre de cavités (ou paires de cavités)

elif version == 3 : # V3 module
    h_module = 60 # hauteur du module en mm
    chamber_model =  'model_chambres_generic_60_simp.stl'
    # chamber_model =  'model_chambres_v2_90_test.stl'  ###  
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

## Simulation Parameters ##
name_module = 'Module' # pas utilisé il me semble
name_cavity = 'Bellow'
#nb_poutre = nb_module*17 # (best 7 beam with 20 slices)
nb_slices = 16
nb_poutre_per_module = nb_slices 
# nb_poutre_per_module = 11 
nb_poutre = nb_module*nb_poutre_per_module + 1
h_effector = h_module * nb_module
goal_pas = 5 # step in mm for displacement of goal point with the keyboard


## Trajectory parameters ## 
# CIRCLE
circle_radius = 20
nb_iter_circle = 600 # 600 eq to 1min/tour approximately
circle_height = h_effector
# SQUARE
nb_iter_square = 200# 600 eq 10min/tour /// 
square_height = circle_height
square_radius = 15
# POINT PER POINT TRAJECTORY
# point_tab = [ [5,5,55], [10,10,55],[10,5,55],[10,0,55],[10,-5,55],[10,-10,55],[5,-10,55],[5,-5,55],[5,0,55], [0,0,55],[0,0,60], [-5,5,60], [-10,10,60],[-10,5,60],[-10,0,60],[-10,-5,60],[-10,-10,60],[-5,-10,60],[-5,-5,60],[-5,0,60], [0,0,60], [0,0,55]] # once the robot will have reach all the positions, he will start again with the 1st position
# point_tab = [ [105,5,5], [105,10,10],[110,5,10],[110,0,10],[110,-5,10],[110,-10,10],[110,-10,5],[110,-5,5],[110,0,5], [110,0,0]]#,[0,0,60], [-5,5,60], [-10,10,60],[-10,5,60],[-10,0,60],[-10,-5,60],[-10,-10,60],[-5,-10,60],[-5,-5,60],[-5,0,60], [0,0,60], [0,0,55]] # once the robot will have reach all the positions, he will start again with the 1st position
point_tab = [ [ 77, 26, -44],[44 ,70 ,-31],[51,73,3]]

d_et_h = str(datetime.now())
nom_dossier = d_et_h[0:19]

position = [0,0,h_effector]
############## PARAM7TRES -- FIN ###################

# stiff = Stiff_Flop(h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model,nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool)

def MyScene(rootNode, out_flag,step,YM_soft_part,coef_poi,act_flag,data_exp):

    stiff = Stiff_Flop(h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model,nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool,min_pression,force_field,dynamic,dt,nb_slices,r_disk_chamber,r_cavity)
    
    # rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/SoftRobots/lib/') #libSoftRobots.so 1.0
    # rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/ModelOrderReduction/lib/') #libSoftRobots.so 1.0
    # rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/BeamAdapter/lib')#/libBeamAdapter.so 1.0

    pluginNode  = rootNode.addChild('pluginNode')
    pluginNode.addObject('RequiredPlugin', name='SoftRobots.Inverse') # Where is SofaValidation ? => Deprecated Error in terminal
    pluginNode.addObject('RequiredPlugin', name='SoftRobots')
    pluginNode.addObject('RequiredPlugin', name='BeamAdapter')
    pluginNode.addObject('RequiredPlugin', name='SOFA.Component.IO.Mesh')
    pluginNode.addObject('RequiredPlugin', name='SOFA.Component.Engine.Generate')
    pluginNode.addObject('RequiredPlugin', name='SOFA.Component.Mass')
    pluginNode.addObject('RequiredPlugin', name='SOFA.Component.LinearSolver.Direct')
    pluginNode.addObject('RequiredPlugin', name='SOFA.Component.Constraint.Lagrangian.Correction')  
    pluginNode.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D') 
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Diffusion')
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop') # Needed to use components [FreeMotionAnimationLoop]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry') # Needed to use components [SphereCollisionModel]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Correction') # Needed to use components [GenericConstraintCorrection,UncoupledConstraintCorrection]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Lagrangian.Solver') # Needed to use components [GenericConstraintSolver]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Generate') # Needed to use components [ExtrudeQuadsAndGenerateHexas]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select') # Needed to use components [BoxROI]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh') # Needed to use components [MeshOBJLoader]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Direct') # Needed to use components [SparseLDLSolver]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative') # Needed to use components [CGLinearSolver]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [UniformMass]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward') # Needed to use components [EulerImplicitSolver]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Setting') # Needed to use components [BackgroundSetting]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic') # Needed to use components [HexahedronFEMForceField]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.Spring') # Needed to use components [RestShapeSpringsForceField]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic') # Needed to use components [HexahedronSetTopologyContainer,TriangleSetTopologyContainer]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Grid') # Needed to use components [RegularGridTopology]  
    pluginNode.addObject('RequiredPlugin', name='Sofa.Component.Visual') # Needed to use components [VisualStyle]  


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
    if dynamic == 1 :
        rigidFramesNode.addObject('EulerImplicitSolver', firstOrder='0', vdamping=0,rayleighStiffness='0.3',rayleighMass='0.1')
    else :
        rigidFramesNode.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)

    rigidFramesNode.addObject('SparseLDLSolver', name='ldlsolveur',template="CompressedRowSparseMatrixd")    # CompressedRowSparseMatrixMat3x3d (plus rapide pour les Vec3d)
    # rigidFramesNode.addObject('SofaDenseSolver', name='ldlsolveur')  # test  
    rigidFramesNode.addObject('GenericConstraintCorrection')

    rigidFramesNode.addObject('RegularGridTopology',  name='meshLinesCombined',  nx=nb_poutre, ny='1', nz='1', xmax=h_module*nb_module, xmin='0.0', ymin='0', ymax='0',zmin='0',zmax='0')
    # rigidFramesNode.addObject('RegularGridTopology',  name='meshLinesCombined',  nx='1', ny='1', nz=nb_poutre, xmax='0.0', xmin='0.0', ymin='0', ymax='0',zmin='0',zmax=h_module*nb_module)
    rigidFramesNode.addObject('MechanicalObject',  name='DOFs', template='Rigid3d', showObject='1', showObjectScale='1', rotation=[0, -0 ,0], translation = [0,0,0]) # -90 on y
    rigidFramesNode.addObject('BeamInterpolation', name='BeamInterpolation', printLog = '1', defaultYoungModulus=YM_soft_part, dofsAndBeamsAligned='true', straight='1', crossSectionShape='circular', radius=radius)#, radius=5*radius)
    # rigidFramesNode.addObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', computeMass='0', massDensity=0.001)
    rigidFramesNode.addObject('RestShapeSpringsForceField', name='anchor', points='0', stiffness='1e12', angularStiffness='1e12')
      

    stiff_flop = stiff.createRobot(parent = rigidFramesNode, name = "MyStiffFlop",out_flag = out_flag, act_flag = act_flag)

    if nb_module == 2 :
        stiff_2 = rigidFramesNode.getChild('stiff_flop1')

    # MeasuredPosition = EffectorGoal(node=rootNode, position = [0,0,0],name = 'MeasuredPosition',taille = 4)
    # DesiredPosition = EffectorGoal(node=rootNode, position = [0,0,h_effector],name = 'DesiredPosition',taille = 0.5)
    # OrientationGoal = EffectorGoal(node=rootNode, position = [0,0,h_effector],name = 'OrientationGoal',taille = 6)

    if nb_module == 2 :
        MeasuredPosition_2 = EffectorGoal(node=rootNode, position = [0,0,h_effector/2],name = 'MeasuredPosition_2',taille = 7)

    if act_flag == 0 :
        rootNode.addObject('QPInverseProblemSolver', name="QP", printLog='0', saveMatrices = True ,epsilon = 0.01) # initialement epsilon = 0.001

        
         ## 2eme goal poitn pour décalage, permet de mieux attraper à la souris
        if close_loop == 0 :
            goal2 = EffectorGoal(node=rootNode, position = [h_effector+shift,0,0],name = 'goal2',taille = 0.5)

        # CLASSIC 
        # goal = EffectorGoal(node=rootNode, position = [0,0,h_effector],name = 'goal',taille = 0.5)
        # controlledPoints = stiff_flop.addChild('controlledPoints')
        # controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Vec3",position=[h_effector, 0, 0])#,rotation=[0, 90 ,0]) # classic
        # # controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Rigid3",position=[0, 0, h_effector,0., 0., 0., 1.])#,rotation=[0, 90 ,0]) # rigid pour l'orientation
        # controlledPoints.addObject('PositionEffector', template="Vec3d", indices='0', effectorGoal="@../../../goal/goalM0.position") # classic
        # controlledPoints.addObject('BarycentricMapping', mapForces=False, mapMasses=False)

        ## GOAL ON BEAM 
        goal = rootNode.addChild('goal')
        goal.addObject('EulerImplicitSolver', firstOrder=True)
        goal.addObject('CGLinearSolver', iterations=100, threshold=1e-12, tolerance=1e-10)
        goal.addObject('MechanicalObject', name='goalM0', position=[h_effector,0,0,0,0,0,1],template = "Rigid3d",showObject = True,showObjectScale = 10)
        goal.addObject('SphereCollisionModel', radius=0.5)
        # goal.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1e5, stiffness=1e5)
        goal.addObject('UncoupledConstraintCorrection')
        controlledPoints = rigidFramesNode.addChild('controlledPoints')
        controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Rigid3d",position=[0, 0, 0,0,0,0,1],showObject = True,showObjectScale = 5)#,rotation=[0, 90 ,0]) # classic
        controlledPoints.addObject('PositionEffector', template="Rigid3d", indices='0', effectorGoal="@../../goal/goalM0.position") # classic
        # controlledPoints.addObject('BarycentricMapping', mapForces=False, mapMasses=False)
        # controlledPoints.addObject('IdentityMapping', mapForces=False, mapMasses=False)
        controlledPoints.addObject("RigidRigidMapping", index=nb_poutre-1) 

        # ORIENTATION TESTS #ORIENTAION
        # OrientationGoal = stiff_flop.addChild('OrientationGoal')
        # OrientationGoal.addObject('EulerImplicitSolver', firstOrder=True)
        # OrientationGoal.addObject('CGLinearSolver', iterations=100, threshold=1e-12, tolerance=1e-10)
        # OrientationGoal.addObject('MechanicalObject', name='OrientationGoalM0', template='Rigid3', position=[0.,0., 50., 0., 0., 0., 1.], showObject=True, showObjectScale=10)
        # goal.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1e5, stiffness=1e5)
        # OrientationGoal.addObject('UncoupledConstraintCorrection')

        # controlledPoints = rigidFramesNode.addChild('controlledPoints')
        # controlledPoints = rigidFramesNode.addChild('controlledPoints')
        # controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Rigid3",position=[0, 0, h_effector]+[0., 0., 0., 1.])#,rotation=[0, 90 ,0]) # rigid pour l'orientation
        # controlledPoints.addObject('MechanicalObject', name="actuatedPoints", template="Rigid3")#,rotation=[0, 90 ,0]) # rigid pour l'orientation
        # controlledPoints.addObject('PositionEffector', template="Rigid3", indices= 0, effectorGoal="@../../../OrientationGoal/OrientationGoalM0.position",useDirections=[0, 0, 0, 1, 0, 1],limitShiftToTarget=True, maxShiftToTarget=5) # rigid pour l'orientation
        # controlledPoints.addObject('PositionEffector', template="Vec3d", indices='0', effectorGoal="@../../../goal/goalMO.position",maxShiftToTarget=-2.0,limitShiftToTarget = True)
        # controlledPoints.addObject('BarycentricMapping', mapForces=False, mapMasses=False)
        # controlledPoints.addObject("RigidMapping", index=0) #test
        #  # ORIENTATION TESTS #ORIENTAION

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
            if act_flag == 0 : # On peut par la suite ajouter un else pour enregistrer aussi des information en commande directe si nécessaire
                rootNode.addObject(ParameterPrinterCsv(module =stiff,nom_dossier = nom_dossier,RootNode=rootNode,K_I = K_I, K_P = K_P,dt=dt))
                rootNode.addObject(PositionPrinterCsv(node = rigidFramesNode,name = 'DOFs',module =stiff,nom_dossier = nom_dossier,nom_fichier = 'RigidFrames',beam_ind = nb_poutre-1))
                rootNode.addObject(PositionPrinterCsv(node = goal,name = 'goalM0',module =stiff,nom_dossier = nom_dossier,nom_fichier = 'goal'))
                rootNode.addObject(PositionPrinterCsv(node = MeasuredPosition,name = 'MeasuredPositionM0',module =stiff,nom_dossier = nom_dossier,nom_fichier = 'MeasuredPosition'))
                if close_loop == 1 :
                    rootNode.addObject(PositionPrinterCsv(child_name = 'DesiredPosition',name = 'DesiredPositionM0',module =stiff,nom_dossier = nom_dossier,RootNode=rootNode))
                else :
                    rootNode.addObject(PositionPrinterCsv(node = goal2,name = 'goal2M0',module =stiff,nom_dossier = nom_dossier,nom_fichier = 'goal2'))
                if nb_module == 2 :
                    rootNode.addObject(PositionPrinterCsv(node = MeasuredPosition_2,name = 'MeasuredPosition_2M0',module =stiff,nom_dossier = nom_dossier,nom_fichier = 'MeasuredPosition2'))
                    rootNode.addObject(PositionPrinterCsv(node = rigidFramesNode,name = 'DOFs',module =stiff,nom_dossier = nom_dossier,nom_fichier = 'RigidFramesMiddle',beam_ind = (nb_poutre/2)-1))
                    rootNode.addObject(PressurePrinterCsv(module =stiff,nom_dossier = nom_dossier,parent=stiff_flop,act_flag=act_flag,node2 = stiff_2))
                else :
                    rootNode.addObject(PressurePrinterCsv(module =stiff,nom_dossier = nom_dossier,parent=stiff_flop,act_flag=act_flag))


        if setup == 1:
            rootNode.addObject(ArduinoPressure_UCL(module = stiff,RootNode = rootNode, dt = dt)) # for UCL setup
            if nb_module == 1:
                rootNode.addObject(AuroraTracking(child_name = 'MeasuredPosition',name = 'MeasuredPositionM0',module =stiff,RootNode=rootNode))
            elif nb_module == 2:
                rootNode.addObject(AuroraTracking_2_nodes(node = MeasuredPosition,name = 'MeasuredPositionM0',node2 = MeasuredPosition_2,name2 = 'MeasuredPosition_2M0',module =stiff))
        elif setup == 2:
            rootNode.addObject(ArduinoPressure(module = stiff,RootNode = rootNode)) # pour envoyer les pressions calculées par le modèle inverse au robot (hardware) # (mettre après le if suivant !)
            rootNode.addObject(PolhemusTracking(node = MeasuredPosition,name = 'MeasuredPositionM0',offset = [0,0,h_effector]) )


        if act_flag == 0 :
            if close_loop == 1 :
                # test = 2
                # rootNode.addObject(GoalKeyboardController(goal_pas = goal_pas,child_name = 'DesiredPosition',name = 'DesiredPositionM0', RootNode = rootNode)) # for goal without shift
                #rootNode.addObject(CloseLoopController(name="CloseLoopController",RootNode=rootNode, K_P = K_P, K_I = K_I))
                
                rootNode.addObject(CloseLoopController2(name="CloseLoopController",RootNode=rootNode, K_P = K_P, K_I = K_I))


                #rootNode.addObject(LineTrajectory(nb_iter=10,node = DesiredPosition,name = 'DesiredPositionM0',p_begin = [0, 0 , 55], p_end = [15, -15 , 60]))
                #rootNode.addObject(PointPerPointTrajectory(node = DesiredPosition,name = 'DesiredPositionM0',module = stiff,point_tab = point_tab, node_pos = MeasuredPosition, name_pos = 'MeasuredPositionM0',err_d = 50,shift=0,beam_flag = 0))
                # rootNode.addObject(CircleTrajectory(rayon =circle_radius, nb_iter = nb_iter_circle,node = DesiredPosition,name = 'DesiredPositionM0',circle_height = circle_height -5, module=stiff))

                #rootNode.addObject(SquareTrajectory(rayon =square_radius, nb_iter = nb_iter_square,node = DesiredPosition,name = 'DesiredPositionM0',square_height = square_height,module=stiff))
                # rootNode.addObject(SquareTrajectory(RootNode = rootNode, rayon =square_radius, nb_iter = nb_iter_square,child_name = 'DesiredPosition',name = 'DesiredPositionM0',square_height = square_height,module=stiff))
                
                # rootNode.addObject(PrintGoalPos(name="CloseLoopController",RootNode=rootNode))
                rootNode.addObject(PointPerPointTrajectory(node = DesiredPosition,name = 'DesiredPositionM0',module = stiff,point_tab = point_tab, node_pos = MeasuredPosition, name_pos = 'MeasuredPositionM0',err_d = 50,shift=0,beam_flag = 0))   
            else : # open loop ( close_loop == 0 )     
                # rootNode.addObject(PressurePrinter_local(module = stiff,node = rigidFramesNode))
                # rootNode.addObject(PositionViewer(module = stiff,node = rigidFramesNode))

                # rootNode.addObject(PointPerPointTrajectory(node = goal,name = 'goalM0',module = stiff,point_tab = point_tab, node_pos = rigidFramesNode, name_pos = 'DOFs',err_d = 50,shift=0,type_flag = 1))

                # rootNode.addObject(OrientationControl(node = goal,name = 'goalM0'))

        
                # rootNode.addObject(LineTrajectory(nb_iter=20,node = goal2,name = 'goal2M0',p_begin = [0, 0 , 55], p_end = [10, -10 , 55]))
                # rootNode.addObject(PointPerPointTrajectory(node = goal2,name = 'goal2M0',module = stiff,point_tab = point_tab, node_pos = rigidFramesNode, name_pos = 'DOFs',err_d = 50,shift=0,type_flag = 1))
                rootNode.addObject(CircleTrajectory(rayon =circle_radius, nb_iter = nb_iter_circle, node = goal2,name = 'goal2M0',circle_height = circle_height+0,module=stiff,axis = [0,1,1]))
                # rootNode.addObject(SquareTrajectory(length =square_radius, nb_iter = nb_iter_square,node = goal2,name = 'goal2M0',square_height = square_height+0,module=stiff))

                #rootNode.addObject(GoalKeyboardController(goal_pas = goal_pas,node = goal2,name = 'goal2M0')) # for goal with shift
                rootNode.addObject(GoalShift(node_follow= goal ,object_follow = 'goalM0',node_master = goal2,object_master = 'goal2M0',shift_tab = [0,0,0]))
        elif act_flag == 1 :
            rootNode.addObject(PositionViewer(module = stiff,node = rigidFramesNode,nb_poutre=nb_poutre))
            if nb_module == 1 :
                rootNode.addObject(StiffController(pas=pas,module = stiff,parent = stiff_flop))
            elif nb_module == 2 :
                rootNode.addObject(StiffController(pas=pas,module = stiff,parent = stiff_flop,node2 = stiff_2))
            #rootNode.addObject(AuroraTracking(child_name = 'MeasuredPosition',name = 'MeasuredPositionM0',module =stiff,RootNode=rootNode))


    # rootNode.addObject(PrintBox(noeud = stiff_flop))

    return rootNode

def createScene(rootNode):
    MyScene(rootNode,out_flag = 0,step = 0,YM_soft_part=YM_soft,coef_poi = coef_poisson,data_exp = 0,act_flag = act_flag) # act_flag = 0- IP ; 1- Direct Control


    
    
   

    
   
