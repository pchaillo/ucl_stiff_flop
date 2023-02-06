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

from stlib3.physics.mixedmaterial import rigidification as ri
import meshing_functions as ri

from splib3.objectmodel import setData

from stlib3.scene import MainHeader


from Stiff_Function import *
from flopMultiController import *
from SimuController_ucl import *
from TrajectoryController import *
from CloseLoopController import *
from UCL_Controller import *
# from polhemus_liberty.python.Polhemus_SOFA_Controller import *

import time

############## PARAM7TRES A FIXER ####################
## FLAG ##
act_flag = 1 # set 0 for IP (Inverse Problem resolution with QP) and 1 for direct control
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
point_tab = [ [5,5,55], [10,10,55],[10,5,55],[10,0,55],[10,-5,55],[10,-10,55],[5,-10,55],[5,-5,55],[5,0,55], [0,0,55],[0,0,60], [-5,5,60], [-10,10,60],[-10,5,60],[-10,0,60],[-10,-5,60],[-10,-10,60],[-5,-10,60],[-5,-5,60],[-5,0,60], [0,0,60], [0,0,55]] # once the robot will have reach all the positions, he will start again with the 1st position


d_et_h = str(datetime.now())
nom_dossier = d_et_h[0:19]

position = [0,0,h_effector]
############## PARAM7TRES -- FIN ###################

# stiff = Stiff_Flop(h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model,nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool)

def MyScene(rootNode, out_flag,step,YM_soft_part,coef_poi,act_flag,data_exp):

    MainHeader(rootNode)

    stiff = Stiff_Flop(h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model,nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool,min_pression,force_field,dynamic,dt,nb_slices,r_disk_chamber,r_cavity)
    
    rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/SoftRobots/lib/') #libSoftRobots.so 1.0
    rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/ModelOrderReduction/lib/') #libSoftRobots.so 1.0
    rootNode.addObject('AddPluginRepository', path = '/home/pchaillo/Documents/10-SOFA/sofa/build/master/external_directories/plugins/BeamAdapter/lib')#/libBeamAdapter.so 1.0

    # required plugins:
    rootNode.addObject('RequiredPlugin', name='SoftRobots.Inverse')
    rootNode.addObject('RequiredPlugin', name='SoftRobots')
    rootNode.addObject('RequiredPlugin', name='BeamAdapter')
    # rootNode.addObject('RequiredPlugin', name='SofaConstraint')
    # rootNode.addObject('RequiredPlugin', name='SofaDeformable')
    # rootNode.addObject('RequiredPlugin', name='SofaGeneralAnimationLoop')
    rootNode.addObject('RequiredPlugin', name='SofaImplicitOdeSolver')
    # rootNode.addObject('RequiredPlugin', name='SofaLoader')
    # rootNode.addObject('RequiredPlugin', name='SofaMeshCollision')
    rootNode.addObject('RequiredPlugin', name='SofaSimpleFem')
    # rootNode.addObject('RequiredPlugin', name='SofaSparseSolver')
    rootNode.addObject('RequiredPlugin', name='SofaEngine')
    # rootNode.addObject('RequiredPlugin', name='SofaGeneralLoader')
    # rootNode.addObject('RequiredPlugin', name='SofaGeneralEngine')
    #rootNode.addObject('RequiredPlugin', name='SofaPython')

    # rootNode.findData('gravity').value=[0, 0, 9810];
    # rootNode.findData('gravity').value=[0, 0, 0];

        #visual dispaly
    rootNode.addObject('VisualStyle', displayFlags='showVisualModels showBehaviorModels showCollisionModels hideBoundingCollisionModels showForceFields showInteractionForceFields hideWireframe')
    rootNode.addObject('BackgroundSetting', color='0 0.168627 0.211765')
    #rootNode.createObject('OglSceneFrame', style="Arrows", alignment="TopRight")
    
    rootNode.findData('dt').value= dt;
    
    # rootNode.addObject('FreeMotionAnimationLoop') # Hein ? # A mettre pour la poutre ?
    rootNode.addObject('DefaultVisualManagerLoop')   

    rigidFramesNode  = rootNode.addChild('RigidFrames')
    # if dynamic == 1 :
    #     rigidFramesNode.addObject('EulerImplicitSolver', firstOrder='0', vdamping=0,rayleighStiffness='0.3',rayleighMass='0.1')
    # else :
    #     rigidFramesNode.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)

    # rigidFramesNode.addObject('SparseLDLSolver', name='ldlsolveur',template="CompressedRowSparseMatrixd")    # CompressedRowSparseMatrixMat3x3d (plus rapide pour les Vec3d)
    # # rigidFramesNode.addObject('SofaDenseSolver', name='ldlsolveur')  # test  
    # rigidFramesNode.addObject('GenericConstraintCorrection')

    rigidFramesNode.addObject('RegularGridTopology',  name='meshLinesCombined',  nx=nb_poutre, ny='1', nz='1', xmax=h_module*nb_module, xmin='0.0', ymin='0', ymax='0',zmin='0',zmax='0')
    # rigidFramesNode.addObject('RegularGridTopology',  name='meshLinesCombined',  nx='1', ny='1', nz=nb_poutre, xmax='0.0', xmin='0.0', ymin='0', ymax='0',zmin='0',zmax=h_module*nb_module)
    rigidFramesNode.addObject('MechanicalObject',  name='DOFs', template='Rigid3d', showObject='1', showObjectScale='1', rotation=[0, -90 ,0], translation = [0,0,0]) # -90 on y
    rigidFramesNode.addObject('BeamInterpolation', name='BeamInterpolation', printLog = '1', defaultYoungModulus=YM_soft_part, dofsAndBeamsAligned='true', straight='1', crossSectionShape='circular', radius=radius)#, radius=5*radius)
    # rigidFramesNode.addObject('AdaptiveBeamForceFieldAndMass', name='BeamForceField', computeMass='0', massDensity=0.001)
    rigidFramesNode.addObject('RestShapeSpringsForceField', name='anchor', points='0', stiffness='1e12', angularStiffness='1e12')
      

    # ModelNode  = rootNode.addChild('ModelNode')
    # stiff_flop = stiff.createRobot2(parent = ModelNode, name = "MyStiffFlop",out_flag = out_flag, act_flag = act_flag) # for working version

    # ModelNode  = rigidFramesNode.addChild('ModelNode')
    # stiff_flop = stiff.createRobot2(parent = ModelNode, name = "MyStiffFlop",out_flag = out_flag, act_flag = act_flag) # for working mapping between beams and cavities 

    ModelNode  = rigidFramesNode.addChild('ModelNode') 
    stiff_flop = stiff.createRobot3(BeamNode = rigidFramesNode,ModelNode = ModelNode, name = "MyStiffFlop",out_flag = out_flag, act_flag = act_flag) # version pour essayer de faire le mapping entre le FEM et la poutre

    stiff_flop.addObject('LinearSolverConstraintCorrection', name='correction')

    if act_flag == 0 :
        rootNode.addObject('QPInverseProblemSolver', name="QP", printLog='0', saveMatrices = True ,epsilon = 0.01) # initialement epsilon = 0.001
    elif act_flag == 1:
        rootNode.addObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')

    display_flag = 1
    stiff_flop.addObject('BoxROI',name="RigidificationBox" , template="Vec3d" ,box= [48, -10, -10, 52, 8, 10] ,drawBoxes=display_flag ,strict=True)#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
    object_list = ['topo' ,'engine', 'container','tetras','RigidificationBox'] # A FAIRE DE MANI7RE PROPRE !!!!
    # object_list = ['topo' ,'engine', 'container','modifier','tetras'] # A FAIRE DE MANI7RE PROPRE !!!!
    object_list_init(object_list=object_list,node=stiff_flop)
    Boite = stiff_flop.getObject('RigidificationBox')
    indices_rigid = copy(Boite.indices.value)
    corps = stiff_flop.getObject('tetras')
    # print("rr")
    # print(indices_rigid)
    # print(type(indices_rigid))
    indices_rigid = [list(indices_rigid)]
    # print(indices_rigid)
    # print(type(indices_rigid))

    # test = ri.Rigidify(stiff_flop, corps,groupIndices = indices_rigid) # Ok faut utiliser le template ElasticMaterial => naze je vais (essayer) de refaire une version moi-même
    NewFEM_Node = ri.Rigidify2(targetNode = ModelNode , sourceNode = stiff_flop, sourceObject = corps,groupIndices = indices_rigid) # Version maison

        # Activate some rendering on the rigidified object.
    setData(NewFEM_Node.RigidParts.dofs, showObject=True, showObjectScale=1, drawMode=2)
    setData(NewFEM_Node.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=0.1,
            drawMode=1, showColor=[1., 1., 0., 1.])
    setData(NewFEM_Node.DeformableParts.dofs, showObject=True, showObjectScale=0.1, drawMode=2)
    # test.RigidParts.addObject("FixedConstraint", indices=0) # Pou fixer le position d'un point dans l'espace


    SimulationNode  = rootNode.addChild('SimulationNode') # ne marche pas => trouver comment faire le lien entre la beam et l'objet split en 2
    if dynamic == 1 :
        SimulationNode.addObject('EulerImplicitSolver', firstOrder='0', vdamping=0,rayleighStiffness='0.3',rayleighMass='0.1')
    else :
        SimulationNode.addObject('EulerImplicitSolver', firstOrder='1', vdamping=0)

    # SimulationNode.addObject("CGLinearSolver", iterations=25, tolerance=1e-5, threshold=1e-5)
    SimulationNode.addObject('SparseLDLSolver', name='ldlsolveur',template="CompressedRowSparseMatrixd")    # CompressedRowSparseMatrixMat3x3d (plus rapide pour les Vec3d)
    # SimulationNode.addObject('SparseLUSolver', name='ldlsolveur')  # test  
    SimulationNode.addObject('GenericConstraintCorrection')
    SimulationNode.addObject('GenericConstraintSolver', maxIterations='100', tolerance = '0.0000001')

    # rigidNode = SimulationNode.addChild(NewFEM_Node) # On met le FEM en enfant du noeud simulation => fonctionne !

    # SimulationNode  = rootNode.addChild('SimulationNode') # On met le noeud poutre en enfant du noeud simu, lui même parent du noeud FEM : ne marche pas => trouver comment faire le lien entre la beam et l'objet split en 2
    rigid_frame_bis = SimulationNode.addChild(rigidFramesNode)

    # rigidNode = rigid_frame_bis.addChild(NewFEM_Node) # NewFEMNODE déjà enfant de beam avec en intermédiaire ModelNode

    ri.AddBeamMapping(BeamNode = rigid_frame_bis, ModelNode = ModelNode, MixedFEM = NewFEM_Node )

    # rigidNode = SimulationNode.addChild(stiff_flop)

    rootNode.addObject(StiffController(pas=pas,module = stiff,parent = stiff_flop))

    return rootNode

def createScene(rootNode):
    MyScene(rootNode,out_flag = 0,step = 0,YM_soft_part=YM_soft,coef_poi = coef_poisson,data_exp = 0,act_flag = act_flag) # act_flag = 0- IP ; 1- Direct Control


    
    
   

    
   
