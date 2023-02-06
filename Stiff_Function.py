import Sofa.Core

import meshing_functions as mf

# utilise Rigidify de STLIB
#from stlib3.physics.mixedmaterial import Rigidify
from os import getcwd
from math import sin,cos, sqrt, acos, radians
from spicy import copy

# permet d'initialiser un robot au nb de module voulu :
# - mettre tous les paramètres dans l'initialisation de la variable
# - appeler ensuite seulemnt createRobot qui appelle les autres fonctions, tous les paramètres étant dans l'initialisation (voir stiff_module.pyscn)

# def auto_stl_choice(h_module,stl_base): 
#     return

def object_list_init(object_list,node):
    for i in object_list :
        node_object = node.getObject(i)
        node_object.init()

def EffectorGoal(node, position,name,taille):
    goal = node.addChild(name)
    goal.addObject('EulerImplicitSolver', firstOrder=True)
    goal.addObject('CGLinearSolver', iterations=100, threshold=1e-12, tolerance=1e-10)
    goal.addObject('MechanicalObject', name=name + 'M0', position=position)
    goal.addObject('SphereCollisionModel', radius=taille)
    # goal.addObject('RestShapeSpringsForceField', points=0, angularStiffness=1e5, stiffness=1e5)
    goal.addObject('UncoupledConstraintCorrection')
    return goal

def define_mesh_path(mesh_name,out_flag):
    path = getcwd()
    if out_flag == 0:
        inter = '/mesh/'
    else :
        inter = '/ucl_collaboration/mesh/'

    file_path = path + inter +mesh_name
    print("########################### OK")
    print(str(file_path))
    return file_path

def get_extension(string):
    length = len(string)
    string_out = string[length-4:length]
    return string_out


class Stiff_Flop() :  

    def __init__(self,h_module,init_pressure_value,value_type,YM_soft_part,YM_stiff_part,coef_poi,nb_cavity,chamber_model, nb_module,module_model,max_pression,name_cavity,masse_module,nb_poutre,rigid_base,rigid_top,rigid_bool,min_pression,force_field,dynamic,dt,nb_slices,r_disk_chamber,r_cavity):
        self.h_module = h_module
        self.init_pressure_value = init_pressure_value
        self.value_type = value_type
        self.YM_soft_part = YM_soft_part
        self.YM_stiff_part = YM_stiff_part
        self.coef_poi = coef_poi
        self.nb_cavity = nb_cavity
        self.i_cavity = 0
        self.ang_dec = 360/nb_cavity # calcul du placement des cavités
        self.chamber_model = chamber_model
        self.nb_module = nb_module
        self.module_model = module_model
        self.max_pression = max_pression
        self.name_cavity = name_cavity
        self.masse_module = masse_module
        self.nb_poutre = nb_poutre
        self.rigid_base = rigid_base
        self.rigid_top = rigid_top
        self.rigid_bool = rigid_bool
        self.min_pression = min_pression
        self.force_field = force_field
        self.dyn_flag = dynamic
        self.dt = dt
        self.nb_slices = nb_slices
        self.r_disk_chamber = r_disk_chamber
        self.r_cavity = r_cavity

        self.r_disk_box = r_disk_chamber + r_cavity/3 # *1.2 just to be a bit smaller
        self.l_box = r_cavity*2.6 # Box length.  2* because it's radius and not diameter, and *0.6 more to be a bit bigger than the cavities itself


    def createCavity(self,parent,name_c,i,cavity_model,act_flag): # for v1 -------
        bellowNode = parent.addChild(name_c+str(i+1))
        MeshLoad = bellowNode.addObject('MeshSTLLoader', filename=cavity_model, flipNormals='0', triangulate='true', name='meshLoader',rotation=[0,0,self.ang_dec*self.i_cavity], translation=[0, 0,self.h_module*i])#, rotation=[self.ang_dec*self.i_cavity,0,0] if pre-rotated 3D model
        MeshLoad.init()
        points  = MeshLoad.position.value
        triangles = MeshLoad.triangles.value

        # ## Pour remmettre les indices dans le bon ordre #009
        # l = len(points)
        # indices = [k for k in range(l)]
        # [new_points, new_points_l] = mf.new_index(points = points, axis = 2,old_indices = indices) # axis = 0 => x //  axis = 1 => y  // axis = 2 => z 
        # new_triangle = mf.reindex_mesh(new_points_list=new_points_l,mesh=triangles)
        # ##  #009

        [new_points, new_points_l,new_triangle] = mf.remesh(points = points,mesh = triangles,axis= 2 )

        MeshLoad.position.value = new_points
        MeshLoad.triangles.value = new_triangle

        # bellowNode.addObject('MeshSTLLoader', filename=cavity_model, flipNormals='0', triangulate='true', name='meshLoader',rotation=[0,self.ang_dec*self.i_cavity,0], translation=[0, 0,self.h_module*i])#, rotation=[self.ang_dec*self.i_cavity,0,0] if pre-rotated 3D model
        bellowNode.addObject('MeshTopology', src='@meshLoader', name='chambreAMesh'+str(i+1))
        bellowNode.addObject('MechanicalObject', name='chambreA'+str(i+1),rotation=[0, 90 , 0])#,translation = [0,0,h_module*i]) # 90 on y
        bellowNode.addObject('TriangleCollisionModel', moving='0', simulated='1')
        if act_flag == 0 :
            bellowNode.addObject('SurfacePressureActuator', name='SPC', template = 'Vec3d',triangles='@chambreAMesh'+str(i+1)+'.triangles',minPressure = self.min_pression,maxPressure = self.max_pression)#,maxPressureVariation = 20)#,valueType=self.value_type)
        elif  act_flag == 1 :
            bellowNode.addObject('SurfacePressureConstraint', name='SPC', triangles='@chambreAMesh'+str(i+1)+'.triangles', value=self.init_pressure_value,minPressure = self.min_pression,maxPressure = self.max_pression, valueType=self.value_type)#,maxPressureVariation = 20)#,
        # bellowNode.addObject('SurfacePressureModel', name='SPC_model', minPressure = 0,maxPressure = 20)
        # bellowNode.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./chambreA'+str(i+1) )
        self.i_cavity = self.i_cavity + 1;
        if self.i_cavity == self.nb_cavity :
            self.i_cavity = 0

        # circles = mf.circle_detection(points = new_points, pt_per_slice = 12) # pas 12 pt par étage à partir du stl classique => ne fonctionne pas
        # print(circles)
        
        # bellowNode.addObject('StiffSpringForceField')

        return bellowNode

    def createModule(self,parent,name_c,i,model):
        module = parent.addChild('stiff_flop'+str(i+1))
        module.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        module.addObject('SparseLDLSolver', name='directSolver' , template="CompressedRowSparseMatrixd")
        # module.addObject('MeshSTLLoader', filename='mesh/solid_STIFF_FLOP02_attach_s.stl', flipNormals='0', triangulate='true', name='loader' )#, translation=[4., 0,-3.275], rotation=[0,0,90])
        module.addObject('MeshVTKLoader', name='loader', filename=model,translation = [0,0,self.h_module*i], rotation=[0, 0 , 0]) # 
        module.addObject('MeshTopology', src='@loader', name='container')
        module.addObject('MechanicalObject', name='tetras', template='Vec3', showObject=True, showObjectScale=1,rotation=[0, 90 , 0])#,translation = [0,0,h_module*i]) # 90 on y
        # module.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_soft_part) # stable youngModulus = 500 / réel ? = 103
        module.addObject('UniformMass', totalMass=self.masse_module)
        return module

    def createModule_extrude(self,parent,name_c,i,model):
        module = parent.addChild('stiff_flop'+str(i+1))

        # module.addObject('EulerImplicitSolver', name='odesolver', rayleighStiffness=0.1, rayleighMass=0.1)
        # module.addObject('SparseLDLSolver', name='directSolver' , template="CompressedRowSparseMatrixd")

        #nb_slices = 16

        object_list = ['topo' ,'engine', 'container','modifier','tetras'] # to try do init of objects in SOFA
        module.addObject('MeshOBJLoader', name=object_list[0] , filename=model,translation = [self.h_module*i,0,0],rotation=[0, 0 , 90])
        engine = module.addObject('ExtrudeQuadsAndGenerateHexas', name=object_list[1], template='Vec3d', thicknessIn='0.0', thicknessOut=-self.h_module, numberOfSlices=self.nb_slices, surfaceVertices='@topo.position', surfaceQuads='@topo.quads' )
        
        engine.init() #auncun effet ? Du à la 2nd initialisation des points ? 'pas effectif sur les cavités ?)' #003
        hexas = engine.extrudedHexas.value
        points = engine.extrudedVertices.value 
        [new_points, new_points_l,new_hexas] = mf.remesh(points = points,mesh = hexas,axis= 1 ) 
        # print("TEST")
        # print(new_points)
        engine.extrudedHexas.value = new_hexas
        engine.extrudedVertices.value = new_points # 003

        module.addObject('HexahedronSetTopologyContainer', position='@engine.extrudedVertices', hexas='@engine.extrudedHexas', name = object_list[2])
        module.addObject('HexahedronSetTopologyModifier', name = object_list[3])
        # finger.createObject('HexahedronSetTopologyAlgorithms')
        # finger.createObject('HexahedronSetGeometryAlgorithms')
        module.addObject('MechanicalObject',name=object_list[4], template="Vec3d", position='@container.position', showIndices="false", showIndicesScale="4e-5", ry="0", rz="0")#, dx="55", dy="50", dz="-25", )
        # module.addObject('MechanicalObject',name="tetras", template="Vec3d", position='@container.position', showIndices="false", showIndicesScale="4e-5",dz = h_seal)#,translation = [0,0,h_seal*10])#, ry="0", rz="-90", dx="55", dy="50", dz="-25", )
        module.addObject('UniformMass', totalMass='0.05')
        # module.addObject('HexahedronFEMForceField' , template='Vec3d', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_soft_part)
        

        # module.addObject('MeshOBJLoader', name="topo" , filename=model,translation = [self.h_module*i,0,0],rotation=[0, 0 , 90])
        # engine = module.addObject('ExtrudeQuadsAndGenerateHexas', name='engine', template='Vec3d', thicknessIn='0.0', thicknessOut=-self.h_module, numberOfSlices=self.nb_slices, surfaceVertices='@topo.position', surfaceQuads='@topo.quads' )
        # module.addObject('HexahedronSetTopologyContainer', position='@engine.extrudedVertices', hexas='@engine.extrudedHexas' )
        # module.addObject('HexahedronSetTopologyModifier')
        # # finger.createObject('HexahedronSetTopologyAlgorithms')
        # # finger.createObject('HexahedronSetGeometryAlgorithms')
        # module.addObject('MechanicalObject',name="tetras", template="Vec3d", position='@container.position', showIndices="false", showIndicesScale="4e-5", ry="0", rz="0")#, dx="55", dy="50", dz="-25", )
        # # module.addObject('MechanicalObject',name="tetras", template="Vec3d", position='@container.position', showIndices="false", showIndicesScale="4e-5",dz = h_seal)#,translation = [0,0,h_seal*10])#, ry="0", rz="-90", dx="55", dy="50", dz="-25", )
        # module.addObject('UniformMass', totalMass='0.05')
        # # module.addObject('HexahedronFEMForceField' , template='Vec3d', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_soft_part)

        return module

    def OneCavitFEM(self,points,quads,indices,axis = 0):
        [new_points, old_ind_eq_tab,new_quads] = mf.remesh(points = points,mesh = quads,axis= 0 ,old_indices=indices)
        triangles = mf.quad_2_triangles(quads=new_quads)

        [circles, ind_tab] = mf.circle_detection(points = new_points, pt_per_slice = 12)
        # closing_tri = mf.close_cavity(circles,ind_tab) # ind not good for triangle
        [new_circle_tab,new_ind_tab_full] = mf.ordering_cylinder(circles,ind_tab)
        l = len(new_ind_tab_full)
        closing_tri = mf.close_cavity_2(ind_bottom = new_ind_tab_full[0],ind_top = new_ind_tab_full[l-1])
        # print(circles)

        triangles = triangles + closing_tri

        return [new_points, triangles,old_ind_eq_tab]

    def createCavityFromFEM(self,parent,module,i,act_flag):
        # for j in range(self.nb_cavity): pour la prochaine version plus universelle

        # bellowNode = parent.addChild(name_c+str(i+1)) # Cavities node

        Dx = self.r_disk_box*sin(radians(30))
        Dy = self.r_disk_box*cos(radians(30))
        Ex = -self.r_disk_box
        Ey = 0
        Fx = self.r_disk_box*cos(radians(60))
        Fy = - self.r_disk_box*sin(radians(60))

        D1x = Dx + self.l_box
        D1y = Dy
        F1x = Fx + self.l_box
        F1y = Fy

        E2x = Ex - self.l_box*sin(radians(30))
        E2y = Ey - self.l_box*cos(radians(30))
        F2x = Fx - self.l_box*sin(radians(30))
        F2y = Fy - self.l_box*cos(radians(30))

        D3y = Dy + self.l_box*cos(radians(30))
        D3x = Dx - self.l_box*sin(radians(30))
        E3x = Ex - self.l_box*sin(radians(30))
        E3y = Ey + self.l_box*cos(radians(30))

        coef_div = 2
        DA3x = (D3x + E3x)/coef_div
        DA3y = (D3y + E3y)/coef_div
        DB3x = (Dx + Ex)/coef_div
        DB3y = (Dy + Ey)/coef_div

        EA2x = (F2x + E2x)/coef_div
        EA2y = (F2y + E2y)/coef_div

        FA1x = (F1x + D1x)/coef_div
        FA1y = (F1y + D1y)/coef_div

        III_K_x = Dx+1-2 # Pour être sûr que les display et les vraies boites correspondent => A faire pour toutes les biotes si il ya un pb
        III_K_y = Dy+0.5-1.5
        III_K_x2 = D3x+1-2
        III_K_y2 = D3y+0.5-1.5
        III_K_x3 = DA3x+1 
        III_K_y3 = DA3y+0.5

        II_K_x = Ex +1.2
        II_K_y = Ey -0.6
        II_K_x2 = E2x +1.2
        II_K_y2 = E2y -0.6
        II_K_x3 = EA2x -0.7
        II_K_y3 = EA2y +0.7

        II_H_x = EA2x + 0.7
        II_H_y = EA2y - 0.7
        II_H_x2 = F2x -1
        II_H_y2 = F2y + 1
        II_H_x3 = Fx -1
        II_H_y3 = Fy + 1

        I_K_x = Fx # + 1
        I_K_y = Fy +1.5 #- 1
        I_K_x2 = F1x
        I_K_y2 = F1y +1.5 
        I_K_x3 = FA1x
        I_K_y3 = FA1y -1

        I_H_x = FA1x
        I_H_y = FA1y + 1
        I_H_x2 = D1x
        I_H_y2 = D1y -1.5
        I_H_x3 = Dx
        I_H_y3 = Dy -1.5

        # boxNodes = module.addChild("BoxNodes") # ok ça ne marche pas du tout

        ## Pour voir ou sont les box sur le FEM
        display_flag = 0
        module.addObject('BoxROI',name="DISPLAY_boxROI_III_K"+str(i+1) , template="Vec3d" ,orientedBox= [III_K_x, III_K_y, 0 ,III_K_x2, III_K_y2, 0 ,III_K_x3, III_K_y3,0 , self.h_module*2] ,drawBoxes=display_flag ,strict=True)#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        module.addObject('BoxROI',name="DISPLAY_boxROI_III_H"+str(i+1) , template="Vec3d" ,orientedBox= [DA3x-0.6, DA3y-0.6, 0 ,E3x+1.5 ,E3y+0.5 ,0,Ex+1.5, Ey+0.5, 0,self.h_module*2] ,drawBoxes=display_flag ,strict=True)#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        module.addObject('BoxROI',name="DISPLAY_boxROI_II_K"+str(i+1) , template="Vec3d" ,orientedBox= [II_K_x, II_K_y, 0 ,II_K_x2,II_K_y2 , 0 ,II_K_x3 ,II_K_y3,0 , self.h_module*2] ,drawBoxes=display_flag,strict=True )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        module.addObject('BoxROI',name="DISPLAY_boxROI_II_H"+str(i+1) , template="Vec3d" ,orientedBox= [II_H_x, II_H_y, 0 ,II_H_x2,II_H_y2 , 0 ,II_H_x3 ,II_H_y3,0 , self.h_module*2] ,drawBoxes=display_flag ,strict=True)#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        module.addObject('BoxROI',name="DISPLAY_boxROI_I_K"+str(i+1) , template="Vec3d" ,orientedBox= [I_K_x, I_K_y, 0 ,I_K_x2,I_K_y2 , 0 ,I_K_x3 ,I_K_y3,0 , self.h_module*2] ,drawBoxes=display_flag ,strict=True)#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        module.addObject('BoxROI',name="DISPLAY_boxROI_I_H"+str(i+1) , template="Vec3d" ,orientedBox= [I_H_x, I_H_y, 0 ,I_H_x2,I_H_y2 , 0 ,I_H_x3 ,I_H_y3,0 , self.h_module*2] ,drawBoxes=display_flag )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />

        module.addObject('BoxROI',name="boxROI_III_K"+str(i+1) , template="Vec3d" ,orientedBox= [0, III_K_y , III_K_x ,0,III_K_y2, III_K_x2 ,0 ,III_K_y3,III_K_x3 , self.h_module*2] ,drawBoxes=display_flag ,strict=True,drawQuads = True )#,doUpdate = False)#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        module.addObject('BoxROI',name="boxROI_III_H"+str(i+1) , template="Vec3d" ,orientedBox= [0, DA3y-0.6, DA3x-0.6 ,0 ,E3y+0.5 ,E3x+1.5,0, Ey+0.5, Ex+1.5,self.h_module*2] ,drawBoxes=display_flag,strict=True)#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        module.addObject('BoxROI',name="boxROI_II_K"+str(i+1) , template="Vec3d" ,orientedBox= [0, II_K_y, II_K_x ,0,II_K_y2 , II_K_x2 , 0,II_K_y3,II_K_x3 , self.h_module*2] ,drawBoxes=display_flag ,strict=True)#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        module.addObject('BoxROI',name="boxROI_II_H"+str(i+1) , template="Vec3d" ,orientedBox= [0, II_H_y, II_H_x ,0,II_H_y2 , II_H_x2 ,0 ,II_H_y3,II_H_x3 , self.h_module*2] ,drawBoxes=display_flag ,strict=True)#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        module.addObject('BoxROI',name="boxROI_I_K"+str(i+1) , template="Vec3d" ,orientedBox= [0, I_K_y, I_K_x ,0,I_K_y2 , I_K_x2 ,0 ,I_K_y3,I_K_x3 , self.h_module*2] ,drawBoxes=display_flag ,strict=True )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        module.addObject('BoxROI',name="boxROI_I_H"+str(i+1) , template="Vec3d" ,orientedBox= [0, I_H_y, I_H_x ,0,I_H_y2 , I_H_x2 ,0 ,I_H_y3,I_H_x3 , self.h_module*2] ,drawBoxes=display_flag ,strict=True)#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />

        
        object_list = ['topo' ,'engine', 'container','modifier','tetras'] # A FAIRE DE MANI7RE PROPRE !!!!
        object_list_init(object_list=object_list,node=module)

        object_list2 = ['boxROI_III_K1' ,'boxROI_III_H1', 'boxROI_II_K1','boxROI_II_H1','boxROI_I_K1','boxROI_I_H1'] 
        object_list_init(object_list=object_list2,node=module)
        # loader = module.getObject("topo")
        # loader.init()

        for j in range(self.nb_cavity):
            name_base = "boxROI_" + "I"*(j+1) 

            Boite_K = module.getObject(name_base + "_K" + str(i+1))
            Boite_H = module.getObject(name_base + "_H" + str(i+1))

            # print("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU \n \n \n")
            # print(copy(Boite_III_K.pointsInROI.value))
            # print(copy(Boite_III_K.indices.value))
            # print(copy(Boite_III_K.quadInROI.value))
            # print(copy(Boite_III_K.quadIndices.value))
            # # print(copy(Boite_III_K.edgesInROI.value))
            # print(copy(Boite_III_K.nbIndices.value))
            # print("\n \n \n UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU")
             
            # ## Remeshing part
            quads = copy(Boite_K.quadInROI.value)
            points = copy(Boite_K.pointsInROI.value)
            indices = copy(Boite_K.indices.value)

            # triangles_A = self.OneCavitFEM()

            [new_points_K, triangles_K,old_ind_eq_tab_K] = self.OneCavitFEM(points=copy(Boite_K.pointsInROI.value),quads=Boite_K.quadInROI.value,indices=Boite_K.indices.value)
            [new_points_H, triangles_H,old_ind_eq_tab_H] = self.OneCavitFEM(points=copy(Boite_H.pointsInROI.value),quads=Boite_H.quadInROI.value,indices=Boite_H.indices.value)
            
            print("PPPPPPPPPPPPPPPPPPP9999999999999999999999999999")
            # print(triangles)
            print(old_ind_eq_tab_K)
            print(old_ind_eq_tab_H)

            nb_tri = len(new_points_K) # - 1 ?
            print(nb_tri)

            print(triangles_K)
            print(triangles_H)

            for d in range(len(triangles_H)):
                triangles_H[d] = [ triangles_H[d][0] + nb_tri, triangles_H[d][1] + nb_tri, triangles_H[d][2] + nb_tri  ]

            print(triangles_H)

            triangles = [*triangles_K, *triangles_H]
            new_points = [*new_points_K, *new_points_H]

            print(len(new_points))

            [new_points2, old_ind_eq_tab_tot, triangles] = mf.remesh(points = new_points,mesh = triangles,axis= 2 )

            # print("PPPPPPPPPPPPPPPPPPP")
            # # print(triangles)
            # print(len(new_points))
            # print(len(new_points_K))

            # triangles = triangles_K.extend(triangles_H)
            # new_points = new_points_K + new_points_H

            # act_flag = 1

            triangles = mf.invers_normal(triangles)

            Cavity = module.addChild("Bellow"+ str(j+1)+ str(i+1))
            # Cavity.addObject("PointSetTopologyContainer", points = "@../Boite_III_K1.pointsInROI" )
            # Cavity.addObject("PointSetTopologyContainer", points = Boite_III_K.pointsInROI.value )
            # Cavity.addObject("QuadSetTopologyContainer", position =Boite_III_K.pointsInROI.value, points = Boite_III_K.indices.value,quad = copy(Boite_III_K.quadInROI.value))# quad = copy(Boite_III_K.quadIndices.value) )
            Cavity.addObject("TriangleSetTopologyContainer", triangles = triangles,name="meshLoader",points = new_points2)#, position =Boite_III_K.pointsInROI.value, points = Boite_III_K.indices.value, quad = Boite_III_K.quadIndices.value )

            # Cavity.addObject('MeshTopology', src='@meshLoader', name='chambreAMesh'+str(i+1))
            Cavity.addObject('MechanicalObject', name='chambreA'+str(i+1),rotation=[0, 0 , 0])#,translation = [0,0,h_module*i]) # 90 on y
            Cavity.addObject('TriangleCollisionModel', moving='0', simulated='1')
            if act_flag == 0 :
                Cavity.addObject('SurfacePressureActuator', name='SPC', template = 'Vec3d',triangles='@chambreAMesh'+str(i+1)+'.triangles',minPressure = self.min_pression,maxPressure = self.max_pression)#,maxPressureVariation = 20)#,valueType=self.value_type)
            elif  act_flag == 1 :
                Cavity.addObject('SurfacePressureConstraint', name='SPC', triangles='@chambreAMesh'+str(i+1)+'.triangles', value=self.init_pressure_value,minPressure = self.min_pression,maxPressure = self.max_pression, valueType=self.value_type)#,maxPressureVariation = 20)#,
            # bellowNode.addObject('SurfacePressureModel', name='SPC_model', minPressure = 0,maxPressure = 20)
            # bellowNode.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./chambreA'+str(i+1) )
            self.i_cavity = self.i_cavity + 1;
            if self.i_cavity == self.nb_cavity :
                self.i_cavity = 0

            Cavity.addObject('AdaptiveBeamMapping', interpolation='@../../BeamInterpolation', input='@../../DOFs', output='@./chambreA'+str(i+1) )

        # Cavity.addObject("QuadSetTopologyContainer", quad = "@../Boite_III_K.quadInROI" )

        # # to print what we get
        # print("UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU \n \n \n")
        # print(copy(Boite_III_K.pointsInROI.value))
        # print(copy(Boite_III_K.quadInROI.value))
        # print(copy(Boite_III_K.quadIndices.value))
        # print(copy(Boite_III_K.edgesInROI.value))
        # print(copy(Boite_III_K.nbIndices.value))
        # print("\n \n \n UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU")

       # #bOX GOOD, but I need 1 box per cavity :'('
       #  module.addObject('BoxROI',name="DISPLAY_boxROI_I_"+str(i+1) , template="Vec3d" ,orientedBox= [Dx, Dy, 0 ,D1x, D1y, 0 ,F1x ,F1y ,0 , self.h_module*2] ,drawBoxes="1" )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
       #  module.addObject('BoxROI',name="DISPLAY_boxROI_II_" +str(i+1), template="Vec3d" ,orientedBox= [Dx, Dy, 0 ,D3x, D3y, 0 ,E3x ,E3y ,0 , self.h_module*2] ,drawBoxes="1" )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
       #  module.addObject('BoxROI',name="DISPLAY_boxROI_III_"+str(i+1) , template="Vec3d" ,orientedBox= [Ex, Ey, 0 ,Fx, Fy, 0 ,F2x ,F2y ,0 , self.h_module*2] ,drawBoxes="1" )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
       #  module.addObject('BoxROI',name="boxROI_I_"+str(i+1) , template="Vec3d" ,orientedBox= [0, Dy, Dx ,0, D1y, D1x ,0 ,F1y ,F1x , self.h_module*2] ,drawBoxes="0",strict=True, drawQuads = True )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
       #  module.addObject('BoxROI',name="boxROI_II_" +str(i+1), template="Vec3d" ,orientedBox= [0, Dy, Dx ,0, D3y, D3x ,0 ,E3y ,E3x , self.h_module*2] ,drawBoxes="1",strict=True )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
       #  module.addObject('BoxROI',name="boxROI_III_"+str(i+1) , template="Vec3d" ,orientedBox= [0, Ey, Ex ,0, Fy, Fx ,0 ,F2y ,F2x , self.h_module*2] ,drawBoxes="1",strict=True )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />

        # #bOX GOOD, but I need 1 box per cavity :'('
        # module.addObject('BoxROI',name="DISPLAY_boxROI_I_"+str(i+1) , template="Vec3d" ,orientedBox= [Dx, Dy, 0 ,D1x, D1y, 0 ,F1x ,F1y ,0 , self.h_module*2] ,drawBoxes="1" )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        # module.addObject('BoxROI',name="DISPLAY_boxROI_II_" +str(i+1), template="Vec3d" ,orientedBox= [Dx, Dy, 0 ,D3x, D3y, 0 ,E3x ,E3y ,0 , self.h_module*2] ,drawBoxes="1" )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        # module.addObject('BoxROI',name="DISPLAY_boxROI_III_"+str(i+1) , template="Vec3d" ,orientedBox= [Ex, Ey, 0 ,Fx, Fy, 0 ,F2x ,F2y ,0 , self.h_module*2] ,drawBoxes="1" )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        # module.addObject('BoxROI',name="boxROI_I_"+str(i+1) , template="Vec3d" ,orientedBox= [0, Dy, Dx ,0, D1y, D1x ,0 ,F1y ,F1x , self.h_module*2] ,drawBoxes="0",strict=True, drawQuads = True )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        # module.addObject('BoxROI',name="boxROI_II_" +str(i+1), template="Vec3d" ,orientedBox= [0, Dy, Dx ,0, D3y, D3x ,0 ,E3y ,E3x , self.h_module*2] ,drawBoxes="1",strict=True )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />
        # module.addObject('BoxROI',name="boxROI_III_"+str(i+1) , template="Vec3d" ,orientedBox= [0, Ey, Ex ,0, Fy, Fx ,0 ,F2y ,F2x , self.h_module*2] ,drawBoxes="1",strict=True )#orientedBox="   8 3 0 9 5.5 0 8 6 0 1" , box="3 3 0 6 6 1", #position="@mecaObj.position" drawTriangles="1" triangles="@Container.triangles" name="boxROI" />

        # # box n'est pas le bon composant de BoxROI pour faire récupérer les points dont j'ai besoin (utiliser plutôt orientedBox) # je garde dans un coin au cas ou
        # module.addObject('BoxROI', name='CavityBox_0'+str(i+1), box=[self.h_module*i-1, Fy, Fx, self.h_module + 1, D1y, D1x], drawBoxes=True, strict=True,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot
        # module.addObject('BoxROI', name='CavityBox_0_For_Display'+str(i+1), box=[Fx, Fy,self.h_module*i-1, D1x, D1y, self.h_module + 1], drawBoxes=True, strict=True,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot
        # module.addObject('BoxROI', name='CavityBox_1'+str(i+1), box=[self.h_module*i-1, Ey, Ex, self.h_module + 1, F2y, F2x], drawBoxes=True, strict=True,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot
        # module.addObject('BoxROI', name='CavityBox_1_For_Display'+str(i+1), box=[Ex, Ey,self.h_module*i-1, F2x, F2y, self.h_module + 1], drawBoxes=True, strict=True,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot
        # module.addObject('BoxROI', name='CavityBox_2'+str(i+1), box=[self.h_module*i-1, Dy, Dx, self.h_module + 1, E3y, E3x], drawBoxes=True, strict=True,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot
        # module.addObject('BoxROI', name='CavityBox_2_For_Display'+str(i+1), box=[Dx, Dy,self.h_module*i-1, E3x, E3y, self.h_module + 1], drawBoxes=True, strict=True,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot


    def createRobot(self,parent,name,out_flag,act_flag):

        print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB")

        for i in range(self.nb_module):

            parent.addObject('MechanicalMatrixMapper',template='Rigid3d,Rigid3d', object1='@DOFs', object2='@DOFs', nodeToParse='@./stiff_flop'+str(i+1))  # je ne projete que dans la structure globale => mechanical object
            
            module_model_path = define_mesh_path(self.module_model,out_flag)
            chamber_model_path = define_mesh_path(self.chamber_model,out_flag)

            name = 'module'

            extension = get_extension(self.module_model)
            if extension == ".obj":
                module = self.createModule_extrude(parent,name,i,module_model_path)
            elif extension == ".vtk" :
                module = self.createModule(parent,name,i,module_model_path)

            if self.force_field == 0 :
                module.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_soft_part) # stable youngModulus = 500 / réel ? = 103
            elif self.force_field == 1 :
                module.addObject('HexahedronFEMForceField' , template='Vec3d', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_soft_part)

            module.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./tetras')# , useCurvAbs = False ) fait planter

            self.createCavityFromFEM(parent = parent, module =  module ,i = i,act_flag=act_flag)

            if self.rigid_bool == 1 :
                ## choisir strict a False ou True = > pour ne prendre que les tetras qui sont entièrement dans la boite 
                # Mettre 8 en fct du rayon du module ?
                module.addObject('BoxROI', name='boxROI_base'+str(i+1), box=[self.h_module*i-1, -8, -8, self.h_module*i+self.rigid_base + 1, 8, 8], drawBoxes=True, strict=True,drawTetrahedra = False) # si autom complète, mettre 8 dépendant des dimensions du robot
                module.addObject('BoxROI', name='boxROI_top'+str(i+1), box=[self.h_module*(i+1)-self.rigid_top, -8, -8, self.h_module*(i+1)+ 1, 8, 8 ], drawBoxes=True, strict=True,drawTetrahedra = False) # utilisé initialement pour rendre rigide une des parois du robot => ici le sommet du module
                print("-- Application de la rigidification des extrémités des modules --")
                modelSubTopo = module.addChild('modelSubTopo')
                if self.force_field == 0 :
                    modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROI_base'+str(i+1)+'.tetrahedraInROI', name='container')
                    modelSubTopo.addObject('TetrahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROI_top'+str(i+1)+'.tetrahedraInROI', name='container')
                    modelSubTopo.addObject('TetrahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_stiff_part)
                elif self.force_field == 1: ## NOT WORKING YET
                    print("000000000 \n \n Rigidification des extrémités des modules impossible avec Hexahèdres pour l'instant => décocher rigid_bool et relancer la scène \n \n")
                    modelSubTopo.addObject('HexahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROI_base'+str(i+1)+'.tetrahedraInROI', name='container')
                    modelSubTopo.addObject('HexahedronSetTopologyContainer', position='@loader.position', tetrahedra='@boxROI_top'+str(i+1)+'.tetrahedraInROI', name='container')
                    modelSubTopo.addObject('HexahedronFEMForceField', template='Vec3', name='FEM', method='large', poissonRatio=self.coef_poi,  youngModulus=self.YM_stiff_part)

            # for j in range(self.nb_cavity):
            #     name = 'Bellow' + str(j+1)
            #     bellowNode1 = self.createCavity(parent,name,i,chamber_model_path,act_flag)
            #     bellowNode1.addObject('AdaptiveBeamMapping', interpolation='@../BeamInterpolation', input='@../DOFs', output='@./chambreA'+str(i+1) )

        return module

