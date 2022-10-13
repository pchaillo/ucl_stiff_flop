# from operator import ge
import Sofa
# from splib3.numerics import Quat
from numpy import *
# from time import clock

# from QuaternionUtils import *
# import statusvars

class CloseLoopController(Sofa.Core.Controller):


    def __init__(self, K_P,K_I,*args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
         
        self.RootNode = kwargs['RootNode']

        # consigne a manipuler
        self.inputMO1 = self.RootNode.DesiredPosition.DesiredPositionM0
        # self.outputMO1 = self.RootNode.goal.goalMO
        self.trackingMO1 = self.RootNode.MeasuredPosition.MeasuredPositionM0

        self.stiffNode = self.RootNode.getChild('goal') # for the generic one
        self.outputMO1 = self.stiffNode.getObject('goalM0')

        self.K_P = K_P # P proportionnal coefficient to avoid over correction
        self.K_I = K_I

        self.first_flag = 0 # just to avoid the 1st time step
        self.outputPos = array(self.outputMO1.position[0])
        print("AAAAAAAA")
        print(self.outputPos)

        self.integrate_error = 0

    def onAnimateBeginEvent(self,dt):

        ########################
        # TIP SENSOR (rotation)
        ########################

        if self.first_flag == 0 :
            self.first_flag = 1
        else :

            inputPos = array(self.inputMO1.position[0])
            trackingPos = array(self.trackingMO1.position[0])
            # outputPos = array(self.outputMO1.position[0])

            print("TTTTTTTTTT")
            # print(outputPos)
            print(trackingPos)
            print(inputPos)

            error = inputPos - trackingPos
            print(error)


            self.integrate_error = self.integrate_error + error
            self.outputPos = self.outputPos + error*self.K_P + self.integrate_error*self.K_I
            print(self.outputPos)

            # outputPos = [outputPos[0],outputPos[1],outputPos[2]]

            # print([outputPos])

            self.outputMO1.position = [self.outputPos]


class CloseLoopController2(Sofa.Core.Controller):


    def __init__(self, K_P,K_I,*args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
         
        self.RootNode = kwargs['RootNode']

        # consigne a manipuler
        self.inputMO1 = self.RootNode.DesiredPosition.DesiredPositionM0
        # self.outputMO1 = self.RootNode.goal.goalMO
        self.trackingMO1 = self.RootNode.MeasuredPosition.MeasuredPositionM0

        self.stiffNode = self.RootNode.getChild('goal') # for the generic one
        self.outputMO1 = self.stiffNode.getObject('goalM0')

        self.K_P = K_P # P proportionnal coefficient to avoid over correction
        self.K_I = K_I

        self.first_flag = 0 # just to avoid the 1st time step
        self.outputPos = array(self.outputMO1.position[0])
        print("AAAAAAAA")
        print(self.outputPos)

        self.integrate_error = 0

    def onAnimateBeginEvent(self,dt):

        ########################
        # TIP SENSOR (rotation)
        ########################

        if self.first_flag == 0 :
            self.first_flag = 1
        else :

            inputPos = array(self.inputMO1.position[0])
            trackingPos = array(self.trackingMO1.position[0])
            # outputPos = array(self.outputMO1.position[0])

            print("TTTTTTTTTT")
            # print(outputPos)
            print(trackingPos)
            print(inputPos)

            error = inputPos - trackingPos
            print(error)


            self.integrate_error = self.integrate_error + error
            self.outputPos = inputPos + error*self.K_P + self.integrate_error*self.K_I
            print(self.outputPos)

            # outputPos = [outputPos[0],outputPos[1],outputPos[2]]

            # print([outputPos])

            self.outputMO1.position = [self.outputPos]


## CODE INITIAL (ALESSANDRINI) ##

# class CloseLoopController(Sofa.Core.Controller):


#     def __init__(self, *args, **kwargs):
#         Sofa.Core.Controller.__init__(self, *args, **kwargs)
         
#         self.RootNode = kwargs['RootNode']

#         # consigne a manipuler
#         self.inputMO1 = self.RootNode.Consigne1.Consigne1
#         self.outputMO1 = self.RootNode.QPTarget1.QPTarget1
#         self.trackingMO1 = self.RootNode.Tracking1.Tracking1
#         self.inputMO2 = self.RootNode.Consigne2.Consigne2
#         self.outputMO2 = self.RootNode.QPTarget2.QPTarget2
#         self.trackingMO2 = self.RootNode.Tracking2.Tracking2



#         self.kp_r1 = 2
#         self.ki_r1 = 2

#         self.kp_r2 = 1
#         self.ki_r2 = 1

#         self.kp_t2x = 0.00
#         self.ki_t2x = 0.8
#         self.kp_t2yz = 0.0
#         self.ki_t2yz = 0.5


#         # verrouillage des rotations (TEMP)
#         # self.kp_r1 = 0
#         # self.ki_r1 = 0
#         # self.kp_r2 = 0
#         # self.ki_r2 = 0


#         self.lastClock = clock()

#         self.q_ki_r1 = Quat([0,0,0,1])	# rotation integree
#         self.q_ki_r2 = Quat([0,0,0,1])	# rotation integree
#         self.t_ki_t2 = [0,0,0] # translation integree

#         return 0

#     def applyDeadZone(self,x,width):
#         if (x>0):
#             if (x<width):
#                 return 0
#             else:
#                 return x-width
#         else:
#             if (x>-width):
#                 return 0
#             else:
#                 return x+width

                


#     def onAnimateBeginEvent(self,dt):

#         if statusvars.close_loop==False:
#             # TODO:
#             if statusvars.actuatorsON==True:
#                 # reset integrators ONLY IF ACTUATORS STILL ON
#                 self.q_ki_r1 = Quat([0,0,0,1])	# rotation integree
#                 self.q_ki_r2 = Quat([0,0,0,1])	# rotation integree
#                 self.t_ki_t2 = [0,0,0] # translation integree
#             return None	        
        

#         c = clock()
#         real_dt = c - self.lastClock
#         self.lastClock = c
#         if (real_dt > 0.05):
#             real_dt = 0.05

#         # print('dt'+str(dt))


#         ########################
#         # TIP SENSOR (rotation)
#         ########################

#         inputPos = array(self.inputMO1.position[0])
#         trackingPos = array(self.trackingMO1.position[0])

#         q_tracking = getQuatFromPos(trackingPos)
#         q_input = getQuatFromPos(inputPos)        
#         q_input = alignQuatX(q_input)	# annulation de la rotation en X
#         applyQuatToPos(q_input,inputPos) # update input position to apply roll reset
#         self.inputMO1.position = [inputPos]

#         # KP
        
#         # ecart position reelle-consigne
#         q_kp = Quat.product( q_input, q_tracking.getConjugate() )
#         axis_angle = q_kp.getAxisAngle()
#         axis_angle[1]*=self.kp_r1
#         q_kp = Quat.createFromAxisAngle(axis_angle[0],axis_angle[1])


#         # KI
        
#         # ecart position reelle-consigne
#         q_slerp = Quat.slerp(q_tracking,q_input,self.ki_r1*real_dt)
#         q = Quat.product( q_slerp, q_tracking.getConjugate() )
        
#         self.q_ki_r1 = Quat.product(self.q_ki_r1,q)

#         # cap q_ki
#         axis_angle = self.q_ki_r1.getAxisAngle()
#         if (axis_angle[1]>1.5):
#             axis_angle[1]=1.5
#             self.q_ki_r1 = Quat.createFromAxisAngle(axis_angle[0],axis_angle[1])

#         self.q_ki_r1 = alignQuatX(self.q_ki_r1)

#         # on applique
#         q_commande = Quat.product(q_kp,self.q_ki_r1)
#         outputPos=[0]*7
#         outputPos[0]=inputPos[0]
#         outputPos[1]=inputPos[1]
#         outputPos[2]=inputPos[2]
#         applyQuatToPos(q_commande,outputPos)
#         self.outputMO1.position = [outputPos]

#         ########################
#         # MIDDLE SENSOR (rotation)
#         ########################

#         inputPos = array(self.inputMO2.position[0])
#         trackingPos = array(self.trackingMO2.position[0])

#         q_tracking = getQuatFromPos(trackingPos)
#         q_input = getQuatFromPos(inputPos)        
#         q_input = alignQuatX(q_input)	# annulation de la rotation en X
#         applyQuatToPos(q_input,inputPos) # update input position to apply roll reset
#         self.inputMO2.position = [inputPos]

#         # KP
        
#         # ecart position reelle-consigne
#         q_kp = Quat.product( q_input, q_tracking.getConjugate() )
#         axis_angle = q_kp.getAxisAngle()
#         axis_angle[1]*=self.kp_r2
#         q_kp = Quat.createFromAxisAngle(axis_angle[0],axis_angle[1])


#         # KI
        
#         # ecart position reelle-consigne
#         q_slerp = Quat.slerp(q_tracking,q_input,self.ki_r2*real_dt)
#         q = Quat.product( q_slerp, q_tracking.getConjugate())
        
#         self.q_ki_r2 = Quat.product(self.q_ki_r2,q)

#         # cap q_ki
#         axis_angle = self.q_ki_r2.getAxisAngle()
#         if (axis_angle[1]>1.5):
#             axis_angle[1]=1.5
#             self.q_ki_r2 = Quat.createFromAxisAngle(axis_angle[0],axis_angle[1])

#         self.q_ki_r2 = alignQuatX(self.q_ki_r2)

#         ########################
#         # MIDDLE SENSOR (translation)
#         ########################

#         inputPos = array(self.inputMO2.position[0])
#         trackingPos = array(self.trackingMO2.position[0])
        
#         # ecart position reelle-consigne
#         epsilon = [inputPos[0]-trackingPos[0],inputPos[1]-trackingPos[1],inputPos[2]-trackingPos[2]]

#         # dead zone sur epsilon !
#         epsilon[0] = self.applyDeadZone(epsilon[0],5)

#         # KP
#         t_kp = [epsilon[0]*self.kp_t2x,epsilon[1]*self.kp_t2yz,epsilon[2]*self.kp_t2yz]

#         # KI
#         self.t_ki_t2[0]+=epsilon[0]*self.ki_t2x*real_dt
#         self.t_ki_t2[1]+=epsilon[1]*self.ki_t2yz*real_dt
#         self.t_ki_t2[2]+=epsilon[2]*self.ki_t2yz*real_dt

#         # cap q_ki
#         # axis_angle = Quat.quatToAxis(self.q_ki_r2)
#         # if (axis_angle[1]>1.5):
#         # 	axis_angle[1]=1.5
#         # 	self.q_ki_r2 = Quat.axisToQuat(axis_angle[0],axis_angle[1])

#         # self.q_ki_r2 = alignQuatX(self.q_ki_r2)

#         # on applique
#         q_commande = Quat.product(q_kp,self.q_ki_r2)
#         outputPos=[0]*7
#         outputPos[0] = inputPos[0]+t_kp[0]+self.t_ki_t2[0]
#         outputPos[1] = inputPos[1]+t_kp[1]+self.t_ki_t2[1]
#         outputPos[2] = inputPos[2]+t_kp[2]+self.t_ki_t2[2]
#         applyQuatToPos(q_commande,outputPos)
#         self.outputMO2.position = [outputPos]

#         return 0