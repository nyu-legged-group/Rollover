import os
import numpy as np
from numpy.linalg import norm, inv, pinv, eig, svd
import subprocess
import threading
import time
import matplotlib.pyplot as plt
from scipy.optimize import fmin_slsqp

import pybullet as p
import pybullet_data
import pinocchio as se3
from pinocchio.utils import *
from pinocchio.rpy import matrixToRpy, rpyToMatrix, rotate
import cv2
from PIL import Image

from py_pinocchio_bullet.wrapper import PinBulletWrapper
from robot_properties_solo.config import SoloConfig

from os.path import join, dirname
from utils import plotter

vec2list = lambda m: np.array(m.T).reshape(-1).tolist()
# np.set_printoptions(precision=5, suppress=True)

class Humanoid(PinBulletWrapper):
    def __init__(self, physicsClient=None, display=False,timeStep=1e-3, rendering=0, view='side'):
        if physicsClient is None:
            if display:
                self.physicsClient = p.connect(p.GUI)
            else:
                self.physicsClient = p.connect(p.DIRECT)
            # Setup for fall simulation, while setup walking, do turn this off!
            p.setGravity(0, 0, -9.81)
            # p.setGravity(0,0,0)
            p.setPhysicsEngineParameter(
                fixedTimeStep=timeStep, numSubSteps=1)

            # Load the plain.

            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            plain_urdf = (SoloConfig.packPath +
                      "/urdf/plane_with_restitution.urdf")
            self.planeId = p.loadURDF(plain_urdf)


            robotStartPos = [0., 0, 0.0014875+0.5095826451096808]
            # robotStartPos = vec2list(x_input)
            # robotStartPos[2] += 0.0014875
            robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

            # Use real time simulation
            self.useRealTimeSim = 0

            # Use rendering tool for real time video output
            self.rendering = rendering
            self.urdf_path_pybullet = SoloConfig.urdf_path_pybullet
            self.pack_path = SoloConfig.packPath
            self.urdf_path = SoloConfig.urdf_path
            self.robotId = p.loadURDF(self.urdf_path_pybullet, robotStartPos,
                robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE,
                useFixedBase=False)
            p.getBasePositionAndOrientation(self.robotId)

            # Create the robot wrapper in pinocchio.
            package_dirs = [os.path.dirname(
                os.path.dirname(self.urdf_path)) + '/urdf']
            self.pin_robot = SoloConfig.buildRobotWrapper()

            # Query all the joints.
            num_joints = p.getNumJoints(self.robotId)

            for ji in range(num_joints):
                p.changeDynamics(self.robotId, ji, linearDamping=0.,
                    angularDamping=0., restitution=0., lateralFriction=0.7)

            self.base_link_name = "base_link"
            self.joint_names = ['j_l_hip_y', 'j_l_hip_r', 'j_l_hip_p', 'j_l_knee', 'j_l_ankle_p',
            'j_l_ankle_r','j_l_shoulder_p', 'j_l_shoulder_r', 'j_l_shoulder_y', 'j_l_elbow', 'j_r_hip_y', 'j_r_hip_r', 'j_r_hip_p', 'j_r_knee', 'j_r_ankle_p',
            'j_r_ankle_r', 'j_r_shoulder_p', 'j_r_shoulder_r', 'j_r_shoulder_y',
            'j_r_elbow']
            controlled_joints = ['j_l_hip_y', 'j_l_hip_r', 'j_l_hip_p', 'j_l_knee', 'j_l_ankle_p',
            'j_l_ankle_r','j_l_shoulder_p', 'j_l_shoulder_r', 'j_l_shoulder_y', 'j_l_elbow', 'j_r_hip_y', 'j_r_hip_r', 'j_r_hip_p', 'j_r_knee', 'j_r_ankle_p',
            'j_r_ankle_r', 'j_r_shoulder_p', 'j_r_shoulder_r', 'j_r_shoulder_y',
            'j_r_elbow']

            # Creates the wrapper by calling the super.__init__.
            super(Humanoid, self).__init__(self.robotId, self.pin_robot,
                controlled_joints,
                ['j_l_hand', 'j_r_hand'], useTorqueCtrl = False
            )

            # Adjust view, close unrelevant window and rendering parameter

            # region
            resultPath = str(os.path.abspath(os.path.join(self.pack_path, '../humanoid_simulation/result')))
            if self.useRealTimeSim == 1:
                p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, os.path.join(resultPath, 'Humanoid_log.mp4'))
                p.setRealTimeSimulation(self.useRealTimeSim)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
            front_view = [0.05,90,-30,[0.8,0,0.45]]
            side_view = [0.1,150,-30,[0.45,0.45,0.45]]
            top_view = [0.4,-90,-89.9,[0.15,0,0.4]]
            left_view = [0.65, 179.9, -20, [0.15,0,0.4]]
            
            def front():
                p.resetDebugVisualizerCamera(cameraDistance=front_view[0],
                cameraYaw=front_view[1],
                cameraPitch=front_view[2],
                cameraTargetPosition=front_view[3])
            
            def side():
                p.resetDebugVisualizerCamera(cameraDistance=side_view[0],
                cameraYaw=side_view[1],
                cameraPitch=side_view[2],
                cameraTargetPosition=side_view[3])
                self.camTargetPos = side_view[3]
                self.yaw = side_view[1]
                self.pitch = side_view[2]+15
                self.roll = 0
                self.camDistance = side_view[0]
            
            def top():
                p.resetDebugVisualizerCamera(cameraDistance=top_view[0],
                cameraYaw=top_view[1],
                cameraPitch=top_view[2],
                cameraTargetPosition=top_view[3])
            def left():
                p.resetDebugVisualizerCamera(cameraDistance=left_view[0],
                cameraYaw=left_view[1],
                cameraPitch=left_view[2],
                cameraTargetPosition=left_view[3])
                self.camTargetPos = side_view[3]
                self.yaw = side_view[1]
                self.pitch = side_view[2]+15
                self.roll = 0
                self.camDistance = side_view[0]
            options = {'front' : front,
                'side' : side,
                'top' : top,
                'left': left,
            }
            options[view]()

            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
            p.configureDebugVisualizer(
                p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

            
 
            self.cameraUp = [0, 0, 1]
            self.cameraPos = [1, 1, 1]

            self.upAxisIndex = 2
            
            self.pixelWidth = 800
            self.pixelHeight = 800
            self.nearPlane = 0.1
            self.farPlane = 100.0
            self.fov = 60
            self.aspect = float(self.pixelWidth) / self.pixelHeight
            self.viewMatrix = p.computeViewMatrixFromYawPitchRoll(self.camTargetPos, self.camDistance, self.yaw, self.pitch,
                                                            self.roll, self.upAxisIndex)
            self.projectionMatrix = p.computeProjectionMatrixFOV(
                self.fov, self.aspect, self.nearPlane, self.farPlane)
            self.size = (self.pixelWidth, self.pixelHeight)
            if self.rendering == 1:
                
                self.camTargetPos = [0.9, 0.9, 0.45]
                self.cameraUp = [0, 0, 1]
                self.cameraPos = [1, 1, 1]
                self.yaw = 160.0
                self.pitch = 0.0
                self.roll = 0
                self.upAxisIndex = 2
                self.camDistance = 0.1
                self.pixelWidth = 1366
                self.pixelHeight = 768
                self.nearPlane = 0.1
                self.farPlane = 100.0
                self.fov = 60
                self.aspect = float(self.pixelWidth) / self.pixelHeight
                self.viewMatrix = p.computeViewMatrixFromYawPitchRoll(self.camTargetPos, self.camDistance, self.yaw, self.pitch,
                                                                self.roll, self.upAxisIndex)
                self.projectionMatrix = p.computeProjectionMatrixFOV(
                    self.fov, self.aspect, self.nearPlane, self.farPlane)
                self.size = (self.pixelWidth, self.pixelHeight)
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                
                self.out = cv2.VideoWriter(os.path.join(resultPath, 'Humanoid_realtime.mp4'), fourcc, 100.0, self.size)
            # endregion

class Simulator(object):
    '''
    Simulator Abstract
    '''
    def __init__(self, horizon_length,timeStep=1e-3, display=False):
        self.horizon_length = horizon_length
        self.timeStep = timeStep
        self.display = display
    def loadModel(self):
        raise NotImplementedError

    def initPose(self, x0, nq, nv, na):
        '''
        Generate trajectories and initialize robot.
        '''
        # Get current state and initialize
        q, dq = self.env.get_state()
        # Update the simulation state to the new initial configuration.
        _x0 = x0.copy()
        q = _x0[:nq]
        dq = _x0[nq:]
        self.x0 = _x0
        self.env.reset_state(q, dq)

class HumanoidSimulator(Simulator):
    def __init__(self, horizon_length, timeStep=1e-3, display=False, view='side'):
        super(HumanoidSimulator, self).__init__(horizon_length, timeStep, display)

        self.view = view
        self.env = self.loadModel()
        
        '''
        This is a simulator designed for a humanoid project
        '''

    def loadModel(self):
        '''
        Load robot model
        '''
        # print(self.view)
        # Setup pybullet for the quadruped and a wrapper to pinocchio.
        env = Humanoid(display=self.display,rendering = 0, view=self.view)
        env.rendering = 0
        env.useTorqueCtrl = False

        return env
    
    def simulate(self, model, kneeTraj, tauTraj):
        '''
        Simulate robot model
        '''
        forceArr = np.zeros([1, self.horizon_length])
        comArr = np.zeros([1,self.horizon_length])
        posArr = np.zeros([20, self.horizon_length])
        peArr = np.zeros([1, self.horizon_length])
        keArr = np.zeros([1, self.horizon_length])

        q, dq = self.env.get_state()

        # The initial state of optimal control problem: the state @the tiem after falling
        initialFallState =  np.concatenate([q.copy(), dq.copy()])

        # The initial state of optimal control problem: the state @the tiem before hand contact
        initialContactState =  np.concatenate([q.copy(), dq.copy()])
    
        visualizer = model
        forceKnot =0
        forceAction = False
        fallAction = False
        initCom =0
         
        isFallTiming = False
        startFallTiming = 0
        tau = q[7:].copy()

        for i in range(self.horizon_length):
            
            # Get the current state
            q, dq = self.env.get_state()
            se3.computePotentialEnergy(self.env.pinocchio_robot.model, self.env.pinocchio_robot.data, q)
            se3.computeKineticEnergy(self.env.pinocchio_robot.model, self.env.pinocchio_robot.data, q, dq)
            peArr[0,i] = self.env.pinocchio_robot.data.potential_energy
            keArr[0,i] = self.env.pinocchio_robot.data.kinetic_energy

            # Get the current force
            active_contact_frames, contact_forces, contact_cop,contact_cop_force, contact_cop_ft_l, contact_cop_ft_r = self.env.get_force()
            active_contact_frames_link, contact_forces_link = self.env.get_force_link()
            forceArr[0, i] = contact_cop_force

            q, dq = self.env.get_state_update_pinocchio()
            posArr[:,i] = vec2list(q[7:])
            
            se3.centerOfMass(self.env.pinocchio_robot.model,self.env.pinocchio_robot.data, q, dq)
            com = self.env.pinocchio_robot.data.com[0]
            comZ = vec2list(com)[2]
            comArr[0, i] = comZ

            ################################################Controller Kernel#######################################################

            if i ==0:
                initCom = comZ
            if comZ > initCom -0.005:
                if not fallAction:
                    tau[3] = tau[13] = kneeTraj[3, i].copy()
                    initialFallState =  np.concatenate([q.copy(), dq.copy()])
                    
                    initialFallHandPos = self.env.pinocchio_robot.data.oMf[visualizer.rhId].translation.copy()
                    # print(visualizer.rhId)
                    # print('initialFallHandPos', initialFallHandPos)
                    initialCoM = com.copy()
                    initialCoMv= self.env.pinocchio_robot.data.vcom[0]
                    initialKneeAngle = kneeTraj[3, i].copy()
            else:
                fallAction = True
                if not isFallTiming:
                    # exit()
                    isFallTiming = True
                    startFallTiming = i
                    print('CoM is pulled out of support polygon @ %d'%startFallTiming)
                    
            # print(contact_cop_force)
            if contact_cop_force >0:
                if not forceAction:
                    # print(contact_cop_force)
                    # print(forceAction)
                    forceAction = True
                    forceKnot = i
                    endContactHandPos = self.env.pinocchio_robot.data.oMf[visualizer.rhId].translation.copy()
                    endFallHandPos = self.env.pinocchio_robot.data.oMf[visualizer.rhId].translation.copy()
                    # print('endFallHandPos:',endFallHandPos)
                    endCoM = com.copy()

            if forceAction and i-forceKnot < np.shape(tauTraj)[1]:
                j = i-forceKnot
                # tau[6:9] = np.matrix(tauTraj[6:9,j]).T
                # tau[15:18] = np.matrix(tauTraj[15:18,j]).T
                # tau = tauTraj[:,j]
                
            
            if not forceAction:
                initialContactState =  np.concatenate([q.copy(), dq.copy()])
                initialContactHandPos = self.env.pinocchio_robot.data.oMf[visualizer.rhId].translation.copy()
                

            ################################################Controller Kernel #######################################################
            self.env.send_joint_command(tau)

            # Step the simulator and sleep.
            if (self.env.useRealTimeSim == 0):
                p.stepSimulation()
                if self.display:
                    time.sleep(self.timeStep)

        #     # Rendering
        # region
        #     if self.env.rendering == 1:
        #         img_arr = p.getCameraImage(self.env.pixelWidth,
        #                                     self.env.pixelHeight,
        #                                     self.env.viewMatrix,
        #                                     self.env.projectionMatrix,
        #                                     shadow=1,
        #                                     lightDirection=[1, 1, 1],
        #                                     renderer=p.ER_BULLET_HARDWARE_OPENGL)
        #         proj_opengl = np.uint8(np.reshape(
        #             img_arr[2], (self.env.pixelHeight, self.env.pixelWidth, 4)))

        #         # frame = cv2.resize(proj_opengl,(env.pixelWidth,env.pixelHeight))
        #         frame = cv2.cvtColor(proj_opengl, cv2.COLOR_RGB2BGR)

        #         self.env.out.write(frame)
        #         if (cv2.waitKey(1) & 0xFF) == ord('q'):  # Hit `q` to exit
        #             break

        # if self.env.rendering == 1:
        #     self.env.out.release()
        #     cv2.destroyAllWindows()
        # endregion
        
        # handContactLength: the offset vector between hand point of initial state before hand contact and contact on the ground
        handContactLength = endContactHandPos-initialContactHandPos
        # handFallLength: the offset vector between hand point of initial state after initializing fall and contact on the ground
        # print('initialFallHandPos', initialFallHandPos)
        # print('endFallHandPos:',endFallHandPos)
        handFallLength = endFallHandPos-initialFallHandPos
        
        # comLength: the offset vector between com of initial state and final pose
        comLength = endCoM - initialCoM

        # Timing between initiating fall and hand contact
        timeLength = forceKnot - startFallTiming

        p.disconnect()
        return forceArr, comArr, posArr, initialContactState, handContactLength, comLength, initialFallState, handFallLength, timeLength, initialCoM,initialCoMv, initialKneeAngle, peArr, keArr
    
    def simulateOptTraj(self, model, kneeTraj, tauTraj, ctrlTimeStep = 1e-2, speed=1., isBimanual=False, videoPath=None, videoName=None):
        '''
        Simulate an optimal trajectory
        '''
        
        forceArr = np.zeros([1, self.horizon_length])
        comArr = np.zeros([3,self.horizon_length])
        posArr = np.zeros([20, self.horizon_length])
        torqueArr = np.zeros([20, self.horizon_length])
        qArr = np.zeros([26, self.horizon_length]) # convert quaternion to rpy
        velArr = np.zeros([20, self.horizon_length])
        tauArr = np.zeros([20, self.horizon_length])
        peArr = np.zeros([1, self.horizon_length])
        keArr = np.zeros([1, self.horizon_length])
        
        q, dq = self.env.get_state()

        forcePose = q.copy()

        # The initial state of optimal control problem: the state @the tiem after falling
        initialFallState =  np.concatenate([q.copy(), dq.copy()])

        # The initial state of optimal control problem: the state @the tiem before hand contact
        initialContactState =  np.concatenate([q.copy(), dq.copy()])
    
        visualizer = model
        forceKnot =0
        forceAction = False
        fallAction = False
        initCom =0
        isFallTiming = False
        contactTime = 0
        tau = q[7:].copy()
        kneeBuffer = 0.
        coefficient = 0
        # draw simulation space
        if not isBimanual:
            pts = [[-0.25,-0.15,0],
                    [0.045,-0.15,0],
                    [0.045,0.15,0],
                    [-0.25,0.15,0]]
            pts_l = [[0.45,-0.172,0],
                    [0.516,-0.172,0],
                    [0.516,-0.106,0],
                    [0.45,-0.106,0]]
            pts_r = [[0.45,0.106,0],
                    [0.516,0.106,0],
                    [0.516,0.172,0],
                    [0.45,0.172,0]]
        else:
            # draw new baseline for bimanual part due to position error
            pts = [[-0.25,-0.15,0],
                    [0.045,-0.15,0],
                    [0.045,0.15,0],
                    [-0.25,0.15,0]]
            pts_l = [[0.45,-0.134,0],
                    [0.516,-0.134,0],
                    [0.516,-0.067,0],
                    [0.45,-0.067,0]]
            pts_r = [[0.45,0.067,0],
                    [0.516,0.067,0],
                    [0.516,0.134,0],
                    [0.45,0.134,0]]
        # p.addUserDebugLine(pts[0],pts[1], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        # p.addUserDebugLine(pts[1],pts[2], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        # p.addUserDebugLine(pts[2],pts[3], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        # p.addUserDebugLine(pts[3],pts[0], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)

        # p.addUserDebugLine(pts_l[0],pts_l[1], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        # p.addUserDebugLine(pts_l[1],pts_l[2], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        # p.addUserDebugLine(pts_l[2],pts_l[3], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        # p.addUserDebugLine(pts_l[3],pts_l[0], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)

        # p.addUserDebugLine(pts_r[0],pts_r[1], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        # p.addUserDebugLine(pts_r[1],pts_r[2], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        # p.addUserDebugLine(pts_r[2],pts_r[3], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        # p.addUserDebugLine(pts_r[3],pts_r[0], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)

        def start_recording(file_name):
            p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, file_name)
        def stop_recording(file_name):
            p.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4)
        if videoPath is not None:
            if videoName is None:
                file_name = videoPath + time.strftime("%m_%d_%H_%M_%S") + '.mp4'
            else:
                file_name = videoPath + videoName + '.mp4'
            start_recording(file_name)
        for i in range(self.horizon_length):
            #print('current time %f s'%(i*0.001))
            # Get the current state
            q, dq = self.env.get_state()
            se3.computePotentialEnergy(self.env.pinocchio_robot.model, self.env.pinocchio_robot.data, q)
            se3.computeKineticEnergy(self.env.pinocchio_robot.model, self.env.pinocchio_robot.data, q, dq)
            peArr[0,i] = self.env.pinocchio_robot.data.potential_energy
            keArr[0,i] = self.env.pinocchio_robot.data.kinetic_energy
            
            # Get the current force
            active_contact_frames, contact_forces, contact_cop,contact_cop_force, contact_cop_ft_l, contact_cop_ft_r = self.env.get_force()
            active_contact_frames_link, contact_forces_link = self.env.get_force_link()
            forceArr[0, i] = contact_cop_force

            # Get the current postion and applied torque
            q, dq = self.env.get_state_update_pinocchio()
            

            posArr[:,i] = vec2list(q[7:])

            
            _se3 = se3.XYZQUATToSE3(q[:7])
            rpy_vector = matrixToRpy(_se3.rotation)

            qArr[:3,i] = vec2list(q[:3])
            qArr[3:6,i] = vec2list(rpy_vector*180./np.pi)
            qArr[6:,i] = vec2list(q[7:])
            

            velArr[:,i] = vec2list(dq[6:])
            
            com = se3.centerOfMass(self.env.pinocchio_robot.model,self.env.pinocchio_robot.data, q)
            comZ = vec2list(com)[2]
            com = vec2list(com)
            comArr[:, i] = com
            


            ################################################Controller Kernel#######################################################

            if i ==0:
                initCom = comZ
            '''
            This parameter is used to tune the delay between experiment and simulation!
            This parameter should be changed from 0.005 to 0.003 before deployment in the experiment
            '''
            if comZ > initCom -0.005:
            # if comZ > initCom -0.0025:
                if not fallAction:
                    tau[3] = tau[13] = kneeTraj[3, i].copy()
                    kneeBuffer = tau[3]

            else:
                fallAction = True
                if not isFallTiming:
                    isFallTiming = True
                    forceKnot = i
                    print('action knot @ %d, action time: %.2f ms' % (i, i*self.timeStep*1000.))

            if contact_cop_force >0:
                if not forceAction:
                    forceAction = True
                    forcePose = q.copy()
                    contactTime = i
                    print('contact knot @ %d, contact time: %.2f ms'%(i, i*self.timeStep*1000.))
                    # print('com position: [ %.2f, %.2f, %.2f ]'%(com[0], com[1], com[2]))
                    rkId = self.env.pinocchio_robot.model.getFrameId('r_knee_rp')
                    rkPos = self.env.pinocchio_robot.data.oMf[rkId].translation
                    # print('knee contact point: [ %.2f, %.2f, %.2f ]'%(rkPos[0], rkPos[1], rkPos[2]))
                    # print('hand contact point: [ %.2f, %.2f, %.2f ]'%(contact_cop[0], contact_cop[1], contact_cop[2]))
                    x_arr = [ rkPos[0], contact_cop[0]]
                    y_arr = [-1*rkPos[1], -1*contact_cop[1]]
                    
                    # plt.scatter(-1*com[1], com[0])
                    # plt.plot(y_arr, x_arr, '-')
                    # plt.axis('scaled')
                    # exit

                 

            if isFallTiming and (i-forceKnot)//int(ctrlTimeStep/self.timeStep) < np.shape(tauTraj)[1]:
                j = (i-forceKnot)//int(ctrlTimeStep/self.timeStep)
                # print(j)
                # tau[6:9] = np.matrix(tauTraj[6:9,j]).T
                # tau[15:18] = np.matrix(tauTraj[15:18,j]).T
                tau = tauTraj[:,j].copy()
                tau[3] = tau[13] = kneeBuffer
                # print(tau[18])
            

            ################################################Controller Kernel #######################################################
            tauArr[:,i] = vec2list(tau)
            self.env.send_joint_command(tau)

            # Step the simulator and sleep.
            if (self.env.useRealTimeSim == 0):
                p.stepSimulation()

                # textColor = [1, 1, 1]
                # shift = 0.05
                # p.addUserDebugText("explicit PD", [shift, 0, .1],
                #                     textColor,
                #                     parentObjectUniqueId=self.env.robotId,
                #                     parentLinkIndex=1)
                # if self.display:
                #     if isFallTiming:
                time.sleep(self.timeStep*speed)
            torque = self.env.get_state_torque()
            torqueArr[:,i] = vec2list(torque)
            # Rendering
            # region
            if self.env.rendering == 1:
                img_arr = p.getCameraImage(self.env.pixelWidth,
                                            self.env.pixelHeight,
                                            self.env.viewMatrix,
                                            self.env.projectionMatrix,
                                            shadow=1,
                                            lightDirection=[1, 1, 1],
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)
                proj_opengl = np.uint8(np.reshape(
                    img_arr[2], (self.env.pixelHeight, self.env.pixelWidth, 4)))

                # frame = cv2.resize(proj_opengl,(env.pixelWidth,env.pixelHeight))
                frame = cv2.cvtColor(proj_opengl, cv2.COLOR_RGB2BGR)

                self.env.out.write(frame)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):  # Hit `q` to exit
                    break

        if self.env.rendering == 1:
            self.env.out.release()
            cv2.destroyAllWindows()
        # endregion
        if videoPath is not None:
            stop_recording(file_name)
        p.disconnect()
        # plt.title('max force: %.2f N'%np.amax(forceArr))
        return forceArr, comArr, posArr, velArr, torqueArr, qArr, forcePose, tauArr, peArr, keArr, contactTime, forceKnot
    # def drawSquare(self, center=[0,0,0], l=0.01):
    #     # draw a small square with debug tool
    #     # l: side length of square
    #     # z direction is zero
    #     corner_1 = center+[0.5*l,0.5*l,0]
    #     corner_2 = center+[-0.5*l, 0.5*l,0]
    #     corner_3 = center+[-0.5*l, -0.5*l,0]
    #     corner_4 = center+[0.5*l, -0.5*l,0]
    #     p.addUserDebugLine(pts[0],pts[1], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)


    def replayEncoder(self, model, ENCODER_DIR, ctrlTimeStep = 3e-3, speed=1., DIR=None):
        '''
        Simulate an optimal trajectory
        '''
        tauTraj = np.loadtxt(open(ENCODER_DIR, "rb"), delimiter=",")
        
        forceArr = np.zeros([1, self.horizon_length])
        comArr = np.zeros([3,self.horizon_length])
        posArr = np.zeros([20, self.horizon_length])
        torqueArr = np.zeros([20, self.horizon_length])
        qArr = np.zeros([27, self.horizon_length])
        velArr = np.zeros([20, self.horizon_length])
        tauArr = np.zeros([20, self.horizon_length])
        peArr = np.zeros([1, self.horizon_length])
        keArr = np.zeros([1, self.horizon_length])   
        motorEArr = np.zeros([1, self.horizon_length])
        q, dq = self.env.get_state()

        forcePose = q.copy()

        # The initial state of optimal control problem: the state @the tiem after falling
        initialFallState =  np.concatenate([q.copy(), dq.copy()])

        # The initial state of optimal control problem: the state @the tiem before hand contact
        initialContactState =  np.concatenate([q.copy(), dq.copy()])
    
        visualizer = model
        forceKnot =0
        forceAction = False
        fallAction = False
        initCom =0
        isFallTiming = False
        tau = q[7:].copy()
        kneeBuffer = 0.
        coefficient = 0
        contactTime = 0
        motorE = 0.
        # region
        '''
        # draw simulation space
        pts = [[-0.25,-0.15,0],
				 [0.045,-0.15,0],
				 [0.045,0.15,0],
				 [-0.25,0.15,0]]
        pts_l = [[0.45,-0.172,0],
				 [0.516,-0.172,0],
				 [0.516,-0.106,0],
				 [0.45,-0.106,0]]
        pts_r = [[0.45,0.106,0],
				 [0.516,0.106,0],
				 [0.516,0.172,0],
				 [0.45,0.172,0]]
        p.addUserDebugLine(pts[0],pts[1], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        p.addUserDebugLine(pts[1],pts[2], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        p.addUserDebugLine(pts[2],pts[3], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        p.addUserDebugLine(pts[3],pts[0], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)

        p.addUserDebugLine(pts_l[0],pts_l[1], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        p.addUserDebugLine(pts_l[1],pts_l[2], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        p.addUserDebugLine(pts_l[2],pts_l[3], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        p.addUserDebugLine(pts_l[3],pts_l[0], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)

        p.addUserDebugLine(pts_r[0],pts_r[1], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        p.addUserDebugLine(pts_r[1],pts_r[2], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        p.addUserDebugLine(pts_r[2],pts_r[3], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        p.addUserDebugLine(pts_r[3],pts_r[0], lineColorRGB=[0, 0, 1], lineWidth=5.0, lifeTime=0)
        '''
        # endregion
        for i in range(self.horizon_length):
            #print('current time %f s'%(i*0.001))
            # Get the current state
            q, dq = self.env.get_state()

            se3.computePotentialEnergy(self.env.pinocchio_robot.model, self.env.pinocchio_robot.data, q)
            se3.computeKineticEnergy(self.env.pinocchio_robot.model, self.env.pinocchio_robot.data, q, dq)
            peArr[0,i] = self.env.pinocchio_robot.data.potential_energy
            keArr[0,i] = self.env.pinocchio_robot.data.kinetic_energy

            # Get the current force
            active_contact_frames, contact_forces, contact_cop,contact_cop_force, contact_cop_ft_l, contact_cop_ft_r = self.env.get_force()
            active_contact_frames_link, contact_forces_link = self.env.get_force_link()
            forceArr[0, i] = contact_cop_force

            # Get the current postion and applied torque
            q, dq = self.env.get_state_update_pinocchio()
            
            posArr[:,i] = vec2list(q[7:])
            velArr[:,i] = vec2list(dq[6:])
            qArr[:,i] = vec2list(q)
            
            
            com = se3.centerOfMass(self.env.pinocchio_robot.model,self.env.pinocchio_robot.data, q)
            comZ = vec2list(com)[2]
            com = vec2list(com)
            comArr[:, i] = com

            ################################################Controller Kernel#######################################################

            # if i ==0:
            #     initCom = comZ
            # if comZ > initCom -0.005:
            #     if not fallAction:
            #         #tau[3] = tau[13] = kneeTraj[3, i].copy()
            #         #kneeBuffer = tau[3]

            # else:
            #     fallAction = True
            #     if not isFallTiming:
            #         isFallTiming = True
            #         forceKnot = i

            if contact_cop_force >0:
                if not forceAction:
                    forceAction = True
                    # forcePose = q.copy()
                    print('contact knot @ %d'%i)
                    contactTime = i
                    

            #if isFallTiming and (i-forceKnot)//int(ctrlTimeStep/self.timeStep) < np.shape(tauTraj)[1]:
            
            j = i//int(ctrlTimeStep/self.timeStep)
            if j < np.shape(tauTraj)[1]:
                # print(j)
                # tau[6:9] = np.matrix(tauTraj[6:9,j]).T
                # tau[15:18] = np.matrix(tauTraj[15:18,j]).T
                tau = tauTraj[:,j].copy()
                # tau[3] = tau[13] = kneeBuffer
                    # print(tau[18])
            

            ################################################Controller Kernel #######################################################
            tauArr[:,i] = vec2list(tau)
            self.env.send_joint_command(tau)

            # Step the simulator and sleep.
            if (self.env.useRealTimeSim == 0):
                p.stepSimulation()
                # textColor = [1, 1, 1]
                # shift = 0.05
                # p.addUserDebugText("explicit PD", [shift, 0, .1],
                #                     textColor,
                #                     parentObjectUniqueId=self.env.robotId,
                #                     parentLinkIndex=1)
                if self.display:
                    time.sleep(self.timeStep*speed)
            torque = self.env.get_state_torque()
            torqueArr[:,i] = vec2list(torque)
            _motorE = 0.
            for j in range(20):
                
                _motorE += np.abs(torqueArr[j,i]*dq[j+6])*self.timeStep
            
            if forceAction:
                print(_motorE)
                print(motorE)
            motorE += _motorE
            motorEArr[0,i] = motorE
            '''
            if i in {1200, 1500, 1800, 2031, 2600, 3200}:
                img_arr = p.getCameraImage(self.env.pixelWidth,
                                self.env.pixelHeight,
                                self.env.viewMatrix,
                                self.env.projectionMatrix,
                                shadow=1,
                                lightDirection=[1, 1, 1],
                                renderer=p.ER_BULLET_HARDWARE_OPENGL)
                proj_opengl = np.uint8(np.reshape(
                    img_arr[2], (self.env.pixelHeight, self.env.pixelWidth, 4)))

                # frame = cv2.resize(proj_opengl,(env.pixelWidth,env.pixelHeight))
                frame = cv2.cvtColor(proj_opengl, cv2.COLOR_RGB2BGR)
                font                   = cv2.FONT_HERSHEY_SIMPLEX
                bottomLeftCornerOfText = (10,750)
                fontScale              = 1
                fontColor              = (255,255,255)
                lineType               = 2

                cv2.putText(frame,"%.2fs"%(i*0.001), 
                    bottomLeftCornerOfText, 
                    font, 
                    fontScale,
                    fontColor,
                    lineType)
                name = DIR+"frame%d.png"%i
                RGBimage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                PILimage = Image.fromarray(RGBimage)
                PILimage.save(name, dpi=(300,300))
                # cv2.imwrite(name, frame, dpi=(300,300))     # save frame as JPEG file
            '''
            # Rendering
            # region
            if self.env.rendering == 1:
                img_arr = p.getCameraImage(self.env.pixelWidth,
                                            self.env.pixelHeight,
                                            self.env.viewMatrix,
                                            self.env.projectionMatrix,
                                            shadow=1,
                                            lightDirection=[1, 1, 1],
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL)
                proj_opengl = np.uint8(np.reshape(
                    img_arr[2], (self.env.pixelHeight, self.env.pixelWidth, 4)))

                # frame = cv2.resize(proj_opengl,(env.pixelWidth,env.pixelHeight))
                frame = cv2.cvtColor(proj_opengl, cv2.COLOR_RGB2BGR)

                self.env.out.write(frame)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):  # Hit `q` to exit
                    break

        if self.env.rendering == 1:
            self.env.out.release()
            cv2.destroyAllWindows()
        # endregion
        
        p.disconnect()
        return forceArr, comArr, posArr, torqueArr, qArr, forcePose, tauArr, peArr, keArr, contactTime, motorEArr, velArr

    def checkLinkState(self):
        stat = p.getLinkState(self.env.robotId, self.env.bullet_endeff_ids[0])
        pos = stat[0]
        print('Simulator: Initial Feet Height:', pos)

    def plot(self,forceArr, comArr, posInfo, simVelInfo, ocVelInfo, torqueInfo,savePath, time_step, cmpInfo=None):
        '''
        Plot simulation result
        posInfo: plot joint position
        torqueInfo: plot torque 
        '''
        

        jointNames=['l_hip_y', 'l_hip_r', 'l_hip_p',\
                    'l_knee', 'l_ankle_p', 'l_ankle_r',\
                    'l_shoulder_p', 'l_shoulder_r', 'l_shoulder_y','l_elbow',\
                    'r_hip_y', 'r_hip_r', 'r_hip_p',\
                    'r_knee', 'r_ankle_p', 'r_ankle_r',\
                    'r_shoulder_p', 'r_shoulder_r', 'r_shoulder_y', 'r_elbow']
        jointBulletNames = [i+'_out' for i in jointNames]
        jointSolNames = [i+'_cmd' for i in jointNames]

        drawer1 = plotter(forceArr, 
                timeStep = time_step, 
                xlabel='t [s]', 
                ylabel='Contact force [N]', 
                fileName=savePath+'force.png')
        drawer1.timePlot()
        drawer2 = plotter(comArr, 
                        timeStep = time_step,
                        legend=['X', 'Y', 'Z'], 
                        xlabel='t [s]', 
                        ylabel='CoM position [m]', 
                        fileName=savePath+'com.png')
        drawer2.timePlot()
        drawer3 = plotter(posInfo[0], 
                        timeStep = time_step,
                        legend=jointNames, 
                        xlabel='t [s]', 
                        ylabel='Joint position [Radian]', 
                        fileName=savePath+'joint_pos.png')
        drawer3.multiTimePlot(posInfo[1], posInfo[2])

        drawer3_ = plotter(simVelInfo[0], 
                        timeStep = time_step,
                        legend=jointNames, 
                        xlabel='t [s]', 
                        ylabel='Simulated Joint velocity [Radian/s]', 
                        fileName=savePath+'simulated_joint_vel.png')
        drawer3_.multiTimePlot(simVelInfo[1], simVelInfo[2])

        _drawer3 = plotter(ocVelInfo[0], 
                        timeStep = time_step,
                        legend=jointNames, 
                        xlabel='t [s]', 
                        ylabel='Optimized Joint velocity [Radian/s]', 
                        fileName=savePath+'optimization_joint_vel.png')
        _drawer3.multiTimePlot(ocVelInfo[1], ocVelInfo[2])

        drawer4 = plotter(torqueInfo[0], 
                        timeStep = time_step,
                        legend=jointNames, 
                        xlabel='t [s]', 
                        ylabel='Joint torque [N$\cdot$m]', 
                        fileName=savePath+'joint_torque.png')
        drawer4.multiTimePlot(torqueInfo[1], torqueInfo[2])
        if cmpInfo is not None:
            drawer5 = plotter(cmpInfo[0],cmpInfo[1], 
                            timeStep = time_step,
                            legend=jointBulletNames,
                            legend_cmp=jointSolNames,
                            xlabel='t [s]', 
                            ylabel='Joint position [radian]', 
                            fileName=savePath+'joint_position_cmp_simulation.png')
            drawer5.cmpMultiTimePlot(cmpInfo[2], cmpInfo[3])
        
    def toHardwareTraj(self, traj):
        traj = traj.copy()
        print('original data columns: %d'%traj.shape[1])
        SAMPLE_TIME = 0.008
        traj_cols = int(traj.shape[1])
        new_traj = np.zeros([int(traj.shape[0]),int(traj.shape[1]//3+1)])

        for i in range(traj_cols):
            if i%3 ==0:
                new_traj[:,i//3] = traj[:,i]
        new_traj[:,-1] = traj[:,-1]
        print('converted data columns: %d'%new_traj.shape[1])
        return new_traj
    
    def downloadTraj(self, fileName):
        def downloadCmd():
            shell = "scp"
            goal = "jack_nuc:~/catkin_ws/src/humanoid/fall_data/"
            s = subprocess.call([shell, fileName, goal])
            if s:
                print('Error: unable to download')
            else:
                print('Success: download to hardware.')
        try:
            thread = threading.Thread(target=downloadCmd)
            thread.start()
            #thread.join()
        except:
            print "Error: unable to call thread!"

def simulation():
    simulator = Simulator(5000, display=True)

if __name__ == "__main__":
    simulation()

    





    
    
    

    




    
