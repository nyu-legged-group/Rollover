import numpy as np
from generator import tripleGenerator, simpleGenerator
from simulator import HumanoidSimulator
from visualizer import VisualModel
from pinocchio.rpy import matrixToRpy, rpyToMatrix, rotate
import pinocchio
import crocoddyl
import math
import subprocess
import threading
import time

class Planner(object):
    def __init__(self, x0, nq, nv, na, control_length, contact_index=0, timeStep=1e-3):
        self.nq, self.nv, self.na, self.x0 = nq, nv, na, x0
        self.control_length, self.contact_index = control_length, contact_index
        self.timeStep = timeStep
        self.check()
        self.bc_length = contact_index
        self.ac_length = control_length - contact_index

    def forward(self):
        raise NotImplementedError

    def check(self):
        assert self.contact_index < self.control_length-1, "[Error] contact_index should be smaller than total plan length."

    def saveTraj(self, tauTraj, savePath):
        np.savetxt(savePath, tauTraj, fmt="%f", delimiter=",")
    

class OptimalPlanner(Planner):
    def __init__(self, x0, nq, nv, na, control_length, contact_index=1, timeStep=1e-3, display=False):
        super(OptimalPlanner, self).__init__(x0, nq, nv, na, control_length, contact_index, timeStep)
        self.display = display
    def forward(self):
        raise NotImplementedError

    def solve(self, model, problem, endEffectors):
        cameraTF = [1.2, 1.5, 0.5, 0.2, 0.62, 0.72, 0.22]

        self.ddp = crocoddyl.SolverBoxFDDP(problem)
        print('*** SOLVE  ***')
        if self.display==True:
            _display = crocoddyl.GepettoDisplay(model.robot, 4, 4, cameraTF, frameNames=endEffectors)
            self.ddp.setCallbacks(
                [crocoddyl.CallbackLogger(),
                crocoddyl.CallbackVerbose(),
                crocoddyl.CallbackDisplay(_display)])
        else:
            self.ddp.setCallbacks(
                [crocoddyl.CallbackLogger(),
                crocoddyl.CallbackVerbose()])

        # Solving the problem with the DDP solver
        xs = [model.model.defaultState] * (self.ddp.problem.T + 1)
        us = [
            m.quasiStatic(d, model.model.defaultState)
            for m, d in list(zip(self.ddp.problem.runningModels, self.ddp.problem.runningDatas))
        ]
        done = self.ddp.solve(xs, us, 500, False, 0.1)
        print(done)
        if self.display==True:
            _display = crocoddyl.GepettoDisplay(model.robot, -1,0.1, cameraTF=cameraTF,visibility=False)
            _display.displayFromSolver(self.ddp, factor=5.)
        
        self.x = np.zeros([self.nq, len(self.ddp.xs)])
        self.v = np.zeros([self.nv, len(self.ddp.xs)])
        for i in range(len(self.ddp.xs)):
            for j in range(self.nq):
                self.x[j, i] = self.ddp.xs[i][j]

        for i in range(len(self.ddp.xs)):
            for j in range(self.nv):
                self.v[j, i] = self.ddp.xs[i][self.nq+j]

        tauTraj = self.x[7:,:]
        velTraj = self.v[6:,:]
        
        return tauTraj, velTraj
    def displayTraj(self, model):
        def showGepetto():
            subprocess.call(["gepetto-gui"])
        
        try:
            thread = threading.Thread(target=showGepetto)
            thread.start()
            #thread.join()
        except:
            print "Error: unable to start Gepetto-GUI thread"
        time.sleep(5)

        cameraTF = [1.2, 1.5, 0.5, 0.2, 0.62, 0.72, 0.22]
        _display = crocoddyl.GepettoDisplay(model.robot, -1,0.1, cameraTF=cameraTF,visibility=False)
        _display.displayFromSolver(self.ddp, factor=5.)




