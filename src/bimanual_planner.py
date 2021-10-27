import os
import sys
import pinocchio
import crocoddyl
import numpy as np
import math

from humanoid import SimpleHumanoidGaitProblem, plotSolution
from visualizer import VisualModel
from simulator import HumanoidSimulator
from planner import Planner, OptimalPlanner
from simple_planner import SimpleKneePlanner

import matplotlib.pyplot as plt

class BimanualPlanner(OptimalPlanner):
    def __init__(self, x0, nq, nv, na, control_length, contact_index=1, timeStep=1e-3, display=False):
        super(BimanualPlanner, self).__init__(x0, nq, nv, na, control_length, contact_index, timeStep, display)
        
    def forward(self, model, handLength, comLength):
        def showGepetto():
            subprocess.call(["gepetto-gui"])
        if self.display:
            try:
                thread = threading.Thread(target=showGepetto)
                thread.start()
                time.sleep(10)
                #thread.join()
            except:
                print "Error: unable to start Gepetto-GUI thread"

        # Setting up contact timings
        timeStep = self.timeStep
        flyingKnots = self.bc_length
        groundKnots = self.ac_length

        # Setting up contact position
        handLength = handLength
        comLength = comLength

        # Setting up falling problem
        torso = 'torso'
        lFoot, rFoot = 'l_foot', 'r_foot'
        lKneePoint, rKneePoint, rKneePoint1  = 'l_knee_lp', 'r_knee_rp', 'r_knee_lp'
        lHandPoint, rHandPoint = 'l_hand_dp', 'r_hand_dp'

        q0 = self.x0[:self.nq].copy()
        gait = SimpleHumanoidGaitProblem(model.model, q0, torso, lFoot, rFoot,lKneePoint, rKneePoint, rKneePoint1, lHandPoint, rHandPoint)

        # Setting up all tasks
        problem = gait.createBimanualFallProblem(self.x0, handLength, comLength, timeStep,
                                                groundKnots, flyingKnots, final=False)
        endEffectors = [lKneePoint, rKneePoint,lHandPoint, rHandPoint]
        tauTraj = self.solve(model, problem, endEffectors)

        return tauTraj

def test():
    pinocchio.switchToNumpyMatrix()
    horizon_length = 5000
    control_length = 400
    m = VisualModel()
    x0, nq, nv, na = m.x0, m.nq, m.nv, m.na

    simpleKneePlanner = SimpleKneePlanner(x0, nq, nv, na, horizon_length)
    kneeTraj = simpleKneePlanner.forward()

    s = HumanoidSimulator(horizon_length, display=True)
    s.initPose(x0, nq, nv, na)
    forceArr, comArr, x1, handLength, comLength  = s.simulate(m, kneeTraj, kneeTraj)
    bimanualPlanner = BimanualPlanner(x1, nq, nv, na, control_length, display=False)
    tauBimanualTraj = bimanualPlanner.forward(m, handLength, comLength)
    tauBimanualTraj_index = bimanualPlanner.contact_index
    bimanualPlanner.saveTraj(np.matrix(tauBimanualTraj).T)
    ss = HumanoidSimulator(horizon_length, display=True)
    ss.initPose(x0, nq, nv, na)
    forceArr, comArr, _, _, _  = ss.simulate(m, kneeTraj, tauBimanualTraj[:,tauBimanualTraj_index:])

if __name__ == "__main__":
    test()