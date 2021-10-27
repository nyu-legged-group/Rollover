import os
import sys
import pinocchio
import crocoddyl
import numpy as np
import math
import subprocess
import threading

from humanoid import SimpleHumanoidGaitProblem, plotSolution
from visualizer import VisualModel
from simulator import HumanoidSimulator
from planner import Planner, OptimalPlanner
from simple_planner import SimpleKneePlanner
import time

import matplotlib.pyplot as plt

class RolloverPlanner(OptimalPlanner):
    def __init__(self, x0, nq, nv, na, control_length, contact_index=1, timeStep=1e-3, display=False):
        super(RolloverPlanner, self).__init__(x0, nq, nv, na, control_length, contact_index, timeStep, display)

    def forward(self, model, handLength, timeLength):
        def showGepetto():
            subprocess.call(["gepetto-gui"])
        if self.display:
            try:
                thread = threading.Thread(target=showGepetto)
                thread.start()
                time.sleep(1)
                #thread.join()
            except:
                print "Error: unable to start Gepetto-GUI thread"

        # Setting up contact timings
        timeStep = self.timeStep
        flyingKnots = self.bc_length
        groundKnots = self.ac_length

        # Setting up contact position
        handLength = handLength

        # Setting up falling problem
        torso = 'base_link'
        lFoot, rFoot = 'l_foot', 'r_foot'
        lKneePoint, rKneePoint, rKneePoint1  = 'l_knee_lp', 'r_knee_rp', 'r_knee_lp'
        lHandPoint, rHandPoint = 'l_hand_dp', 'r_hand_dp'

        q0 = self.x0[:self.nq].copy()
        gait = SimpleHumanoidGaitProblem(model.model, q0, torso, lFoot, rFoot,lKneePoint, rKneePoint, rKneePoint1, lHandPoint, rHandPoint)

        # Setting up all tasks
        problem = gait.createRolloverFallProblem(self.x0, handLength, timeLength, timeStep, groundKnots, flyingKnots, final=False)
        endEffectors = [lKneePoint, rKneePoint,lHandPoint, rHandPoint]

        tauTraj, velTraj = self.solve(model, problem, endEffectors)

        return tauTraj, velTraj

        
def test():
    pinocchio.switchToNumpyMatrix()
    # Define simulation steps
    horizon_length = 5000
    time_step = 1e-3

    # Define control trajectory steps 
    ctrl_time_step = 1e-2

    # Load pinocchio model
    m = VisualModel()
    x0, nq, nv, na = m.x0, m.nq, m.nv, m.na

    # Simple knee trajectory to initiate robot's movement
    simpleKneePlanner = SimpleKneePlanner(x0, nq, nv, na, horizon_length)
    kneeTraj = simpleKneePlanner.forward()

    # Simulate static trajectory to obtain handLength and initial pose
    s = HumanoidSimulator(horizon_length, display=False, timeStep=time_step)
    s.initPose(x0, nq, nv, na)
    _, _, _, _, _, x1, handLength, timeLength  = s.simulate(m, kneeTraj, kneeTraj)
    ctrl_horizon_length = timeLength//int(ctrl_time_step/time_step)
    print('ctrl_length:%d'%ctrl_horizon_length)
    # Plan optimal rollover trajectory
    rolloverPlanner = RolloverPlanner(x1, nq, nv, na, ctrl_horizon_length*2, contact_index=ctrl_horizon_length, timeStep=ctrl_time_step, display=True)
    tauRolloverTraj = rolloverPlanner.forward(m, handLength)
    tauRolloverTraj_index = rolloverPlanner.contact_index
    rolloverPlanner.saveTraj(np.matrix(tauRolloverTraj).T)

    # # Simulate optimal rollover trajectory
    # ss = HumanoidSimulator(horizon_length, display=True,timeStep=time_step)
    # ss.initPose(x0, nq, nv, na)
    # forceArr, comArr = ss.simulateOptTraj(m, kneeTraj, tauRolloverTraj,ctrlTimeStep=ctrl_time_step)

    # # Plot simulated result
    # ss.plot(forceArr,comArr)

if __name__ == "__main__":
    test()