import sys
import os
from os.path import dirname, exists, join

import numpy as np
import pinocchio
from pinocchio.utils import *
from pinocchio.robot_wrapper import RobotWrapper
from pinocchio.rpy import matrixToRpy, rpyToMatrix, rotate
from robot_properties_solo.config import SoloConfig


def vec2list(m): return np.array(m.T).reshape(-1).tolist()
def m2a(m): return np.array(m.flat)
def a2m(a): return np.matrix(a).T

def readParamsFromSrdf(robot, SRDF_PATH, verbose, has_rotor_parameters=True, referencePose='kneeling'):
    rmodel = robot.model

    if has_rotor_parameters:
        pinocchio.loadRotorParameters(rmodel, SRDF_PATH, verbose)
    rmodel.armature = np.multiply(rmodel.rotorInertia.flat, np.square(rmodel.rotorGearRatio.flat))
    pinocchio.loadReferenceConfigurations(rmodel, SRDF_PATH, verbose)
    if referencePose is not None:
        robot.q0.flat[:] = rmodel.referenceConfigurations[referencePose].copy()
    return
def addFreeFlyerJointLimits(robot):
    rmodel = robot.model

    ub = rmodel.upperPositionLimit
    ub[:7] = 1
    rmodel.upperPositionLimit = ub
    lb = rmodel.lowerPositionLimit
    lb[:7] = -1
    rmodel.lowerPositionLimit = lb
def loadRobot():
    vec2list = lambda m: np.array(m.T).reshape(-1)
    pack_path = SoloConfig.packPath
    urdf_path = SoloConfig.urdf_path
    srdf_path = os.path.dirname(
        os.path.dirname(urdf_path)) + '/urdf/humanoid.srdf'
    package_dirs = [os.path.dirname(
        os.path.dirname(urdf_path)) + '/urdf']
    robot = SoloConfig.buildRobotWrapper()
    readParamsFromSrdf(robot,srdf_path, False)
    assert ((robot.model.armature[:6] == 0.).all())
    addFreeFlyerJointLimits(robot)

    '''
    legMaxId = 14
    m1 = robot.model
    m2 = pinocchio.Model()
    for j, M, name, parent, Y in zip(m1.joints, m1.jointPlacements, m1.names, m1.parents, m1.inertias):
        if j.id < legMaxId:
            jid = m2.addJoint(parent, getattr(pinocchio, j.shortname())(), M, name)
            upperPos = m2.upperPositionLimit
            lowerPos = m2.lowerPositionLimit
            effort = m2.effortLimit
            upperPos[m2.joints[jid].idx_q:m2.joints[jid].idx_q + j.nq] = m1.upperPositionLimit[j.idx_q:j.idx_q + j.nq]
            lowerPos[m2.joints[jid].idx_q:m2.joints[jid].idx_q + j.nq] = m1.lowerPositionLimit[j.idx_q:j.idx_q + j.nq]
            effort[m2.joints[jid].idx_v:m2.joints[jid].idx_v + j.nv] = m1.effortLimit[j.idx_v:j.idx_v + j.nv]
            m2.upperPositionLimit = upperPos
            m2.lowerPositionLimit = lowerPos
            m2.effortLimit = effort
            assert (jid == j.id)
            m2.appendBodyToJoint(jid, Y, pinocchio.SE3.Identity())

    upperPos = m2.upperPositionLimit
    upperPos[:7] = 1
    m2.upperPositionLimit = upperPos
    lowerPos = m2.lowerPositionLimit
    lowerPos[:7] = -1
    m2.lowerPositionLimit = lowerPos
    effort = m2.effortLimit
    effort[:6] = np.inf
    m2.effortLimit = effort
    for f in m1.frames:
        if f.parent < legMaxId:
            m2.addFrame(f)

    g2 = pinocchio.GeometryModel()
    for g in robot.visual_model.geometryObjects:
        if g.parentJoint < 14:
            g2.addGeometryObject(g)

    robot.model = m2
    robot.data = m2.createData()
    robot.visual_model = g2
    # robot.q0=q2
    robot.visual_data = pinocchio.GeometryData(g2)
    '''
    return robot