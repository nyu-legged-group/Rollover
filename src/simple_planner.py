from planner import Planner
from generator import simpleGenerator, tripleGenerator, MultipleGenerator
import numpy as np

class SimpleKneePlanner(Planner):
    def __init__(self, x0, nq, nv, na, control_length, contact_index=0, timeStep=1e-3):
        super(SimpleKneePlanner, self).__init__(x0, nq, nv, na, control_length, contact_index, timeStep)

    def forward(self):
        '''
        Plan a simple bimanual trajecotry using kinematic data from [Abdolshah2018]
        '''
        tauTraj = np.zeros([self.na, self.control_length])

        initKnee = np.array([self.x0[10]])
        endKnee = np.array([self.x0[10]+np.pi/2])
        kneeTraj = simpleGenerator(initKnee,endKnee,self.control_length,isLinear= False)
        
        q = self.x0[:self.nq]
        for i in range(self.ac_length):
            tauTraj[:,i] = q[7:].copy().ravel()
            tauTraj[3,i] = tauTraj[13,i] = kneeTraj.traj[0, i]
        
        return tauTraj

class SimpleBimaunalPlanner(Planner):
    def __init__(self, x0, nq, nv, control_length, contact_index=0, timeStep=1e-3):
        super(SimpleBimaunalPlanner, self).__init__(x0, nq, nv, control_length, contact_index, timeStep)

    def forward(self):
        '''
        Plan a simple bimanual trajecotry using kinematic data from [Abdolshah2018]
        '''
        tauTraj = np.zeros([self.na, self.control_length])

        force_length = self.ac_length
        initElbow = np.array([self.x0[16]])
        midElbow = np.array([-(np.pi-131./180.*np.pi)])
        endElbow = np.array([-(np.pi-152./180.*np.pi)])
        elbowTraj = tripleGenerator(initElbow,endElbow,midElbow,force_length, midPara=0.25, isLinear= False)
        q = self.x0[:self.nq]
        for i in range(self.ac_length):
            tauTraj[:,i] = q[7:].copy().ravel()
            tauTraj[9,i]=tauTraj[19,i]=elbowTraj.traj[0,i]
        
        return tauTraj
class SimpleHipPlanner(Planner):
    def __init__(self, x0, nq, nv, control_length, contact_index=0, timeStep=1e-3):
        super(SimpleHipPlanner, self).__init__(x0, nq, nv, control_length, contact_index, timeStep)

    def forward(self):
        '''
        Plan a simple hip trajecotry
        '''
        tauTraj = np.zeros([self.na, self.control_length])

        force_length = self.ac_length
        # x0[first id]
        initHipYR = np.array([0, 0, 0, 0,  self.x0[26], self.x0[24], self.x0[25]])
        midHipYR = np.array([-30./180.*np.pi, 45./180.*np.pi, 10./180.*np.pi, 0,30./180.*np.pi, -10./180.*np.pi, np.pi/4+np.pi/2])
        endHipYR = np.array([15./180.*np.pi, 15./180.*np.pi, 15./180.*np.pi, 15./180.*np.pi, 30./180.*np.pi, -10./180.*np.pi, np.pi/4+np.pi/2])
        posArr = [initHipYR, midHipYR, endHipYR]
        paraArr = [0.5]
        hipTraj = MultipleGenerator(posArr, self.control_length,paraArr=paraArr, isLinear= False)
        
        #hipTraj = tripleGenerator(initHipYR,endHipYR,midHipYR,self.control_length, midPara=0.5, isLinear= False)
        # hipTraj = simpleGenerator(initHipYR,endHipYR,self.control_length,isLinear= False)
        q = self.x0[:self.nq]
        for i in range(self.control_length):
            tauTraj[:,i] = q[7:].copy().ravel()

            # r_hip_y
            tauTraj[10,i]=hipTraj.traj[0,i]
            # r_hip_r
            tauTraj[11,i]=hipTraj.traj[1,i]
            # l_hip_r
            tauTraj[1,i]=hipTraj.traj[2,i]
            # l_hip_y
            tauTraj[0,i]=hipTraj.traj[3,i]
            # r_elbow
            tauTraj[19,i]=hipTraj.traj[4,i]
            # r_shoulder_r
            tauTraj[17,i]=hipTraj.traj[5,i]
            # r_shoulder_y
            tauTraj[18,i]=hipTraj.traj[6,i]
            # tauTraj[second id]
        # print(hipTraj.traj[-1,:])
        return tauTraj
class SimpleShoulderPlanner(Planner):
    def __init__(self, x0, nq, nv, control_length, contact_index=0, timeStep=1e-3):
        super(SimpleShoulderPlanner, self).__init__(x0, nq, nv, control_length, contact_index, timeStep)

    def forward(self):
        '''
        Plan a simple hip trajecotry
        '''
        tauTraj = np.zeros([self.na, self.control_length])

        force_length = self.ac_length
        # x0[first id]
        self.x1 = self.x0.T.tolist()[0]
        # 13: l_shoulder_pitch
        # 16: l_elbow
        # 14: l_shoulder_roll
        # 15: l_shoulder_yaw
        # 26: r_elbow
        # 24: r_shoulder_roll
        # 
        initHipYR = np.array([self.x1[13], self.x1[16], self.x1[14], self.x1[15],  self.x1[26], self.x1[24], self.x1[25]])
        midHipYR = np.array([-88./180.*np.pi, (180.-120.)/180.*np.pi, 25./180.*np.pi, -np.pi/2-np.pi/10.,30./180.*np.pi, -10./180.*np.pi, np.pi/4+np.pi/2])
        endHipYR = np.array([-88./180.*np.pi, (180.-120.)/180.*np.pi, 25./180.*np.pi, -np.pi/2-np.pi/10., 30./180.*np.pi, -10./180.*np.pi, np.pi/4+np.pi/2])
        posArr = [initHipYR, midHipYR, endHipYR]
        # print(posArr)
        paraArr = [0.5]
        hipTraj = MultipleGenerator(posArr, self.control_length,paraArr=paraArr, isLinear= False)
        
        #hipTraj = tripleGenerator(initHipYR,endHipYR,midHipYR,self.control_length, midPara=0.5, isLinear= False)
        # hipTraj = simpleGenerator(initHipYR,endHipYR,self.control_length,isLinear= False)
        q = self.x0[:self.nq]
        for i in range(self.control_length):
            tauTraj[:,i] = q[7:].copy().ravel()

            # l_shoulder_p, r_shoulder_p
            tauTraj[6,i]=tauTraj[16,i]=hipTraj.traj[0,i]
            # l_elbow, r
            tauTraj[9,i]=tauTraj[19,i]=hipTraj.traj[1,i]
            # l_shoulder_r, r..
            tauTraj[7,i]=tauTraj[17,i]=hipTraj.traj[2,i]
            # l_shoulder_y
            tauTraj[8,i]=hipTraj.traj[3,i]
            tauTraj[18,i]=-hipTraj.traj[3,i]
            
        return tauTraj
class SimpleRolloverTrajWrapper(Planner):
    '''
    This class is used to finish the final action in rollover position.
    '''
    def __init__(self, x0, nq, nv, control_length, contact_index=0, timeStep=1e-3):
        super(SimpleRolloverTrajWrapper, self).__init__(x0, nq, nv, control_length, contact_index, timeStep)

    def forward(self, traj):

        initPose = traj[:,-1]
        endPose = np.asarray(self.x0[7:self.nq].copy().T)
        # Adjust the final posture based on paper [Abdolshah2018]
        print(endPose.shape)
        endPose[0,6] = endPose[0,16] = -np.pi/6
        endPose[0,2] = endPose[0,12] = np.pi/6
        endPose[0,8] = -np.pi/2
        endPose[0,18] = np.pi/2
        endPose[0,3] = endPose[0,13] = -np.pi/3
        #print(initPose.shape, endPose.shape)
        endTraj = simpleGenerator(initPose,endPose,self.control_length,isLinear= False)
        #print(traj.shape, endTraj.traj.shape)
        tauTraj = np.hstack((traj, endTraj.traj))
        
        return tauTraj
def test():
    pinocchio.switchToNumpyMatrix()
    horizon_length = 5000
    m = VisualModel()
    x0, nq, nv, na = m.x0, m.nq, m.nv, m.na

    simpleKneePlanner = SimpleKneePlanner(x0, nq, nv, na, horizon_length)
    kneeTraj = simpleKneePlanner.forward()

    simple_control_length = 400
    simpleBimaunalPlanner = SimpleBimaunalPlanner(x0, nq, nv, na, simple_control_length)
    tauBimanualTraj = simpleBimaunalPlanner.forward()

    s = HumanoidSimulator(5000, display=True)
    x0[7:nq] = np.matrix(tauBimanualTraj[:,0]).T
    s.initPose(x0, nq, nv, na)
    forceArr, comArr, _, _, _  = s.simulate(m, kneeTraj, tauBimanualTraj)
    s.plot(forceArr, comArr)
    
if __name__ == "__main__":
    test()
