from __future__ import division
import numpy as np
import pinocchio
from pinocchio.utils import eye, rotate, zero
from utils import plotter
from klampt.math import se3,so3


class simpleGenerator:
    def __init__(self, initPos, endPos, sample_length, isLinear = False):
        self.sample_length = sample_length
        if isLinear:
            self.traj = self.linear_generator(initPos, endPos)
        else:
            self.traj = self.smooth_generator(initPos, endPos)
    
    def linear_generator(self, initPos, endPos):
        traj = np.zeros([initPos.shape[0],self.sample_length])
        traj[:,0] = initPos
        traj[:,self.sample_length-1] = endPos
        for i in range(self.sample_length):
            if i>0 and i< (self.sample_length-1):
                traj[:,i] = (traj[:,self.sample_length-1]-traj[:,0])/(self.sample_length-1)*i+traj[:,0]
        return traj

    def smooth_generator(self, initPos, endPos):
        traj = np.zeros([initPos.shape[0],self.sample_length])
        traj[:,0] = initPos
        traj[:,self.sample_length-1] = endPos
        a = np.zeros([initPos.shape[0],4])
        a[:,0] = traj[:,0]
        a[:,1] = np.zeros([initPos.shape[0]])
        a[:,3] = 2*(a[:,0]-traj[:,self.sample_length-1])/((self.sample_length-1)**3)
        a[:,2] = -1.5*a[:,3]*(self.sample_length-1)
        for i in range(self.sample_length):
            traj[:,i] = a[:,0] + a[:,1]*i +a[:,2]*i**2+a[:,3]*i**3

        return traj

class tripleGenerator:
    def __init__(self, initPos, endPos, midPos, sample_length, midPara=0.5,isLinear = False):
        self.sample_length = sample_length
        self.midPara = midPara
        self.traj = self.tripleGenerator(initPos, endPos, midPos,isLinear)

    def tripleGenerator(self, initPos, endPos, midPos, isLinear):

        sample_length1 = int(self.sample_length*self.midPara)
        sample_length2 = self.sample_length - sample_length1+1
        traj1 = simpleGenerator(initPos,midPos,sample_length1, isLinear)
        traj2 = simpleGenerator(midPos,endPos,sample_length2, isLinear)
        traj = np.hstack((traj1.traj,traj2.traj[:,1:]))

        return traj

class MultipleGenerator:
    def __init__(self, poseArr, sample_length, paraArr=None,isLinear = False):
        self.sample_length = sample_length
        if paraArr is not None:
            assert len(paraArr) == (len(poseArr)-2), "[Error] Number of key poses are not consistent with number of parameters"
            self.paraArr = paraArr
        else:
            self.paraArr = np.arange(len(poseArr))
            self.paraArr = np.true_divide(self.paraArr, len(poseArr)-1)[1:-1]
            
        self.traj = self.multipleGenerator(poseArr,isLinear)

    def multipleGenerator(self, poseArr, isLinear):
        sample_length = []
        sample_length.append(int(self.paraArr[0]*self.sample_length))
        for i in range(len(self.paraArr)-1):
            sample_length.append(int((self.paraArr[i+1]-self.paraArr[i])*self.sample_length)+1)
        sample_length.append(int((1-self.paraArr[-1])*self.sample_length)+1)
        # print(sample_length)
        # print(poseArr[0], poseArr[1], sample_length[0])
        traj = simpleGenerator(poseArr[0],poseArr[1],sample_length[0], isLinear)
        traj = traj.traj
        for i in range(len(poseArr)-2):
            # print(poseArr[i+1], poseArr[i+2], sample_length[i+1])
            new_traj = simpleGenerator(poseArr[i+1],poseArr[i+2],sample_length[i+1], isLinear)
            # print(traj, new_traj.traj)
            traj = np.hstack((traj, new_traj.traj[:,1:]))
        # print(traj)
        return traj

class SE3Generator:
    def __init__(self, initSE3, endSE3, sample_length):
        self.initSE3 = initSE3
        self.endSE3 = endSE3
        self.sample_length = sample_length
        self.traj = self.se3Generator()
    
    def se3Generator(self):
        SE3 = pinocchio.SE3
        traj = []
        start_SE3 = (np.asarray(self.initSE3.rotation).reshape(-1).tolist(),np.asarray(self.initSE3.translation).reshape(-1).tolist())
        end_SE3 = (np.asarray(self.endSE3.rotation).reshape(-1).tolist(),np.asarray(self.endSE3.translation).reshape(-1).tolist())
        for i in range(self.sample_length):
            ratio = i/(self.sample_length-1)
            interpolate = se3.interpolate(start_SE3, end_SE3, ratio)
            interpolate_se3= SE3(np.asmatrix(so3.matrix(interpolate[0])),np.asmatrix(interpolate[1]).T)
            traj.append(interpolate_se3)
        
        return traj


def test():
    start = np.array([1])
    end = np.array([1])
    mid = np.array([3])
    sample_length = 200
    traj = tripleGenerator(start,end,mid,sample_length, isLinear= False)
    drawer = plotter(traj.traj, 
                    timeStep = 1.0,
                    label=['sample_trajectory'], 
                    xlabel='samples', 
                    ylabel='sample_trajectory', 
                    fileName='linear.png')

    posArr = [np.array([1]), np.array([2]), np.array([3]), np.array([5]), np.array([20])]
    paraArr = [0.5, 0.8, 0.9]
    sample_length = 200
    traj = MultipleGenerator(posArr, sample_length,paraArr=paraArr, isLinear= False)
    drawer = plotter(traj.traj, 
                    timeStep = 1.0,
                    label=['sample_trajectory'], 
                    xlabel='samples', 
                    ylabel='sample_trajectory', 
                    fileName='linear.png')
    drawer.timePlot()
    SE3 = pinocchio.SE3
    start_SE3 = SE3(eye(3), np.matrix([0, 0, 0]).T)*SE3(eye(3), np.matrix([0, 0, 0]).T)
    end_SE3 = start_SE3*SE3(rotate('x', -np.pi/2), zero(3))*SE3(rotate('z', -np.pi/2),zero(3))
    se3Generator = SE3Generator(start_SE3, end_SE3, sample_length=3)

if __name__ == "__main__":
    test()