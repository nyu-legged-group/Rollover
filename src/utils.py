import numpy as np
import pinocchio
from pinocchio.utils import *
from pinocchio.rpy import matrixToRpy, rpyToMatrix, rotate

import matplotlib.pyplot as plt
from matplotlib.ticker import (MultipleLocator, FormatStrFormatter, AutoMinorLocator)
plt.rcParams['font.family'] = ['serif']
plt.rcParams['font.serif'] = ['Times New Roman']
plt.rcParams.update({'font.size': 12})
vec2list = lambda m: np.array(m.T).reshape(-1).tolist()
class plotter():
    def __init__(self, array, array_cmp = None, v_line = None, timeStep=1e-3,legend=[], legend_cmp=[], xlabel='', ylabel='', fileName='', xlim =[], ylim=[]):
        '''
        The class is to draw graph with IEEE standard used in paper.
        '''
        self.timeStep = timeStep
        self.array = array
        self.legend = legend
        if array_cmp is not None:
            self.array_cmp = array_cmp
            self.legend_cmp = legend_cmp
        
        self.v_line = v_line
        self.xlim = xlim
        self.ylim = ylim
        self.fileName = fileName
        self.xlabel = xlabel
        self.ylabel = ylabel

    def timePlot(self, timeArr=None):
        '''
        The function is to draw a array w.r.t time
        Assume the array's column increase with time
        '''
        array = self.array
        plt.figure()
        for i in range(array.shape[0]):
            if self.legend:
                if timeArr is not None:
                    plt.plot(timeArr[0,:], array[i,:],'-', label=self.legend[i])
                else:
                    plt.plot(np.arange(array.shape[1])*self.timeStep, array[i,:],'-', label=self.legend[i])
            else:
                if timeArr is not None:
                    plt.plot(timeArr[0,:], array[i,:],'-')
                else:
                    plt.plot(np.arange(array.shape[1])*self.timeStep, array[i,:],'-')
        ax = plt.subplot(1, 1, 1)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        if self.ylim:
            plt.ylim(self.ylim[0],self.ylim[1])
            #ax.yaxis.set_major_locator(MultipleLocator(100))
        if self.xlim:
            plt.xlim(self.xlim[0], self.xlim[1])
            #ax.xaxis.set_major_locator(MultipleLocator(0.2))
        plt.legend()
        if self.v_line:
            for line in self.v_line:
                plt.axvline(x=line*self.timeStep, color='r', linewidth=0.5)
        if self.xlabel:
            plt.xlabel(self.xlabel)
        else:
            plt.xlabel('t (s)')
        if self.ylabel:
            plt.ylabel(self.ylabel)
        if self.fileName:
            plt.savefig(self.fileName ,bbox_inches='tight',pad_inches = 0, dpi = 300)
        else:
            plt.savefig('Utitled.png',bbox_inches='tight',pad_inches = 0, dpi = 300)
        
    
    def multiTimePlot(self, lowerLimit=None, upperLimit=None, isVisiable=False):
        '''
        This function is to draw multiple plots w.r.t time
        Assume the array's columns increase w.r.t time
        Assume the array's rows are different entities
        '''
        array = self.array
        fig = plt.figure()
        rows = np.shape(array)[0]
        cols = np.shape(array)[1]
        for i in range(rows):

            if i >0:
                ax = fig.add_axes([0.1, 0.4*i+0.1, 0.8, 0.4], xticklabels=[])
                ax.tick_params(axis = "x", which = "both", bottom = False, top = False)
                ax.set_ylabel(self.ylabel) 
            else:
                ax = fig.add_axes([0.1, 0.4*i+0.1, 0.8, 0.4])
                ax.set_ylabel(self.ylabel) 
                if self.xlabel:
                    ax.set_xlabel(self.xlabel) 
                else:
                    ax.set_xlabel('t [s]')
            ax.plot(np.arange(array.shape[1])*self.timeStep, array[i,:],'-', label=self.legend[i])
            if lowerLimit is not None:
                lower = np.array([lowerLimit[i] for j in xrange(cols)])
                ax.plot(np.arange(array.shape[1])*self.timeStep, lower, 'r--')
            if upperLimit is not None:
                upper = np.array([upperLimit[i] for j in xrange(cols)])
                ax.plot(np.arange(array.shape[1])*self.timeStep, upper, 'r--')
            plt.legend()
            if self.v_line:
                for line in self.v_line:
                    # print('plot')
                    plt.axvline(x=line*self.timeStep, color='r', linewidth=0.5)
        
            
        if self.fileName:
            plt.savefig(self.fileName ,bbox_inches='tight',pad_inches = 0, dpi = 300)
        else:
            plt.savefig('Utitled.png',bbox_inches='tight',pad_inches = 0, dpi = 300)
        if not isVisiable:
            plt.close()
        else:
            plt.show()
    def cmpMultiTimePlot(self, lowerLimit=None, upperLimit=None):
        '''
        This function is to draw multiple plots w.r.t time with two arrays
        Assume the array's columns increase w.r.t time
        Assume the array's rows are different entities
        Assume the array1 and array2 are two arrays for present
        '''
        array1 = self.array
        array2 = self.array_cmp
        fig = plt.figure()
        rows1 = np.shape(array1)[0]
        cols1 = np.shape(array1)[1]
        rows2 = np.shape(array2)[0]
        cols2 = np.shape(array2)[1]
        assert rows1 == rows2, 'rows number is not equal'
        assert cols1 == cols2, 'columns number is not equal'

        rows = rows1
        cols = cols1

        for i in range(rows):
            if i >0:
                ax = fig.add_axes([0.1, 0.4*i+0.1, 0.8, 0.4], xticklabels=[])
                ax.tick_params(axis = "x", which = "both", bottom = False, top = False)
                ax.set_ylabel(self.ylabel) 
            else:
                ax = fig.add_axes([0.1, 0.4*i+0.1, 0.8, 0.4])
                ax.set_ylabel(self.ylabel) 
                if self.xlabel:
                    ax.set_xlabel(self.xlabel) 
                else:
                    ax.set_xlabel('t [s]')
            ax.plot(np.arange(array1.shape[1])*self.timeStep, array1[i,:],'b-', label=self.legend[i])
            ax.plot(np.arange(array1.shape[1])*self.timeStep, array2[i,:],'k-', label=self.legend_cmp[i])
            if lowerLimit is not None:
                lower = np.array([lowerLimit[i] for j in xrange(cols)])
                ax.plot(np.arange(array1.shape[1])*self.timeStep, lower, 'r--')
            if upperLimit is not None:
                upper = np.array([upperLimit[i] for j in xrange(cols)])
                ax.plot(np.arange(array1.shape[1])*self.timeStep, upper, 'r--')
            plt.legend()

        if self.fileName:
            plt.savefig(self.fileName ,bbox_inches='tight',pad_inches = 0, dpi = 300)
        else:
            plt.savefig('Utitled_cmpMultiTimePlot.png',bbox_inches='tight',pad_inches = 0, dpi = 300)

class DataHub():
    def __init__(self, directory):
        self.dir = directory
    def imuProcess(self, sample_freq= 100., period= None):
        IMU_DIR = self.dir+'.csv'
        IMU_OUT_DIR = self.dir+'.png'
        try:
            imu_raw = np.loadtxt(open(IMU_DIR, "rb"), delimiter=",")
        except:
            print("Read IMU data error!")
            exit()
        horizon_length = int(imu_raw.shape[1])
        rpy = np.zeros([3, horizon_length])
        for i in range(horizon_length):
            vector = np.matrix([0, 0, 0, imu_raw[0,i], imu_raw[1,i], imu_raw[2,i], imu_raw[3,i]]).T
            se3 = pinocchio.XYZQUATToSE3(vector)
            rpy_vector = matrixToRpy(se3.rotation)
            rpy[:,i] = vec2list(rpy_vector)
        rpy *=180./np.pi
        time_step = 1./sample_freq
        if period is not None:
            try:
                l_period = int(period[0]/time_step)
                r_period = int(period[1]/time_step)
                rpy = rpy[:,l_period:r_period]
            except:
                print('period is not valid')
                exit()
        # rpy1 = np.zeros([1, np.shape(rpy)[1]])
        # rpy1[0,:] = -rpy[1,:]
        timeArr = np.zeros([1, horizon_length])
        for i in horizon_length:
            timeArr[0,i] = imu_raw[10,i]+imu_raw[11,i]*1.e-9
        drawer = plotter(rpy[:,100:200], 
                        timeStep = time_step,
                        label=['Y','Z','X'], 
                        xlabel='t [s]', 
                        ylabel='angluar position [$^\circ$]', 
                        fileName=IMU_OUT_DIR)
        drawer.timePlot(timeArr=timeArr)
        
        
        drawer1 = plotter(imu_raw[7:10,100:200], 
                        timeStep = time_step,
                        label=['Y','Z','X'], 
                        xlabel='t [s]', 
                        ylabel='linear accelearation [m/s^2]', 
                        fileName=IMU_OUT_DIR)
        drawer1.timePlot()
    def encoderProcess(self, time_step, lowerLimit=None, upperLimit=None):
        ENCODER_DIR = self.dir+'encoder_0204.csv'
        CONTROL_DIR = self.dir+'desired_0204.csv'
        ENCODER_OUT_DIR = self.dir + 'encoder_sensor.png'
        try:
            coder_raw = np.loadtxt(open(ENCODER_DIR, "rb"), delimiter=",")
        except:
            print("Read encoder data error!")
            exit()
        try:
            ctrl_raw = np.loadtxt(open(CONTROL_DIR, "rb"), delimiter=",")
        except:
            print("Read Control data error!")
            exit()
        
        coder_len = int(coder_raw.shape[1])
        ctrl_len = int(ctrl_raw.shape[1])
        
        horizon_len = np.min([coder_len, ctrl_len])
        coder = np.zeros([23, horizon_len])
        ctrl = np.zeros([23, horizon_len])
        coder = coder_raw[:,:horizon_len]
        ctrl = ctrl_raw[:,:horizon_len]

        drawer = plotter(ctrl,coder, 
                        timeStep = time_step,
                        label=['l_hip_y', 'l_hip_r', 'l_hip_p',\
                                'l_knee', 'l_ankle_p', 'l_ankle_r',\
                                'r_hip_y', 'r_hip_r', 'r_hip_p',\
                                'r_knee', 'r_ankle_p', 'r_ankle_r',\
                                'torso',\
                                'l_shoulder_p', 'l_shoulder_r', 'l_shoulder_y','l_elbow',\
                                'r_shoulder_p', 'r_shoulder_r', 'r_shoulder_y', 'r_elbow',\
                                'r_gripper',\
                                'l_gripper'], 
                        xlabel='t (s)', 
                        ylabel='Joint position compare (radian)', 
                        fileName=ENCODER_OUT_DIR)
        drawer.cmpMultiTimePlot(lowerLimit, upperLimit)

def test():
    x = np.zeros([1,500])
    drawer = plotter(x, wrtTime=True, timeStep = 1e-3,label=['force'], xlabel='t (s)', ylabel='force (N)', fileName='force.png')
if __name__ == "__main__":
    test()