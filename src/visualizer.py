import numpy as np
import pinocchio
from pinocchio.utils import *
from pinocchio.rpy import matrixToRpy, rpyToMatrix, rotate
from robot_properties_solo.config import SoloConfig
from loader import loadRobot
import subprocess
import threading
import time

class VisualModel():
    def __init__(self,display=False):
        self.display=display
        def showGepetto():
            subprocess.call(["gepetto-gui"])
        if self.display:
            try:
                thread = threading.Thread(target=showGepetto)
                thread.start()
                #thread.join()
            except:
                print "Error: unable to start Gepetto-GUI thread"
        self.robot = loadRobot()
        self.model = self.robot.model
        self.data = self.model.createData()
        self.nq = self.model.nq
        self.nv = self.model.nv
        self.na = self.model.nq-7
        self.dq0 = pinocchio.utils.zero(self.model.nv)
        self.q0 = self.model.referenceConfigurations["kneeling"]
        lSole = 'l_foot'
        rSole = 'r_foot'
        lKneePoint = 'l_knee_lp'
        rKneePoint = 'r_knee_rp'
        lHandPoint = 'l_hand_dp'
        rHandPoint = 'r_hand_dp'
        lToe = 'l_foot_toe'
        rToe = 'r_foot_toe'

        self.lSoleId = self.model.getFrameId(lSole)
        self.rSoleId = self.model.getFrameId(rSole)
        self.lkId = self.model.getFrameId(lKneePoint)
        self.rkId = self.model.getFrameId(rKneePoint)
        self.lhId = self.model.getFrameId(lHandPoint)
        self.rhId = self.model.getFrameId(rHandPoint)
        self.lToeId = self.model.getFrameId(lToe)
        self.rToeId = self.model.getFrameId(rToe)
        
        # Calibrate origin at center of knee points
        self.calibrateOrigin()
        
        # # Measure distance
        # lhPos0 = self.data.oMf[self.lhId].translation
        # diffPos0 = lhPos0-lkPos0

        # Manual Tuning(Joint number table)
        # torso: 0, 1, 2, 3, 4, 5, 6
        # left_leg: 7(0) l_hip_y, 8(1) l_hip_r, 9(2), 10(3), 11(4), 12(5)
        # left_arm: 13(6), 14(7), 15(8), 16(9)
        # right_leg: 17(10), 18(11), 19(12), 20(13), 21(14), 22(15)
        # right_arm: 23(16), 24(17), 25(18), 26(19)


        # l1: distance between axis of ankle roll and axis of knee
        # l2: distance between axis of ankle roll and toe
        # l3: distance between axis of knee and knee point
        l1 = 0.144
        l2 = 0.03999
        l3 = 0.014
        kneeOffset = anklePitchOffset= np.arcsin((l2-l3)/l1)
        SE3 = pinocchio.SE3
        #arm
        self.q0[13] = self.q0[23] = -np.pi/2
        self.q0[14] = self.q0[24]= 6.0/180.*np.pi
        self.q0[15] = -(np.pi/2-15./180.*np.pi)
        self.q0[25] = np.pi/2-15./180.*np.pi
        self.q0[16] = self.q0[26]= (np.pi-163./180.*np.pi)#Initialize elbow position using kinematic data from [Abdolshah2018]

        #leg
        self.q0[9] = self.q0[19]= 0
        self.q0[10] = self.q0[20]= -np.pi/2-kneeOffset
        self.q0[11] = self.q0[21]= -anklePitchOffset
       
        # start_SE3 = SE3(eye(3), np.matrix([0, 0, 0]).T)*SE3(eye(3), np.matrix([0, 0, 0]).T)
        # mid_SE3 = start_SE3*SE3(rotate('y', np.pi/3),zero(3))*SE3(rotate('z', np.pi/2),zero(3))
        # end_SE3 = mid_SE3 * SE3(rotate('z', np.pi/2),zero(3))
        # self.q0[3:7] = se3ToXYZQUAT(mid_SE3)[3:7]

        pinocchio.forwardKinematics(self.model, self.data, self.q0)
        pinocchio.updateFramePlacements(self.model, self.data)
        lkPos0 = self.data.oMf[self.lkId].translation
        ltPos0 = self.data.oMf[self.lToeId].translation
        
        self.calibrateOrigin()

        lhPos0 = self.data.oMf[self.lhId].translation
        rhPos0 = self.data.oMf[self.rhId].translation
        # print(self.rhId)
        # print(lhPos0, rhPos0)

        self.q0[2]+=0.000001 # increase height for pybullet simulation
        self.x0 = np.concatenate([self.q0, self.dq0])
        self.model.defaultState = np.concatenate([self.q0, np.zeros((self.model.nv, 1))])
        if display:
            self.show()
    
    def show(self):
            time.sleep(10)
            self.robot.initViewer(loadModel=True)
            self.robot.viewer.gui.addFloor('hpp-gui/floor')
            self.robot.display(self.q0)
    
    def calibrateOrigin(self):
        pinocchio.forwardKinematics(self.model, self.data, self.q0)
        pinocchio.updateFramePlacements(self.model, self.data)
        lkPos0 = self.data.oMf[self.lkId].translation
        rkPos0 = self.data.oMf[self.rkId].translation
        self.q0[:3]-=(lkPos0+rkPos0)/2
        pinocchio.forwardKinematics(self.model, self.data, self.q0)
        pinocchio.updateFramePlacements(self.model, self.data)
        lkPos0 = self.data.oMf[self.lkId].translation
        rkPos0 = self.data.oMf[self.rkId].translation
        originPos0 = (lkPos0+rkPos0)/2
        assert abs(originPos0[0])<1.e-6, "[Error]Origin is not zeroed in x direction!"
        assert abs(originPos0[1])<1.e-6, "[Error]Origin is not zeroed in y direction!"
        assert abs(originPos0[2])<1.e-6, "[Error]Origin is not zeroed in z direction!"
    
    def visualizeConfig(self,q):
        def showGepetto():
            subprocess.call(["gepetto-gui"])
        
        try:
            thread = threading.Thread(target=showGepetto)
            thread.start()
            #thread.join()
        except:
            print "Error: unable to start Gepetto-GUI thread"
        time.sleep(5)
        self.robot.initViewer(loadModel=True)
        self.robot.viewer.gui.addFloor('hpp-gui/floor')
        size_q = np.shape(q)[0]
        if size_q == self.nq-7:
            torso_pose = np.array([0,0,0.5, 0,0,0,1])
            q = np.hstack((torso_pose,q))
        self.robot.display(q)
    
    
    def generateSwData(self, x):
        '''
        This function is used to generate solidworks data type
        '''
        q = np.asarray(x).copy()
        quat = q[3:7].copy()
        # Convert quaternion to rpy
        vector = np.matrix([0, 0, 0, quat[0], quat[1], quat[2], quat[3]]).T
        se3 = pinocchio.XYZQUATToSE3(vector)
        rpy = matrixToRpy(se3.rotation)
        q[3] = rpy[0]
        q[4] = rpy[1]
        q[5] = rpy[2]
        q[6] = 0

        # Process unit, for distance convert m to mm
        # Process unit, for angle, convert radian to degree
        # print(q[0:3])
        q[0] += 0.0264
        q[1] += 0.
        q[2] += 0.50958
        for i in range(3):
            q[i] *=1000.
            q[i] = np.abs(q[i])
        
        for i in range(3, self.nq):
            q[i] *=180./np.pi
            if q[i] < 0:
                q[i] +=360.
        return q 

    def saveEquation(self, x, savePath):
        '''
        Legacy function
        This function is used to generate configuration file with Solidworks in "equation" interface
        '''

        q = self.generateSwData(x)

        f = open(savePath, "w+")
        for i in range(self.nq):
            f.write('"q%d" = %f\r\n'%(i, q[i]))
        f.write('\n')

        names = []
        names.append('"D1@q0_x"= "q0"\r\n')
        names.append('"D1@q1_y"= "q1"\r\n')
        names.append('"D1@q2_z"= "q2"\r\n')
        names.append('"D1@q3_x"= "q3"\r\n')
        names.append('"D1@q4_y"= "q4"\r\n')
        names.append('"D1@q5_z"= "q5"\r\n')
        names.append('"D1@q7_l_hip_y"= "q7"\r\n')
        names.append('"D1@q8_l_hip_r"= "q8"\r\n')
        names.append('"D1@q9_l_hip_p"= "q9"\r\n')
        names.append('"D1@q10_l_knee"= "q10"\r\n')
        names.append('"D1@q11_l_ankle_p"= "q11"\r\n')
        names.append('"D1@q12_l_ankle_r"= "q12"\r\n')
        names.append('"D1@q13_l_shoulder_p"= "q13"\r\n')
        names.append('"D1@q14_l_shoulder_r"= "q14"\r\n')
        names.append('"D1@q15_l_elbow"= "q15"\r\n')
        names.append('"D1@q16_r_hip_y"= "q16"\r\n')
        names.append('"D1@q17_r_hip_r"= "q17"\r\n')
        names.append('"D1@q18_r_hip_p"= "q18"\r\n')
        names.append('"D1@q19_r_knee"= "q19"\r\n')
        names.append('"D1@q20_r_ankle_p"= "q20"\r\n')
        names.append('"D1@q21_r_ankle_r"= "q21"\r\n')
        names.append('"D1@q22_r_shoulder_p"= "q22"\r\n')
        names.append('"D1@q23_r_shoulder_r"= "q23"\r\n')
        names.append('"D1@q24_r_elbow"= "q24"\r\n')
        for i in range(self.nq-1):
            f.write(names[i])

    def saveConfig(self, x, savePath):
        '''
        This function is used to generate configuration file with Solidworks in "configuration" interface
        '''
        q = self.generateSwData(x)
        
        import xlsxwriter

        # Create a workbook and add a worksheet.
        workbook = xlsxwriter.Workbook(savePath)
        worksheet = workbook.add_worksheet()

        # Some data we want to write to the worksheet.
        jointName = []
        jointName.append('D1@q0_x')
        jointName.append('D1@q1_y')
        jointName.append('D1@q2_z')
        jointName.append('D1@q3_x')
        jointName.append('D1@q4_y')
        jointName.append('D1@q5_z')
        jointName.append('null')

        jointName.append('D1@q7_l_hip_y')
        jointName.append('D1@q8_l_hip_r')
        jointName.append('D1@q9_l_hip_p')
        jointName.append('D1@q10_l_knee')
        jointName.append('D1@q11_l_ankle_p')
        jointName.append('D1@q12_l_ankle_r')

        jointName.append('D1@q13_l_shoulder_p')
        jointName.append('D1@q14_l_shoulder_r')
        jointName.append('D1@q15_l_shoulder_y')
        jointName.append('D1@q16_l_elbow')

        jointName.append('D1@q17_r_hip_y')
        jointName.append('D1@q18_r_hip_r')
        jointName.append('D1@q19_r_hip_p')
        jointName.append('D1@q20_r_knee')
        jointName.append('D1@q21_r_ankle_p')
        jointName.append('D1@q22_r_ankle_r')

        jointName.append('D1@q23_r_shoulder_p')
        jointName.append('D1@q24_r_shoulder_r')
        jointName.append('D1@q25_r_shoulder_y')
        jointName.append('D1@q26_r_elbow')

        q = q.tolist()

        # Start from the first cell. Rows and columns are zero indexed.
        row = 0
        col = 0
        from sets import Set
        inverse = Set(['D1@q20_r_knee',
                        'D1@q9_l_hip_p'])
        flip = Set(['D1@q10_l_knee',
                        'D1@q23_r_shoulder_p'])
        turn = Set(['D1@q15_l_shoulder_y'])                
        # Iterate over the data and write it out row by row.
        for item, value in zip(jointName, q):

            if item in inverse:
                value = 360 -value
            if item in flip:
                value = 180 + value
            if item in turn:
                value = 90+value
            if item =='null':
                pass
            else:
                worksheet.write(row, col, item)
                worksheet.write(row+1, col, value)
                col += 1

        workbook.close()

    def readConfig(self, filePath):
        '''
        This function is used to read configuration file from Solidworks in "configuration" interface
        '''
        import xlrd

        wb = xlrd.open_workbook(filePath) 
        sheet = wb.sheet_by_index(0)
        joint_values = []
        joint_names = []
        from sets import Set
        inverse = Set(['D1@q20_r_knee',
                        'D1@q9_l_hip_p'])
        flip = Set(['D1@q10_l_knee',
                        'D1@q23_r_shoulder_p'])
        turn = Set(['D1@q15_l_shoulder_y'])
        row =0
        col =0
        for i in range(self.nq):
            if i ==6:
                joint_values.append(0)
                joint_names.append('null')
            else:
                name = sheet.cell_value(row, col)
                value = sheet.cell_value(row+1, col)
                if name in inverse:
                    value = 360-value
                if name in flip:
                    value = 180 + value
                if name in turn:
                    value = 90 + value
                joint_names.append(name)
                joint_values.append(value)
                col +=1
        
        for i in range(3, self.nq):
            if joint_values[i] > 180:
                joint_values[i] -=360

            joint_values[i] *=np.pi/180.
        
        for i in range(3):
            joint_values[i] *=0.001

        joint_values[0] -= 0.0264
        joint_values[1] -= 0.
        joint_values[2] -= 0.50958
        
        # Convert quaternion to rpy
        rpy = np.asarray(joint_values[3:6])
        se3 = pinocchio.SE3.Identity()
        se3.translation = np.asarray(joint_values[0:3])
        se3.rotation = rpyToMatrix(rpy)
        xyzquaternion = pinocchio.SE3ToXYZQUAT(se3).T.tolist()[0]
        joint_values[0:7] = xyzquaternion
        # print(joint_values)

        joint_values = np.asarray(joint_values)
        return joint_values
    def getLimit(self):
        u = []
        l = []
        # l_hip_y
        u.append(np.pi/2)  # fore to inward
        l.append(-np.pi/6) # fore to outward
        # l_hip_r
        u.append(np.pi/4) #lift left
        l.append(-np.pi/4) #lift right
        # l_hip_p
        u.append(np.pi/2) #bend forward
        l.append(-np.pi/6) # bend backward
        # l_knee
        u.append(0) # bend forward impossible
        l.append(-np.pi/2-np.pi/10) # bend backward
        # l_ankle_p
        u.append(np.pi/6) # rear lift
        l.append(-np.pi/2)# fore lift
        # l_ankle_r
        u.append(np.pi/6) # lift right
        l.append(-np.pi/6)# lift left
        # l_shoulder_p
        u.append(np.pi/2) #  arm backward
        l.append(-np.pi)#  arm forward
        # l_shoulder_r
        u.append(np.pi) #  arm left
        l.append(-np.pi/18)#  arm right
        # l_shoulder_y
        u.append(np.pi) #  elbow left
        l.append(-np.pi)#  elbow right
        # l_elbow
        u.append(np.pi/2+np.pi/6) #  elbow bend inward
        l.append(0)#  elbow bend outward impossible

        # r_hip_y
        u.append(np.pi/6)  # fore to outward
        l.append(-np.pi/2) # fore to inward
        # r_hip_r
        u.append(np.pi/4) #lift left
        l.append(-np.pi/4) #lift right
        # r_hip_p
        u.append(np.pi/2) #bend forward
        l.append(-np.pi/6) # bend backward
        # r_knee
        u.append(0) # bend forward impossible
        l.append(-np.pi/2-np.pi/10) # bend backward
        # r_ankle_p
        u.append(np.pi/6) # rear lift
        l.append(-np.pi/2)# fore lift
        # r_ankle_r
        u.append(np.pi/6) # lift right
        l.append(-np.pi/6)# lift left
        # r_shoulder_p
        u.append(np.pi/2) #  arm backward
        l.append(-np.pi)#  arm forward
        # r_shoulder_r
        u.append(np.pi) #  arm right
        l.append(-np.pi/18)#  arm left
        # r_shoulder_y
        u.append(2*np.pi) #  elbow left
        l.append(-2*np.pi)#  elbow right
        # r_elbow
        u.append(np.pi/2+np.pi/6) #  elbow bend inward
        l.append(0)#  elbow bend outward impossible

        return l, u

    def getVelLimit(self):
        velLower = []
        velUpper = []

        mx28=5.76
        mx64=6.59
        mx106=4.71
        velLower.append(-1*mx64)
        velUpper.append(mx64)
        for i in range(5):
            velLower.append(-1*mx106)
            velUpper.append(mx106)
        for i in range(2):
            velLower.append(-1*mx64)
            velUpper.append(mx64) 
        for i in range(2):
            velLower.append(-1*mx28)
            velUpper.append(mx28)
            
        velLower.append(-1*mx64)
        velUpper.append(mx64)
        for i in range(5):
            velLower.append(-1*mx106)
            velUpper.append(mx106) 

        for i in range(2):
            velLower.append(-1*mx64)
            velUpper.append(mx64)
        for i in range(2):
            velLower.append(-1*mx28)
            velUpper.append(mx28)
        return velLower, velUpper

    def getTorqueLimit(self):
        torqueLower = []
        torqueUpper = []

        torqueLower.append(-6.0)
        torqueUpper.append(6.0)
        for i in range(5):
            torqueLower.append(-8.4)
            torqueUpper.append(8.4)
        for i in range(2):
            torqueLower.append(-6.0)
            torqueUpper.append(6.0) 
        for i in range(2):
            torqueLower.append(-2.5)
            torqueUpper.append(2.5)
            
        torqueLower.append(-6.0)
        torqueUpper.append(6.0)
        for i in range(5):
            torqueLower.append(-8.4)
            torqueUpper.append(8.4) 

        for i in range(2):
            torqueLower.append(-6.0)
            torqueUpper.append(6.0)
        for i in range(2):
            torqueLower.append(-2.5)
            torqueUpper.append(2.5)
        return torqueLower, torqueUpper
class ReferenceState():
    def __init__(self, state):
        self.state_names = []
        
        self.state_names.append('x')
        self.state_names.append('y')
        self.state_names.append('z')
        self.state_names.append('rx')
        self.state_names.append('ry')
        self.state_names.append('rz')
        self.state_names.append('l_hip_y')
        self.state_names.append('l_hip_r')
        self.state_names.append('l_hip_p')
        self.state_names.append('l_knee')
        self.state_names.append('l_ankle_p')
        self.state_names.append('l_ankle_r')
        self.state_names.append('l_shoulder_p')
        self.state_names.append('l_shoulder_r')
        self.state_names.append('l_shoulder_y')
        self.state_names.append('l_elbow')
        self.state_names.append('r_hip_y')
        self.state_names.append('r_hip_r')
        self.state_names.append('r_hip_p')
        self.state_names.append('r_knee')
        self.state_names.append('r_ankle_p')
        self.state_names.append('r_ankle_r')
        self.state_names.append('r_shoulder_p')
        self.state_names.append('r_shoulder_r')
        self.state_names.append('r_shoulder_y')
        self.state_names.append('r_elbow')

        self.state_names.append('v_x')
        self.state_names.append('v_y')
        self.state_names.append('v_z')
        self.state_names.append('v_rx')
        self.state_names.append('v_ry')
        self.state_names.append('v_rz')
        self.state_names.append('v_l_hip_y')
        self.state_names.append('v_l_hip_r')
        self.state_names.append('v_l_hip_p')
        self.state_names.append('v_l_knee')
        self.state_names.append('v_l_ankle_p')
        self.state_names.append('v_l_ankle_r')
        self.state_names.append('v_l_shoulder_p')
        self.state_names.append('v_l_shoulder_r')
        self.state_names.append('v_l_shoulder_y')
        self.state_names.append('v_l_elbow')
        self.state_names.append('v_r_hip_y')
        self.state_names.append('v_r_hip_r')
        self.state_names.append('v_r_hip_p')
        self.state_names.append('v_r_knee')
        self.state_names.append('v_r_ankle_p')
        self.state_names.append('v_r_ankle_r')
        self.state_names.append('v_r_shoulder_p')
        self.state_names.append('v_r_shoulder_r')
        self.state_names.append('v_r_shoulder_y')
        self.state_names.append('v_r_elbow')

        self.reference_state = state.copy()
        self.stateWeights = np.array([0.] * 3 + [0.] * 3 
                                + [0.01]*6
                                + [0.01]*4
                                + [0.01]*6
                                + [0.01] * 4 
                                + [10.] * 26)
        self.config_id = {}
        self.weight_id = {}
        self.value = {}
        for i, (name) in enumerate(self.state_names):
            
            if i>5:
                j = i +1
            else:
                j = i
            self.config_id[name] = j
            self.weight_id[name] = i
            self.value[name] = [np.asscalar(self.reference_state[j]), np.asscalar(self.stateWeights[i])]

    @property
    def value(self):
        return self._value

    # @property
    # def weight(self):
    #     return self._weight

    def update(self):
        for name in self.state_names:
            config_id = self.config_id[name]
            weight_id = self.weight_id[name]
            self.reference_state[config_id] = self.value[name][0]
            self.stateWeights[weight_id] = self.value[name][1]
            

def main():
    VisualModel(display=True)
if __name__ == "__main__":
    main()