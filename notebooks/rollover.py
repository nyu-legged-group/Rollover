import os
import inspect
import time
currentdir = os.path.dirname(os.path.abspath(
	inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(1, parentdir+'/src')

savepath = parentdir + '/data/'+time.strftime("%m_%d")+'/rollover/'
if not os.path.exists(savepath):
	os.makedirs(savepath)
import warnings
warnings.filterwarnings('ignore')
from visualizer import VisualModel
from simulator import HumanoidSimulator
from simple_planner import SimpleKneePlanner
import pinocchio
print(pinocchio.__version__)
import numpy as np
pinocchio.switchToNumpyMatrix()
# Define simulation steps
horizon_length = 5000
time_step = 1e-3

# Define control trajectory steps 
ctrl_time_step = 8e-3
print('total simulation steps: %d, simulation time interval %.3f s'%(horizon_length, time_step))
# Load pinocchio model
m = VisualModel(display=False)
x0, nq, nv, na = m.x0, m.nq, m.nv, m.na
lower, upper = m.getLimit()
torqueLower, torqueUpper = m.getTorqueLimit()

simpleKneePlanner = SimpleKneePlanner(x0, nq, nv, na, horizon_length)
kneeTraj = simpleKneePlanner.forward()
s = HumanoidSimulator(horizon_length, display=False, timeStep=time_step, view='left')
s.initPose(x0, nq, nv, na)
_, _, _, _,_, _, x1, handLength, timeLength, initCoM, initCoMv,initKneeAngle, peArr, keArr  = s.simulate(m, kneeTraj, kneeTraj)
ctrl_horizon_length = timeLength//int(ctrl_time_step/time_step)
print('fall simulated knots: %d'%(timeLength))
print('fall control knots: %d, control time interval %.3f s'%(ctrl_horizon_length, ctrl_time_step))

from simulator import HumanoidSimulator
from simple_planner import SimpleHipPlanner

# To differentiate from bimanual, here we are using a hip planner
simpleHipPlanner = SimpleHipPlanner(x1, nq, nv, na, 100)
tauTraj = simpleHipPlanner.forward()

ss3 = HumanoidSimulator(horizon_length, display=False,timeStep=time_step, view='side')
ss3.initPose(x0, nq, nv, na)
forceArr, comArr, posArr, torqueArr, qArr, forcePose, tauArr, _, _, _, _ = ss3.simulateOptTraj(m, kneeTraj, tauTraj,ctrlTimeStep=ctrl_time_step)
from simple_planner import SimpleKneePlanner
from rollover_planner import RolloverPlanner

timeLengths= []
for i in range(10, 40):
	for j in range(10, 40):
		for k in range(10, 40):
			timeLengths.append([i,j,k])
from simulator import HumanoidSimulator
from simple_planner import SimpleRolloverTrajWrapper

def optimal_sim(timeLength):
	rolloverPlanner = RolloverPlanner(x1, nq, nv, na, ctrl_horizon_length, contact_index=0, timeStep=ctrl_time_step, display=False)
	tauRolloverTraj = rolloverPlanner.forward(m, handLength.copy(), timeLength)
	tauRolloverTraj_index = rolloverPlanner.contact_index
	# rolloverPlanner.saveTraj(np.matrix(tauRolloverTraj).T, savepath+'value.csv')
	# rolloverPlanner.saveSwConfig(savepath+'equations.txt')
#           # print('rollover optimal trajectory length(columns): %d'%tauRolloverTraj.shape[1])

	# We wrap up the trajectory by adding an additional trajectory
	finish_ctrl_horizon_length = 150
	simpleRolloverTrajWrapper = SimpleRolloverTrajWrapper(x0, nq, nv, na, finish_ctrl_horizon_length)
	newtauRolloverTraj = simpleRolloverTrajWrapper.forward(tauRolloverTraj)

	# Simulate optimal rollover trajectory
	ss = HumanoidSimulator(horizon_length, display=False,timeStep=time_step, view='side')
	ss.initPose(x0, nq, nv, na)
	forceArr, comArr, posArr, torqueArr, qArr, forcePose, tauArr, peArr, keArr, contactTime, forceKnot = ss.simulateOptTraj(m, 
																   kneeTraj, 
																   newtauRolloverTraj,
																   ctrlTimeStep=ctrl_time_step,
																   speed=1.)
	max_force = np.amax(forceArr)
	# if max_force < min_force:
	#     min_force = max_force
	#     min_timeLength=timeLength
	# print('global current parameter: [ %d, %d, %d]'%(min_timeLength[0], min_timeLength[1], min_timeLength[2]))
	# print('global min force is %.2f N'%min_force)
	# print('current parameter: [ %d, %d, %d]'%(timeLength[0], timeLength[1], timeLength[2]))
	# print('maximum force is %.2f N'%np.amax(forceArr))
	result = []
	result.append(timeLength)
	result.append(np.amax(forceArr))
	return result
import concurrent.futures
with concurrent.futures.ProcessPoolExecutor() as executor:
	results = executor.map(optimal_sim, timeLengths)
	min_force= 1.e3
	min_timeLength = [0.,0.,0.]
	for f in concurrent.futures.as_completed(results):
		max_force = f[1]
		if max_force < min_force:
			min_force = max_force
			min_timeLength=f[0]
	print('global current parameter: [ %d, %d, %d]'%(min_timeLength[0], min_timeLength[1], min_timeLength[2]))
	print('global min force is %.2f N'%min_force)
# posInfo = []
# posInfo.append(posArr)
# posInfo.append(lower)
# posInfo.append(upper)

# torqueInfo = []
# torqueInfo.append(torqueArr)
# torqueInfo.append(torqueLower)
# torqueInfo.append(torqueUpper)

# cmpInfo = []
# cmpInfo.append(posArr)
# cmpInfo.append(tauArr)
# cmpInfo.append(lower)
# cmpInfo.append(upper)

# ss.plot(forceArr,comArr,posInfo, torqueInfo,savepath, time_step, cmpInfo=cmpInfo)