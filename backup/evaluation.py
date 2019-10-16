import numpy as np
import os
import time
from point360 import *
import EnvSetupVar

EnvSetupVar.Need_RealSense_R = True
# EnvSetupVar.Need_RealSense_L = True
EnvSetupVar.Need_PA10_BOTH = True
# EnvSetupVar.Need_Gripper_R = True
# EnvSetupVar.Need_Gripper_L = True
EnvSetupVar.Need_Gripper_BOTH = True
EnvSetupVar.Need_PA10_IN_WORLD = True
EnvSetupVar.Tl2world = [0, 0, 0, 0, 0, 0]
EnvSetupVar.SubProcess = os.fork()

from EnvSetup import *
# from realsenseD435Class import RS_CH1, RS_CH2, RS_CH3, RS_CH4
from realsenseD435Class import RS_COLOR, RS_DEPTH, RS_IRL, RS_IRR, RS_PC

raw_input("Start!")
print "Open Camera"
RealSense_R.open(RS_COLOR | RS_DEPTH | RS_PC)

print "Enable Post Process"
RealSense_R.ControlProcess.EnablePostProcess(True)

print "Start Camera Stream"
RealSense_R.ControlProcess.StartStream()

raw_input("move arm(standby)")
ARM_R.StandbyARM()
ARM_L.StandbyARM()

r_pos1 = [0.5, -0.75, 0.20, 0, 0, 0]
r_pos2 = [0.5, -0.92, 0.12, np.deg2rad(75), 0, 0]
r_pos3 = [0.5, -0.60, 0.12, -np.deg2rad(75), 0, 0]
r_pos4 = [0.45, -0.75, 0.20, 0, -np.pi/9, 0]
r_pos5 = [0.65, -0.75, 0.20, 0,  np.pi/4, 0]
pose = [r_pos2, r_pos3, r_pos4, r_pos5, r_pos1]
label = ["small", "medium", "large"]

for j in range(30):
	for i in pose:
		ARM_R.move_rmrc(i)
		matrix_c2b = (ARM_R.Tc2b).tolist()
		PointCloudProcess.color_pointCloud(matrix_c2b)
	matrix_c2b = (ARM_R.Tc2b).tolist()
	pulp, apple = PointCloudProcess.evaluation(matrix_c2b)
	f = open('evaluation.txt', mode='a')
	f.write("pulp  apple "+str(j)+": "+str(pulp)+"  "+str(apple)+"\n")
	f.close()
	print "pulp", j,": ", pulp, "apple", j, ": ",apple
	raw_input("next apple")


