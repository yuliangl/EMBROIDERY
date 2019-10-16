####################################################
# python -- toolCoordCalibration.py
# calibration of tool coordinate after tool change
####################################################

from point360 import *

import EnvSetupVar
#EnvSetupVar.Need_RealSense_R = True
# EnvSetupVar.Need_RealSense_L = True
EnvSetupVar.Need_PA10_R = True
# EnvSetupVar.Need_PA10_BOTH = True
# EnvSetupVar.Need_Gripper_R = True
# EnvSetupVar.Need_Gripper_L = True
# EnvSetupVar.Need_Gripper_BOTH = True
EnvSetupVar.Need_PA10_IN_WORLD = True
EnvSetupVar.Tl2world = [0, 0, 0, 0, 0, 0]
EnvSetupVar.SubProcess = os.fork()


from EnvSetup import *
#from realsenseD435Class import RS_CH1, RS_CH2, RS_CH3, RS_CH4
#from realsenseD435Class import RS_COLOR, RS_DEPTH, RS_IRL, RS_IRR, RS_PC
### Import the environment setup variables
"""
raw_input("Start!")
print "Open Camera"
RealSense_R.open(RS_DEPTH | RS_PC)

print "Enable Post Process"
RealSense_R.ControlProcess.EnablePostProcess(True)

print "Start Camera Stream"
RealSense_R.ControlProcess.StartStream()
"""
# Define consts.

toDeg = 180.0 / np.pi
toRad = np.pi / 180.0

ARM_R.StandbyARM()
ARM_R.simulation_flag = True
ARM_R.debug_mode = True

raw_input("Start program!")
# ARM_R.move_rmrc([0.65, -0.82, 0.30, 0, 0, 0])


# o = np.array([0.65, -0.82, 0.18, 0, 0, 0])
# r = 0.10
# div = 0
# a = point360(o, r, div)

pose1 = [0.65, -0.82, 0.18, 0, 0, 0]
pose2 = [0.65, -0.82, 0.15, 0, 0, 0]
pose3 = [0.65, -0.82, 0.12, 0, 0, 0]
pose4 = [0.65, -0.82, 0.10, 0, 0, 0]

real_pos = [0.701513409614563, -0.7297733426094055, 0.06828352808952332, 0, 0, 0]
# for i in (pose2):

ARM_R.move_rmrc(pose2)
matrix_c2b = (ARM_R.Tc2b).tolist()

raw_input("record")

ARM_R.StandbyARM()

"""
PointCloudProcess.merge_pointCloud(matrix_c2b)

# matrix_c2b = (ARM_R.Tc2b).tolist()
calipoint = PointCloudProcess.toolCoordinateCalibration(matrix_c2b)


raw_input("start calibration")

print calipoint

ARM_R.set_tool_xyz(-0.052, 0, -0.034)

ARM_R.move_rmrc([calipoint[0], calipoint[1], calipoint[2], 0, 0, 0])

raw_input("next")

ARM_R.move_rmrc([0.6, -0.82, 0.25, 0, 0, 0])

ARM_R.move_rmrc([calipoint[0], calipoint[1], calipoint[2], 0, 0, 0])
"""
"""
ARM_R.set_tool_xyz(-0.053, -0.002, -0.034)
real_pos = [0.701513409614563, -0.7297733426094055, 0.06828352808952332, -np.pi/4, 0, -np.pi/2]
ARM_R.move_rmrc(real_pos)
"""