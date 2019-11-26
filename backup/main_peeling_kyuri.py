import numpy as np
import os
import time
 
import EnvSetupVar
EnvSetupVar.Need_RealSense_R = True
#EnvSetupVar.Need_RealSense_L = True
EnvSetupVar.Need_PA10_BOTH = True
#EnvSetupVar.Need_Gripper_R = True
#EnvSetupVar.Need_Gripper_L = True
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

#raw_input("Camera setting finished\n Open gripper?")

raw_input("move arm(standby)")
# ARM_R.StandbyARM()
# ARM_L.StandbyARM()

print "Arm has been moved to the initial position."

Gripper_L.close(250)
Gripper_R.close(250)

# ARM_R.simulation_flag = False
# ARM_L.simulation_flag = False
# ARM_R.debug_mode = True
# ARM_L.debug_mode = True

# Define consts
toDeg = 180.0 / np.pi
toRad = np.pi / 180.0
theta = np.deg2rad(30) # the angle between two normals
sigma = np.deg2rad(120)  # rotation angle of one time
band = 0.13
yos = 0.005
z_axis = [0, 0, 1]
base_xyzabc = [ 0.65, -0.82, 0.3, 0, 0, 0]
# r_pos1 = [0.5-0.055, -0.6 , 0.22 + band-0.0515 , 0, 0, 0]
# r_pos2 = [0.5-0.055, -0.75, 0.18-0.0515       , np.pi / 4, 0, 0]
# r_pos3 = [0.5-0.055, -0.53, 0.16 + band-0.0515, -np.pi / 18, 0, 0]
r_pos1 = [0.5-0.055, -0.6 , 0.25 + band-0.0515 , 0, 0, 0]
r_pos2 = [0.5-0.055, -0.78, 0.20-0.0515       , np.pi / 4, 0, 0]
r_pos3 = [0.5-0.055, -0.50, 0.20 + band-0.0515, -np.pi / 18, 0, 0]
r_pos4 = [0.35, -0.50, 0.20 + band, 0, -np.pi / 9, -np.pi / 2]
r_pos5 = [0.6, -0.60, 0.20 + band, 0, np.pi / 9, np.pi / 2]
# ARM_R.otc_setfMoveGain(gain=0.05)


raw_input("begin")

"""# using to find food by PCA-algorithm(but had not used so long)
on = [0.55, -0.45, 0.22, 0, 0, 0]

raw_input("get point cloud")
ARM_R.move_rmrc(on)
matrix_c2b = (ARM_R.Tc2b).tolist()
print "matrix_c2b",matrix_c2b

PointCloudProcess.merge_pointCloud(matrix_c2b) #get 3D point cloud
count,point_coor = PointCloudProcess.transform_tc2b(matrix_c2b)

ARM_R.StandbyARM()
j[2] = 0.32
ARM_L.move_rmrc(j)
info_d = PointCloudProcess.xyzabc_mean_pc(base_xyzabc,"+z+x")
print "info_d: ",info_d
"""#

ARM_L.mode_rmrc()
ARM_R.mode_rmrc()

# current_pose_r = ARM_R.t_FRAME_now.xyzabc()
# current_pose_r[1] -= 0.1
# current_pose_r[2] += 0.02
# ARM_R.move_rmrc(current_pose_r)



"""# hold food and take it to fixed place
center_up = [0.65, 0, 0.35, 0, 0, 0]
center_down = [0.65, 0, 0.135, 0, 0, 0]
ARM_L.move_rmrc(center_up)
raw_input("gripper close")
Gripper_L.close(250)
# ARM_L.move_rmrc(center_down)
# ARM_L.move_rmrc(center_up)

ARM_L.move_rmrc([0.55, -0.47, 0.22+band, -90*toRad, 0, np.pi/2])
ARM_L.move_rmrc([0.55, -0.47, 0.12+band, -90*toRad, 0, np.pi])

ARM_L.mode_fMove()
ARM_L.otc_setFOffset()
moddle_p = [0.55, -0.47, 0.06, -75*toRad, 0, np.pi]
ARM_L.move_fMoveLim(moddle_p,1, target_force=1.0, ctrl_axis=[0, 0, 1, 0, 0, 0],v_max=0.003)
ARM_L.mode_rmrc()
"""#


# make position and normal transfer xyzabc
def xyzabc(pl_point, pl_normal):
	# Orthonormal basis
	z = geo.VECTOR(vec=pl_normal)
	x = geo.VECTOR(vec=[0, -1, z[1]/z[2]])
	y = geo.VECTOR(vec=[z[2]*x[1] - z[1]*x[2], 0, 0])
	for i in (0, 1, 2):
		x[i] /= x.abs()
		y[i] = np.abs(y[i]/y.abs())
		z[i] /= z.abs()

	mat = geo.MATRIX(mat=[x, y, z])
	mat = mat.trans()
	frame = geo.FRAME(mat=mat, vec=pl_point)
	gesture = frame.xyzabc()
	return gesture


# calculate angle of both normals
def normal_angle(pre, ne):
	a_b = pre[0]*ne[0] + pre[1]*ne[1] + pre[2]*ne[2]
	a_babs = np.sqrt(pre[0]**2 + pre[1]**2 + pre[2]**2) * np.sqrt(ne[0]**2 + ne[1]**2 + ne[2]**2)
	cos_th = a_b/a_babs
	th_anl = np.arccos(cos_th)
	return th_anl


def bdy_normal_avg(_boundary_normal, _bdy_sizes):
	avg = [0, 0, 0]
	for i in range(_bdy_sizes):
		avg[0] += _boundary_normal[i][0]
		avg[1] += _boundary_normal[i][1]
		avg[2] += _boundary_normal[i][2]
	avg[0] = avg[0] / _bdy_sizes
	avg[1] = avg[1] / _bdy_sizes
	avg[2] = avg[2] / _bdy_sizes
	_sigma = normal_angle(avg, z_axis)
	return _sigma


# calculate average of normals in a period
def normalAve(index1, index2):
	period_points = index2 - index1
	normalSum = [0, 0, 0]
	normalAveg = [0, 0, 0]
	for j in range(index1, index2):
		normalSum[0] += pl_normal[j][0]
		normalSum[1] += pl_normal[j][1]
		normalSum[2] += pl_normal[j][2]
	normalAveg[0] = normalSum[0]/period_points
	normalAveg[1] = normalSum[1]/period_points
	normalAveg[2] = normalSum[2]/period_points
	return normalAveg


raw_input("start peeling")

# ARM_R.simulation_flag = True

# tool coordinate have been calibrated
# ARM_R.set_tool_xyz(-0.053, -0.002, -0.034)
ARM_R.set_tool_xyz(-0.055, -0.002, -0.0515)

for k in range(30):
	"""
	ARM_R.move_rmrc(r_pos1)
	matrix_c2b_rgb = (ARM_R.Tc2b).tolist()
	PointCloudProcess.color_pointCloud(matrix_c2b_rgb)
	normal, bdy_sizes, stop = PointCloudProcess.boundaryNormal(matrix_c2b_rgb)
	print "stop: ", stop
	if stop:
		print "stop is true"
		break
	print "bdy_sizes:", bdy_sizes

	boundary_normal = [[0 for j in range(3)] for i in range(0, bdy_sizes)]
	for i in range(0, bdy_sizes):
		boundary_normal[i][0] = normal[i][0]
		boundary_normal[i][1] = normal[i][1]
		boundary_normal[i][2] = normal[i][2]
	sigma = bdy_normal_avg(boundary_normal, bdy_sizes)
	file_sigma = open('sigma.txt', mode='a')
	file_sigma.write("large3 sigma "+str(k)+": "+str(np.rad2deg(sigma))+"\n")
	file_sigma.close()
	print "=============================sigma", k, ":", np.rad2deg(sigma)
	"""
	current_pose_l = ARM_L.t_FRAME_now.xyzabc()
	current_pose_l[2] += 0.008
	current_pose_l[5] += sigma
	ARM_L.move_rmrc(current_pose_l)

	ARM_L.mode_fMove()
	ARM_L.otc_setFOffset()
	current_pose_l[2] = 0.06
	ARM_L.move_fMoveLim(current_pose_l, 1, target_force=0.5, ctrl_axis=[0, 0, 1, 0, 0, 0], v_max=0.003)
	ARM_L.mode_rmrc()
	time.sleep(1)

	for i in range(1):
		ARM_R.move_rmrc(r_pos1)
		matrix_c2b = (ARM_R.Tc2b).tolist()
		PointCloudProcess.merge_pointCloud(matrix_c2b)
	matrix_c2b = (ARM_R.Tc2b).tolist()
	p_sizes, point_coor = PointCloudProcess.apple_PC(matrix_c2b)
	rei, nor_sort = PointCloudProcess.transform_tc2b(matrix_c2b)
	print "p_sizes:", p_sizes

	pl_point = [[0 for j in range(3)] for i in range(0, p_sizes)]
	pl_normal = [[0 for j in range(3)] for i in range(0, p_sizes)]

	for i in range(0, p_sizes):
		pl_point[i][0] = point_coor[i][0]
		pl_point[i][1] = point_coor[i][1]
		pl_point[i][2] = point_coor[i][2]
		pl_normal[i][0] = 0
		pl_normal[i][1] = nor_sort[i][1]
		pl_normal[i][2] = nor_sort[i][2]
	node = []
	node.append(6)
	node.append(p_sizes-5)

	for i in node:
		print i
		print "xyz: ", pl_point[i]
		print "normal: ", pl_normal[i]

	# ARM_R.set_tool_xyz(-0.055, -0.002, -0.0515)

	print "begin to peel ", k, "times"
	for i in range(1):
		pre = node[i]
		beh = node[i + 1]
		pose = (pre + beh) / 3 - 3
		normalAveg = normalAve(pre, beh)
		previous = xyzabc(pl_point[pre], normalAveg)
		behind = xyzabc(pl_point[beh], normalAveg)
		force_ori = xyzabc(pl_point[pose], normalAveg)
		# previous[1] -= yos
		# previous[2] += 0.006
		# touch_p = previous[:]

		if i == 0:
			# ARM_R.simulation_flag = False
			ARM_R.move_rmrc(previous)
			print "reach first point of ", k, "times"
			# behind[1] -= yos   #0.004 +
			# ARM_R.otc_setFOffset()
			ARM_R.mode_fMove()
			# touch_p[1] = -0.60
			ARM_R.move_fMoveLim(previous, 1, target_force=1.0, ctrl_axis=[0, 0, 1, 0, 0, 0], v_max=0.001)
			# ARM_R.otc_setFOffset()
			ARM_R.otc_setForceCoordinate(behind)
			ARM_R.move_fMoveTo(behind, target_force=5.0, ctrl_axis=[0, 1, 0, 0, 0, 0], v_max=0.005)
		else:
			# ARM_R.simulation_flag = False
			# ARM_R.getState()
			current_pose = ARM_R.t_FRAME_now.xyzabc()
			previous = [current_pose[0], current_pose[1], current_pose[2], previous[3], previous[4], previous[5]]
			ARM_R.move_rmrc(previous)
			# behind[1] += yos - 0.002
			ARM_R.otc_setFOffset()
			# ARM_R.otc_setfMoveGain(gain=0.05)
			ARM_R.mode_fMove()
			if (i == len(node) - 3):
				behind[1] -= yos- 0.001
				ARM_R.move_fMoveTo(behind, target_force=3.0, ctrl_axis=[0, 1, 0, 0, 0, 0], v_max=0.002)
			ARM_R.otc_setForceCoordinate(behind)
			ARM_R.move_fMoveTo(behind, target_force=3.0, ctrl_axis=[0, 1, 0, 0, 0, 0], v_max=0.003)
		ARM_R.mode_rmrc()
		print i + 1, "preiod finished of ", k, "times"
	print k, "times finished"

	current_pose_r = ARM_R.t_FRAME_now.xyzabc()
	current_pose_r[1] -= 0.1
	current_pose_r[2] -= 0.005
	ARM_R.move_rmrc(current_pose_r)

	# if (current_pose_l[5]<-np.pi/18*16):
	# 	print "have rotated 360 degree, peeling have done. "
	# 	break
	"""
	ARM_R.move_rmrc(r_pos1)
	matrix_c2b_rgb = (ARM_R.Tc2b).tolist()
	print "matrix_c2b_rgb: ", matrix_c2b_rgb
	PointCloudProcess.color_pointCloud(matrix_c2b_rgb)
	normal, bdy_sizes, stop = PointCloudProcess.boundaryNormal(matrix_c2b_rgb)
	if stop:
		print "stop is true"
		break
	print "bdy_sizes:", bdy_sizes

	boundary_normal = [[0 for j in range(3)] for i in range(0, bdy_sizes)]
	for i in range(0, bdy_sizes):
		boundary_normal[i][0] = normal[i][0]
		boundary_normal[i][1] = normal[i][1]
		boundary_normal[i][2] = normal[i][2]
	sigma = bdy_normal_avg(boundary_normal, bdy_sizes)
	file_sigma = open('sigma.txt', mode='a')
	file_sigma.write("large3 sigma "+str(k)+": "+str(np.rad2deg(sigma))+"\n")
	file_sigma.close()
	print "=====================================sigma", k, ":", np.rad2deg(sigma)
	"""
raw_input("gripper free")
Gripper_L.open(100)
Gripper_R.open(100)

Gripper_L.free()
Gripper_R.free()

raw_input("it's over")

ARM_R.StandbyARM(0.20)
ARM_L.StandbyARM(0.20)