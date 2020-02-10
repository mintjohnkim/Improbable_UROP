import time

import airobot as ar
import numpy as np
from airobot import Robot
from airobot.utils.common import euler2quat

import pybullet as p
import cv2
import math

def record(robot, out):
	img = robot.cam.get_images(get_rgb=True, get_depth=False)[0]
	out.write(np.array(img))


def main():
	"""
	This function shows an example of block stacking.
	"""
	np.set_printoptions(precision=4, suppress=True)
	robot = Robot('ur5e_2f140', pb=True, pb_cfg={'gui': False})
	success = robot.arm.go_home()
	if not success:
		ar.log_warn('Robot go_home failed!!!')

	#setup cam
	robot_base = robot.arm.robot_base_pos
	robot.cam.setup_camera(focus_pt=robot_base, dist=3, yaw=55, pitch=-30, roll=0)
	out = cv2.VideoWriter('pickandplace.mp4', cv2.VideoWriter_fourcc(*'mp4v'), 30, (640,480)) 

	img = robot.cam.get_images(get_rgb=True, get_depth=False)[0]
	out.write(np.array(img))

	ori = euler2quat([0, 0, np.pi / 2])
	robot.pb_client.load_urdf('table/table.urdf',
							  [.5, 0, 0.4],
							  ori,
							  scaling=0.9)
	box_size = 0.05
	box_id1 = robot.pb_client.load_geom('box', size=box_size,
										mass=1,
										base_pos=[.5, 0.12, 1.0],
										rgba=[1, 0, 0, 1])
	robot.arm.eetool.open()
	record(robot, out)

	obj_pos = robot.pb_client.get_body_state(box_id1)[0]
	move_dir = obj_pos - robot.arm.get_ee_pose()[0]
	move_dir[2] = 0
	eef_step = 0.025

	for i in range(3):
		robot.arm.move_ee_xyz(move_dir/3, eef_step=eef_step)
		record(robot, out)

	move_dir = np.zeros(3)
	move_dir[2] = obj_pos[2] - robot.arm.get_ee_pose()[0][2]

	for i in range(4):
		robot.arm.move_ee_xyz(move_dir/4, eef_step=eef_step)
		record(robot, out)

	robot.arm.eetool.close(wait=False)
	record(robot, out)

	for i in range(10):
		robot.arm.move_ee_xyz([0, 0, 0.03], eef_step=eef_step)
		record(robot, out)

	for i in range(10):
		robot.arm.move_ee_xyz([0.025, 0, 0.0], eef_step=eef_step)
		record(robot, out)

	for i in range(10):
		robot.arm.move_ee_xyz([0, 0, -0.03], eef_step=eef_step)
		record(robot, out)

	robot.arm.eetool.open()
	record(robot, out)

	time.sleep(1)
	record(robot, out)

	for i in range(10):
		robot.arm.move_ee_xyz([0, 0, 0.03], eef_step=eef_step)
		record(robot, out)

	time.sleep(3)
	record(robot, out)

	out.release()


if __name__ == '__main__':
	main()
