import gym
from gym import spaces

import os, inspect

import pybullet as p
import pybullet_data as pd
import math
import numpy as np
from tqdm import tqdm
import argparse


# get the path to assets
# hw_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)).split('/HW3')[0], 'HW3')
hw_dir = os.path.dirname(os.path.abspath(__file__))
assets_dir = os.path.join(hw_dir, 'assets')


BOX_SIZE = 0.1

TARGET_POSE_FREE = np.array([0.7, 0., 0.])
TARGET_POSE_OBSTACLES = np.array([0.7, -0.1, 0.])
OBSTACLE_CENTRE = np.array([0.6, 0.2, 0.])
OBSTACLE_HALFDIMS = np.array([0.05, 0.25, 0.05])


class PandaImageSpacePushingEnv(gym.Env):

    def __init__(self, debug=False, visualizer=None, include_obstacle=False, render_non_push_motions=True,
                 render_every_n_steps=1, camera_heigh=84, camera_width=84, img_width=32, img_height=32, grayscale=False,
                 done_at_goal=True):
        self.debug = debug
        self.visualizer = visualizer
        self.include_obstacle = include_obstacle
        self.render_every_n_steps = render_every_n_steps
        self.done_at_goal = done_at_goal

        if debug:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT, options="--opengl2")
        p.setAdditionalSearchPath(pd.getDataPath())

        self.episode_step_counter = 0
        self.episode_counter = 0
        self.frames = []

        self.pandaUid = None  # panda robot arm
        self.tableUid = None  # Table where to push
        self.objectUid = None  # Pushing object
        self.targetUid = None  # Target object
        self.obstacleUid = None  # Obstacle object

        self.object_file_path = os.path.join(assets_dir, "objects/cube/cube.urdf")
        self.target_file_path = os.path.join(assets_dir, "objects/cube/cube_target.urdf")
        self.obstacle_file_path = os.path.join(assets_dir, "objects/obstacle/obstacle.urdf")

        # self.init_panda_joint_state = [-0.028, 0.853, -0.016, -1.547, 0.017, 2.4, 2.305, 0., 0.]
        # self.init_panda_joint_state = np.array([0., 0., 0., -np.pi * 0.5, 0., np.pi * 0.5, 0.])
        self.init_panda_joint_state = np.array([-0.00895896, -1.27076799,  0.00843109, -3.03526847,  0.00819568,  1.76413092, -2.36423618])

        self.object_start_pose = None
        self.object_target_pose = None

        self.left_finger_idx = 9
        self.right_finger_idx = 10
        self.end_effector_idx = 11

        self.ik_precision_treshold = 1e-4
        self.max_ik_repeat = 100

        # Robot always face that direction
        # self.fixed_orientation = p.getQuaternionFromEuler([0., -math.pi, math.pi / 2.]) # facing towards y
        self.fixed_orientation = p.getQuaternionFromEuler([0., -math.pi, 0.])  # facing towards x

        self.delta_step_joint = 0.016

        self.close_gripper = False

        self.render_non_push_motions = render_non_push_motions
        self.is_render_on = True

        self.space_limits = [np.array([0.35, -0.35]), np.array([.7, 0.35])]  # xy limits


        # Render camera setting
        self.camera_height = camera_heigh
        self.camera_width = camera_width
        # State camera settings
        self.img_width = img_width
        self.img_height = img_height
        self.grayscale = grayscale
        # self.camera_pos_top = [(self.space_limits[0][0] + self.space_limits[1][0]) / 2,
        self.camera_pos_top = [self.space_limits[1][0]*0.7 + 0.3 * self.space_limits[0][0] ,
                               (self.space_limits[0][1] + self.space_limits[1][1]) / 2,
                               # -0.2]
                               .01]
        # self.camera_orn_top = [90, -60, 180]
        self.camera_orn_top = [90, -95, 0]
        self.camera_dist_top = 0.7

        # p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40,cameraTargetPosition=[0.55, -0.35, 0.2])
        p.resetDebugVisualizerCamera(cameraDistance=self.camera_dist_top, cameraYaw=self.camera_orn_top[0], cameraPitch=self.camera_orn_top[1], cameraTargetPosition=self.camera_pos_top) # TODO: Remove

        self.block_size = BOX_SIZE

        # Motion parameter
        self.lower_z = 0.02
        self.raise_z = 0.3
        # self.push_length = 0.02
        self.push_length = 0.1

        if self.grayscale:
            self.observation_space = spaces.Box(low=0, high=255, shape=(self.img_height, self.img_width, 1), dtype=np.uint8)
        else:
            self.observation_space = spaces.Box(low=0, high=255, shape=(self.img_height, self.img_width, 3), dtype=np.uint8)

        self.object_pose_space = spaces.Box(low=np.array([self.space_limits[0][0], self.space_limits[0][1], -np.pi*0.5]),
                                            high=np.array([self.space_limits[1][0], self.space_limits[1][1], np.pi*0.5]))

        self.action_space = spaces.Box(low=np.array([-1, -np.pi * 0.5, 0]),
                                       high=np.array([1, np.pi * 0.5, 1]))  #


    def reset(self, object_pose=None, render_reset=True):
        self._set_object_positions()
        self.episode_counter += 1
        self.episode_step_counter = 0
        p.resetSimulation()
        p.setGravity(0, 0, -9.8)
        self.is_render_on = render_reset
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)  # we will enable rendering after we loaded everything

        # Add panda to the scene
        self.pandaUid = p.loadURDF(os.path.join(assets_dir, "franka_panda/panda.urdf"), useFixedBase=True)
        for i in range(len(self.init_panda_joint_state)):
            p.resetJointState(self.pandaUid, i, self.init_panda_joint_state[i])

        # Load table
        self.tableUid = p.loadURDF(os.path.join(assets_dir, "objects/table/table.urdf"), basePosition=[0.5, 0, -0.65])
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

        # Load objects
        if object_pose is None:
            object_pose = self.object_start_pose
        else:
            object_pose = self._planar_pose_to_world_pose(object_pose)
        self.objectUid = p.loadURDF(self.object_file_path, basePosition=object_pose[:3], baseOrientation=object_pose[3:], globalScaling=1.)

        # p.changeDynamics(self.objectUid, -1, 2)
        self.targetUid = p.loadURDF(self.target_file_path, basePosition=self.object_target_pose[:3], baseOrientation=self.object_target_pose[3:], globalScaling=1., useFixedBase=True)

        if self.include_obstacle:
            self.obstacleUid = p.loadURDF(self.obstacle_file_path, basePosition=[.6, 0.2, 0], useFixedBase=True)

        p.setCollisionFilterGroupMask(self.targetUid, -1, 0, 0)  # remove collisions with targeUid
        p.setCollisionFilterPair(self.pandaUid, self.targetUid, -1, -1, 0)  # remove collision between robot and target

        # p.changeVisualShape(self.targetUid, -1, rgbaColor=[0.05, 0.95, 0.05, .01])  # Change color for target

        # Set robot to rest configuration
        self.move_robot_rest_configuration()
        # get inital state after reset
        state = self.get_state()
        return state

    def step(self, action):
        # check that the action is valid
        is_action_valid = self.check_action_valid(action)
        if not is_action_valid:
            raise AttributeError(f'Action {action} is not valid. Make sure you provide an action within the action space limits.')
        self.episode_step_counter += 1
        # Enable smooth motion of the robot arm
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        # Convert the action values to the true ranges
        push_location_fraction, push_angle, push_length_fraction = action[0], action[1], action[2]
        push_location = push_location_fraction * self.block_size * 0.5 * 0.95 # we add some small 5% gap so we make sure we do not surpass the border
        push_length = push_length_fraction * self.push_length
        # Perform the action
        self.push(push_location, push_angle, push_length=push_length)
        state = self.get_state()
        reward = 0.
        done = self._is_done(state)
        info = {}
        return state, reward, done, info

    def _is_done(self, state):
        done = not self.observation_space.contains(state)
        at_goal = False
        object_pose = self.get_object_pos_planar().astype(np.float32)
        #check if object within limits:
        in_limits = self.object_pose_space.contains(object_pose)
        if self.include_obstacle:
            at_goal = np.sum((object_pose - TARGET_POSE_OBSTACLES)**2) < 0.01
        else:
            at_goal = np.sum((object_pose - TARGET_POSE_FREE)**2) < 0.01

        if not self.done_at_goal:
            return done or not in_limits

        done = (done or at_goal) and not in_limits
        return done

    def check_action_valid(self, action):
        # check that the action is within the action space limits.
        is_action_valid = np.all((self.action_space.low <= action) & (action <= self.action_space.high))
        is_action_valid = is_action_valid or self.action_space.contains(action)
        return is_action_valid

    def lower_down(self, step_size=0.05):
        current_pos = self.get_end_effector_pos()
        target_pos = current_pos.copy()
        target_pos[-1] = self.lower_z
        self._move_ee_trajectory(target_pos, step_size=step_size)

    def raise_up(self, step_size=0.05):
        current_pos = self.get_end_effector_pos()
        target_pos = current_pos.copy()
        target_pos[-1] = self.raise_z
        self._move_ee_trajectory(target_pos, step_size=step_size)

    def planar_push(self, push_angle, push_length=None, step_size=0.001):
        # push_dir in pl
        if push_length is None:
            push_length = self.push_length
        current_pos = self.get_end_effector_pos()
        target_pos = current_pos + push_length * np.array([np.cos(push_angle), np.sin(push_angle), 0])
        self._move_ee_trajectory(target_pos, step_size=step_size)

    def set_planar_xy(self, xy, theta=0., step_size=0.05):
        current_z = self.get_end_effector_pos()[-1]
        target_pos = np.array([xy[0], xy[1], current_z])
        self._move_ee_trajectory(target_pos, step_size=step_size)

    def move_robot_rest_configuration(self):
        rest_xy = [0.2, 0.]
        self.raise_up()
        self.set_planar_xy(rest_xy, theta=0)

    def push(self, push_location, push_angle, push_length=None):
        current_block_pose = self.get_object_pos_planar()
        theta = current_block_pose[-1]
        if not self.render_non_push_motions:
            self.is_render_on = False
        self.raise_up()
        start_gap = 0.1
        start_xy_bf = np.array([-start_gap, push_location])  # in block frame
        w_R_bf = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        start_xy_wf = w_R_bf @ start_xy_bf + current_block_pose[:2]  # in world frame
        # set xy
        self.set_planar_xy(start_xy_wf, theta=theta)
        # set theta
        self.lower_down()
        self.planar_push(theta, push_length=start_gap-0.015-.5*self.block_size, step_size=0.005) # push until barely touch the block
        self.is_render_on = True
        self.planar_push(push_angle + theta, push_length=push_length, step_size=0.005)
        self.is_render_on = False
        self.move_robot_rest_configuration()
        self.is_render_on = True

    def _move_ee_trajectory(self, target_ee_pos, step_size=0.001):
        # interpolate and do ik along the trajectory to set the ee position in many places
        start_ee_pos = self.get_end_effector_pos()
        goal_error = target_ee_pos - start_ee_pos
        goal_length = np.linalg.norm(goal_error)
        goal_dir = goal_error / (goal_length + 1e-6)
        num_steps = int(goal_length // step_size)
        # move in straigh line
        for step_i in range(num_steps):
            target_ee_pos_i = start_ee_pos + step_size * step_i * goal_dir
            render_step_i = step_i % self.render_every_n_steps == 0
            self._move_robot_ee(target_ee_pos_i, render=render_step_i)
        self._move_robot_ee(target_ee_pos, render=True)

    def _move_robot_ee(self, target_ee_pos, render=True):
        # Ensure good action distance + save computation cost + make delta_step customizable without additional tuning
        distance = math.inf
        repeat_counter = 0
        while distance > self.ik_precision_treshold and repeat_counter < self.max_ik_repeat:
            computed_ik_joint_pos = p.calculateInverseKinematics(self.pandaUid, 11, target_ee_pos,
                                                                 self.fixed_orientation)
            # Set the joints
            p.setJointMotorControlArray(self.pandaUid, list(range(7)), p.POSITION_CONTROL,
                                        list(computed_ik_joint_pos[:-2]), forces=[500.0] * 7)

            # set the fingers
            p.setJointMotorControl2(self.pandaUid, self.right_finger_idx,
                                    p.POSITION_CONTROL, 0., force=450)
            p.setJointMotorControl2(self.pandaUid, self.left_finger_idx,
                                    p.POSITION_CONTROL, 0., force=500)

            p.stepSimulation()

            distance = np.linalg.norm(target_ee_pos - self.get_end_effector_pos())
            repeat_counter += 1

        if self.debug:
            self._debug_step()
            p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
        else:
            if render:
                self.render_frame()

    def get_state(self):
        state = self._render_state()
        # state = self.get_object_pos_planar().astype(np.float32)
        return state

    def get_target_state(self):
        # record the current object state
        current_render_mode = self.is_render_on
        self.is_render_on = False
        current_obj_pose = self.get_object_pos_planar()
        # Generate the state where the object is at the target position
        goal_state = self.reset(self._world_pose_to_planar_pose(self.object_target_pose), render_reset=False)
        # set back the state before setting the goal
        self.reset(current_obj_pose, render_reset=False)
        self.is_render_on = current_render_mode
        return goal_state

    def get_object_pose(self):
        pos, quat = p.getBasePositionAndOrientation(self.objectUid)
        pos = np.asarray(pos)
        quat = np.asarray(quat)
        object_pose = np.concatenate([pos, quat])
        return object_pose

    def get_object_pos_planar(self):
        object_pos_wf = self.get_object_pose()  # in world frame
        object_pos_planar = self._world_pose_to_planar_pose(object_pos_wf)
        return object_pos_planar

    def get_end_effector_pos(self):
        """
        :return: The end effector X, Y, Z positions.
        """
        effector_pos = np.asarray(p.getLinkState(self.pandaUid, self.end_effector_idx)[0])
        return effector_pos

    def get_all_joint_pos(self):
        """
        :return: Vector of the positions of all the joints of the robot.
        """
        joints_pos = []
        for i in range(len(self.init_panda_joint_state)):
            joints_pos.append(p.getJointState(self.pandaUid, i)[0])
        joints_pos = np.array(joints_pos)
        return joints_pos

    def _get_target_pos(self, action):
        """
            Give the target position given the action. This is put in a function to be able to modify how action are
            applied for different tasks.
        :param action: Raw action from the user.
        :return: 3d-array of the X, Y, Z target end effector position.
        """
        dx = action[0] * self.delta_step_joint
        dy = action[1] * self.delta_step_joint
        dz = action[2] * self.delta_step_joint
        current_end_effector_pos = self.get_end_effector_pos()
        # TODO: Set target pose constrained within the space limits (like Panda Haptics)
        target_pos = np.array(
            [current_end_effector_pos[0] + dx, current_end_effector_pos[1] + dy, current_end_effector_pos[2] + dz])
        return target_pos

    def render_image(self, camera_pos, camera_orn, camera_width, camera_height, nearVal=0.01, distance=0.7):
        """
        :param camera_pos:
        :param camera_orn:
        :param camera_width:
        :param camera_height:
        :return:
        """
        view_matrix = p.computeViewMatrixFromYawPitchRoll(cameraTargetPosition=camera_pos,
                                                          distance=distance,
                                                          yaw=camera_orn[0],
                                                          pitch=camera_orn[1],
                                                          roll=camera_orn[2],
                                                          upAxisIndex=2)

        proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                   aspect=float(camera_width) / camera_height,
                                                   nearVal=nearVal,
                                                   farVal=100.0)

        (_, _, px, _, _) = p.getCameraImage(width=camera_width,
                                            height=camera_height,
                                            viewMatrix=view_matrix,
                                            projectionMatrix=proj_matrix,
                                            renderer=p.ER_BULLET_HARDWARE_OPENGL,
                                            flags=p.ER_NO_SEGMENTATION_MASK,
                                            shadow=0)

        rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(rgb_array, (camera_height, camera_width, 4))
        rgb_array = rgb_array[:, :, :3]
        rgb_array = np.moveaxis(rgb_array, [0, 1, 2], [1, 2, 0])
        return rgb_array

    def _debug_step(self):
        """
        Add debug code here.
        :return:
        """
        # Here we would add text and bounding boxed to the debug simulation
        p.removeAllUserDebugItems()

    def _set_object_positions(self):
        # set object initial position and final position
        # self.object_start_pos = self.cube_pos_distribution.sample()
        if self.include_obstacle:
            # with obstacles
            object_start_pose_planar = np.array([0.4, 0., -np.pi * 0.2])
            object_target_pose_planar = TARGET_POSE_OBSTACLES
        else:
            # free of obstacles
            object_start_pose_planar = np.array([0.4, 0., np.pi * 0.2])
            object_target_pose_planar = TARGET_POSE_FREE
        self.object_start_pose = self._planar_pose_to_world_pose(
            object_start_pose_planar)  # self.cube_pos_distribution.sample()
        self.object_target_pose = self._planar_pose_to_world_pose(object_target_pose_planar)

    def _planar_pose_to_world_pose(self, planar_pose):
        theta = planar_pose[-1]
        plane_z = 0
        world_pos = np.array([planar_pose[0], planar_pose[1], plane_z])
        quat = np.array([0., 0., np.sin(theta * 0.5), np.cos(theta * 0.5)])
        world_pose = np.concatenate([world_pos, quat])
        return world_pose

    def _world_pose_to_planar_pose(self, world_pose):
        # compute theta
        quat = world_pose[3:]
        R = quaternion_matrix(quat)[:3, :3]
        x_axis = R @ np.array([1., 0., 0.])
        theta = np.arctan2(x_axis[1], x_axis[0])
        planar_pose = np.array([world_pose[0], world_pose[1], theta])
        return planar_pose

    def render_frame(self):
        if self.debug:
            pass
        elif self.visualizer is not None:
            if self.is_render_on:
                rgb_img = self.render_image(camera_pos=[0.55, -0.35, 0.2],
                                            camera_orn=[0, -40, 0],
                                            camera_width=self.camera_width,
                                            camera_height=self.camera_height,
                                            distance=1.5)
                rgb_img = rgb_img.transpose(1, 2, 0)
                self.frames.append(rgb_img)
                if self.visualizer is not None:
                    self.visualizer.set_data(rgb_img)
        else:
            pass

    def _render_state(self):
        state_img = self.render_image(camera_pos=self.camera_pos_top,
                                            camera_orn=self.camera_orn_top,
                                            camera_width=self.img_width,
                                            camera_height=self.img_height,
                                            distance=self.camera_dist_top)
        state_img = state_img.transpose(1, 2, 0)  # (h, w, num_channels)
        if self.grayscale:
            # Do the grayscale transformation
            state_img = state_img[:, :, 0] * 0.2989 + state_img[:, :, 1] * 0.587 + \
                        state_img[:, :, 2] * 0.114
            state_img = np.expand_dims(state_img, axis=2)
            state_img = state_img.astype(np.uint8)

        return state_img


_EPS = np.finfo(float).eps * 4.0


def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.
1176
1177      >>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
1178      >>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
1179      True
1180
1181      """
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3], 0.0),
        (q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3], 0.0),
        (q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1], 0.0),
        (0.0, 0.0, 0.0, 1.0)
    ), dtype=np.float64)


# DEBUG:
if __name__ == '__main__':
    import matplotlib.pyplot as plt
    parser = argparse.ArgumentParser()
    parser.add_argument('--debug', action='store_true')
    parser.add_argument('--obstacle', action='store_true')
    script_args, _ = parser.parse_known_args()

    env = PandaImageSpacePushingEnv(debug=script_args.debug, include_obstacle=script_args.obstacle)
    env.reset()

    for i in tqdm(range(2)):
        action_i = env.action_space.sample()
        # action_i = np.array([0., 0., 1.])
        state, reward, done, info = env.step(action_i)
        plt.imshow(state)
        plt.savefig('/Users/mik/Desktop/test.png')
        if done:
            env.reset()
