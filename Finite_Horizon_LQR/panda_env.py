import numpy as np
import pybullet as p
import os
import gym
import pybullet_data as pd
from base_env import BaseEnv


class PandaEnv(BaseEnv):

    def __init__(self, *args, **kwargs):
        self.panda = None
        self.goal_panda = None
        self.goal_state = None
        self.panda_joint_indxs = [0, 1, 2, 3, 4, 5, 6]
        self.finger_joint_indxs = [7, 8, 9, 10, 11]
        # self.reset_joints = [0., 0., 0., 0., 0., 0., 0.]
        self.reset_joints = np.array([0., 0., 0., -np.pi * 0.5, 0., np.pi * 0.5, 0.])
        self.rest_state = np.concatenate([self.reset_joints, np.zeros(7, )])
        self.joint_limits = [
            np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]),  # lower limit
            np.array([2.8973, 1.7628, 2.8973, 0.0698, 2.8973, 3.7525, 2.8973]),  # upper limit
        ]
        self.vel_limits = [
            -np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]),  # lower limit
            np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100]),  # upper limit
        ]
        self.joint_torque_limits = [
            -25 * np.ones(7),  # lower limit
            25 * np.ones(7),  # upper limit
        ]

        super().__init__(*args, **kwargs)

    def step(self, control):
        for i, control_i in enumerate(control):
            joint_i = self.panda_joint_indxs[i]
            p.setJointMotorControl2(self.panda, joint_i, p.TORQUE_CONTROL, force=control_i)
        p.stepSimulation()
        return self.get_state()

    def _reset_joints(self):
        for i, joint_indx in enumerate(self.panda_joint_indxs):
            joint_rest_value = self.reset_joints[i]
            p.resetJointState(self.panda, joint_indx, joint_rest_value, targetVelocity=0)

    def _view_goal(self):
        if self.goal_state is not None:
            goal_state = self.goal_state
            if self.goal_panda is None:
                self.goal_panda = p.loadURDF('franka_panda/panda_visual.urdf', useFixedBase=True)
            joint_poses = goal_state[:len(self.panda_joint_indxs)]
            joint_vels = np.zeros(len(joint_poses))
            for i, joint_indx in enumerate(self.panda_joint_indxs):
                joint_pos_i = joint_poses[i]
                joint_vel_i = joint_vels[i]
                p.resetJointState(self.goal_panda, joint_indx, targetValue=joint_pos_i, targetVelocity=joint_vel_i)
            for i, joint_indx in enumerate(self.finger_joint_indxs):
                p.setJointMotorControl2(self.panda, joint_indx, p.POSITION_CONTROL, targetPosition=0., )

    def reset(self, state=None):
        if state is not None:
            self.state = state
        else:
            self.state = self.rest_state
        p.resetSimulation()
        p.setAdditionalSearchPath(os.path.dirname(os.path.abspath(__file__)))
        self.panda = p.loadURDF('franka_panda/panda.urdf', useFixedBase=True)
        self._reset_joints()
        p.setGravity(0, 0, 0) # Gravity compensation -- only inertia matters for dynamics
        p.setTimeStep(self.dt)
        p.setRealTimeSimulation(0)
        for i, joint_indx in enumerate(self.panda_joint_indxs):
            p.setJointMotorControl2(self.panda,
                                    joint_indx,
                                    p.TORQUE_CONTROL,
                                    force=0)
            # p.changeDynamics(self.panda, joint_indx, linearDamping=0, angularDamping=0,
            #                  lateralFriction=0, spinningFriction=0, rollingFriction=0, maxJointVelocity=self.vel_limits[1][i])
        self.set_state(self.state)
        self._setup_camera()

    def get_state(self):
        xs = []
        x_dots = []
        for joint_indx in self.panda_joint_indxs:
            x, x_dot = p.getJointState(self.panda, joint_indx)[0:2]
            xs.append(x)
            x_dots.append(x_dot)
        return np.concatenate([xs, x_dots])

    def set_state(self, state):
        # expected state is composed by [joint_poses, joint_vels]
        joint_poses = state[:len(self.panda_joint_indxs)]
        joint_vels = state[len(self.panda_joint_indxs):2*len(self.panda_joint_indxs)]
        for i, joint_indx in enumerate(self.panda_joint_indxs):
            joint_pos_i = joint_poses[i]
            joint_vel_i = joint_vels[i]
            p.resetJointState(self.panda, joint_indx, targetValue=joint_pos_i, targetVelocity=joint_vel_i)
        self._close_finger()
        self._view_goal()

    def _get_state_space(self):
        state_space = gym.spaces.Box(low=np.concatenate([self.joint_limits[0], self.vel_limits[0]]),
                                     high=np.concatenate([self.joint_limits[1], self.vel_limits[1]]),
                                     )
        return state_space

    def _get_action_space(self):
        action_space = gym.spaces.Box(low=self.joint_torque_limits[0], high=self.joint_torque_limits[1])
        return action_space

    def _close_finger(self):
        # Set fingerss to default closed location so they do not float around while the simulation is in progress
        for i, joint_indx in enumerate(self.finger_joint_indxs):
            p.setJointMotorControl2(self.panda, joint_indx, p.POSITION_CONTROL, targetPosition=0.,)

    def _setup_camera(self):
        # self.render_h = 240
        self.render_h = 500
        # self.render_w = 320
        self.render_w = 750
        base_pos = [0.2, -0.3, 0.6]
        cam_dist = 1.5
        cam_pitch = -40
        cam_yaw = 0
        self.view_matrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=base_pos,
            distance=cam_dist,
            yaw=cam_yaw,
            pitch=cam_pitch,
            roll=0,
            upAxisIndex=2)
        self.proj_matrix = p.computeProjectionMatrixFOV(fov=60,
                                                        aspect=self.render_w / self.render_h,
                                                        nearVal=0.1,
                                                        farVal=100.0)