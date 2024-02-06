import torch
import numpy as np
import abc
import pybullet as p
import pybullet_data as pd
import matplotlib.pyplot as plt


class BaseEnv(object):

    def __init__(self, dt=0.05):
        self.sim = p.connect(p.DIRECT)
        self.dt = dt
        self.state = None
        self.render_h = None
        self.render_w = None
        self.view_matrix = None
        self.proj_matrix = None
        self.action_space = self._get_action_space()
        self.state_space = self._get_state_space()
        self.reset()

    @abc.abstractmethod
    def step(self, action):
        pass

    @abc.abstractmethod
    def reset(self, state=None):
        pass

    @abc.abstractmethod
    def get_state(self):
        pass

    @abc.abstractmethod
    def set_state(self, state):
        pass

    @abc.abstractmethod
    def _get_action_space(self):
        pass

    @abc.abstractmethod
    def _get_state_space(self):
        pass

    @abc.abstractmethod
    def _setup_camera(self):
        pass

    def dynamics(self, state, control):
        # first we save the state of the simulation so we can return to it
        saved_sim_state = self.get_state()

        # Use the simulator to query the dynamics
        self.set_state(state)
        next_state = self.step(control)

        # Reset the simulator to saved state
        self.set_state(saved_sim_state)
        return next_state

    def batched_dynamics(self, state, control):
        state_shape = state.shape
        if len(state_shape) > 1:
            state = state.reshape((-1, state.shape[-1]))
            action = control.reshape((-1, control.shape[-1]))
            next_state = []
            for i, state_i in enumerate(state):
                action_i = action[i]
                next_state_i = self.dynamics(state_i, action_i)
                next_state.append(next_state_i)
            next_state = np.stack(next_state, axis=0).reshape(state_shape)
        else:
            next_state = self.dynamics(state, control)
        return next_state

    def rollout(self, initial_state, control_sequence):
        self.reset(state=initial_state)
        states = []
        for control in control_sequence:
            state = self.step(control)
            states.append(state)
        return np.stack(states, axis=0)

    def render(self):
        (_, _, px, _, _) = p.getCameraImage(
            width=self.render_w,
            height=self.render_h,
            renderer=p.ER_BULLET_HARDWARE_OPENGL,
            viewMatrix=self.view_matrix,
            projectionMatrix=self.proj_matrix)

        # rgb_array = np.array(px, dtype=np.uint8)
        rgb_array = np.reshape(np.array(px), (self.render_h, self.render_w, -1))
        rgb_array = rgb_array[:, :, :3]
        return rgb_array
