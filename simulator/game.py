#!/usr/bin/env python3
"""
File:          Game.py
Author:        Alex Cui
Last Modified: Rishabh on 1/17
"""

import os
import time
import pybullet as p

from simulator.racecar_agent import RacecarAgent
from simulator.field import Field
from simulator.legos import Legos
from simulator.utilities import Utilities
from simulator.trainingbot_agent import TrainingBotAgent

class Game:
    """Maintains and coordinates the game loop"""

    def __init__(self):
        """Initializes game elements
        Sets up game and simulation elements. For example,
        the three minute timer of the game.
        """
        self.cwd = os.getcwd()
        self.cid = p.connect(p.SHARED_MEMORY)
        if (self.cid < 0):
            p.connect(p.GUI)

        p.resetSimulation()
        p.setGravity(0, 0, -9.8)

        #for video recording (works best on Mac and Linux, not well on Windows)
        #p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
        p.setRealTimeSimulation(1)

        self.agent = TrainingBotAgent()
        self.field = Field()
        self.legos = Legos()

        # list of locations of the bins and the hole; order is subject to change
        self.goals = [(-0.5200621118, 0.14906832298, 0.1), (-0.24677018633, 0.14906832298, 0.1), (0.02652173913, 0.14906832298, 0.1),
                        (0.29981366459, 0.14906832298, 0.1), (0.57310559005, 0.14906832298, 0.1), (-0.5200621118, -0.14906832298, 0.1),
                         (-0.24677018633, -0.14906832298, 0.1), (0.02652173913, -0.14906832298, 0.1), (0.29981366459, -0.14906832298, 0.1),
                          (0.57310559005, -0.14906832298, 0.1), (0.77186335403, 0, 0.1)]

    def load_statics(self):
        """Loading the static objects
        Including field, buttons, and more.
        """
        self.field.load_urdf(self.cwd)
        self.legos.load_lego_urdfs(self.cwd, [(0, .3, "#ffffff")])

    def load_agents(self):
        """Loading the agents
        Including the button robot and the mobile block stacking robot.
        """
        self.agent.load_urdf(self.cwd)

    def load_ui(self):
        """Loading the UI components
        Such as sliders or buttons.
        """
        self.maxForceSlider = p.addUserDebugParameter("maxForce", 0, 5, 1)

    def read_ui(self):
        """Reads the UI components' state
        And publishes them for all of game to process
        """
        maxForce = p.readUserDebugParameter(self.maxForceSlider)
        self.agent.set_max_force(maxForce)

    def process_keyboard_events(self):
        """Read keyboard events
        And publishes them for all of game to process
        """
        keys = p.getKeyboardEvents()

        if keys.get(65296): #right
            self.agent.increaseRTargetVel()
            self.agent.decreaseLTargetVel()
        elif keys.get(65295): #left
            self.agent.increaseLTargetVel()
            self.agent.decreaseRTargetVel()
        elif keys.get(65297): #up
            self.agent.increaseLTargetVel()
            self.agent.increaseRTargetVel()
        elif keys.get(65298): #down
            self.agent.decreaseLTargetVel()
            self.agent.decreaseRTargetVel()
        else:
            self.agent.normalizeLTargetVel()
            self.agent.normalizeRTargetVel()

    def monitor_buttons(self):
        # Store the return values for readability
        buttonStates = p.getJointStates(
                            self.field.model_id,
                            [b.joint_id for b in self.field.buttons])

        # Get every button and press it if needed
        for i,x in enumerate(buttonStates):
            if x[0] < -.0038:
                self.field.buttons.press_button(i)
            else:
                self.field.buttons.unpress_button(i)

        # We don't have logic changing the button color
        # Too costly in terms of time
        # Can easily be implemented because logic there

    # moves the robot forward by the specified distance; speed was determined experimentally
    def move_forward(self, distance, distance_per_second = 0.023):
        self.agent.increaseLTargetVel()
        self.agent.increaseRTargetVel()
        self.agent.update()
        time.sleep(distance/distance_per_second)
        self.agent.normalizeLTargetVel()
        self.agent.normalizeRTargetVel()

    # moves the robot backward by the specified distance; speed was determined experimentally
    def move_backward(self, distance, distance_per_second = 0.023):
        self.agent.decreaseLTargetVel()
        self.agent.decreaseRTargetVel()
        self.agent.update()
        time.sleep(distance/distance_per_second)
        self.agent.normalizeLTargetVel()
        self.agent.normalizeRTargetVel()

    # turns the robot left by the specified angle; speed was determined experimentally
    def turn_left(self, radians, radians_per_second = 0.189):
        self.agent.increaseLTargetVel()
        self.agent.decreaseRTargetVel()
        self.agent.update()
        time.sleep(radians/radians_per_second)
        self.agent.normalizeLTargetVel()
        self.agent.normalizeRTargetVel()

    # turns the robot right by the specified angle; speed was determined experimentally
    def turn_right(self, radians, radians_per_second = 0.189):
        self.agent.increaseRTargetVel()
        self.agent.decreaseLTargetVel()
        self.agent.update()
        time.sleep(radians/radians_per_second)
        self.agent.normalizeLTargetVel()
        self.agent.normalizeRTargetVel()

    # makes the robot go to the specified goal
    def go_to_goal(self, goal_index, pi = 3.14159265359):
        # move to the center line
        curr_pos, curr_angle_quat = p.getBasePositionAndOrientation(self.agent.robot)
        curr_angle_radians = p.getEulerFromQuaternion(curr_angle_quat)[2]
        if curr_angle_radians < 0:
            curr_angle_radians = curr_angle_radians + 2*pi
        if not (curr_pos[1] < 0.05 and curr_pos[1] > -0.05):
            if curr_angle_radians < pi/2:
                self.turn_right(curr_angle_radians)
            elif curr_angle_radians < pi:
                self.turn_left(pi - curr_angle_radians)
            elif curr_angle_radians < 3*pi/2:
                self.turn_right(curr_angle_radians - pi)
            else:
                self.turn_left(2*pi - curr_angle_radians)
            curr_pos, curr_angle_quat = p.getBasePositionAndOrientation(self.agent.robot)
            curr_angle_radians = p.getEulerFromQuaternion(curr_angle_quat)[2]
            if curr_angle_radians < 0:
                curr_angle_radians = curr_angle_radians + 2*pi
            self.move_backward(abs(curr_pos[1]))
            curr_pos, curr_angle_quat = p.getBasePositionAndOrientation(self.agent.robot)
            curr_angle_radians = p.getEulerFromQuaternion(curr_angle_quat)[2]
            if curr_angle_radians < 0:
                curr_angle_radians = curr_angle_radians + 2*pi

        # move to the x-coordinate of the goal
        if curr_angle_radians < pi/2:
            self.turn_left(pi/2 - curr_angle_radians)
        elif curr_angle_radians < 3*pi/2:
            self.turn_right(curr_angle_radians - pi/2)
        else:
            self.turn_left(5*pi/2 - curr_angle_radians)
        curr_pos, curr_angle_quat = p.getBasePositionAndOrientation(self.agent.robot)
        curr_angle_radians = p.getEulerFromQuaternion(curr_angle_quat)[2]
        if curr_angle_radians < 0:
            curr_angle_radians = curr_angle_radians + 2*pi
        if curr_pos[0] < self.goals[goal_index][0]:
            self.move_forward(self.goals[goal_index][0] - curr_pos[0])
        else:
            self.move_backward(curr_pos[0] - self.goals[goal_index][0])

        # move to the specified bin
        curr_pos, curr_angle_quat = p.getBasePositionAndOrientation(self.agent.robot)
        curr_angle_radians = p.getEulerFromQuaternion(curr_angle_quat)[2]
        if curr_angle_radians < 0:
            curr_angle_radians = curr_angle_radians + 2*pi
        if self.goals[goal_index][1] > 0:
            self.turn_left(pi - curr_angle_radians)
            curr_pos, curr_angle_quat = p.getBasePositionAndOrientation(self.agent.robot)
            curr_angle_radians = p.getEulerFromQuaternion(curr_angle_quat)[2]
            if curr_angle_radians < 0:
                curr_angle_radians = curr_angle_radians + 2*pi
            self.move_forward(self.goals[goal_index][1] - curr_pos[1])
        elif self.goals[goal_index][1] < 0:
            self.turn_right(curr_angle_radians)
            curr_pos, curr_angle_quat = p.getBasePositionAndOrientation(self.agent.robot)
            curr_angle_radians = p.getEulerFromQuaternion(curr_angle_quat)[2]
            if curr_angle_radians < 0:
                curr_angle_radians = curr_angle_radians + 2*pi
            self.move_forward(curr_pos[1] - self.goals[goal_index][1])

    def run(self):
        """Maintains the game loop
        Coordinates other functions to execute here and
        tracks the delta time between each game loop.
        """
        self.load_statics()
        self.load_agents()
        self.load_ui()

        while True:
            self.read_ui()
            # self.process_keyboard_events()
            # self.monitor_buttons()
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(2)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(3)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(6)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(5)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(8)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(0)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(9)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(4)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(7)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(1)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            self.go_to_goal(10)
            time.sleep(1)
            print(p.getBasePositionAndOrientation(self.agent.robot))
            while True:
                self.read_ui()
                self.process_keyboard_events()
                self.agent.update()
            # self.field.buttons.update(1/240)

            # Debugging: safe to remove
            # print(f"buttons state: in_sequence?={self.field.buttons.in_sequence} num_sequenced={self.field.buttons.num_sequenced} extra_sequenced={self.field.buttons.extra_not_sequenced}")

            # # Steps time by 1/240 seconds
            # p.stepSimulation()
            # # Sleep for slightly less time to make it seem realitme
            # # Fudge factor experimentally determined
            # time.sleep(1/240 - .0002)
