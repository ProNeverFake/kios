"""Interface to allow a user to interact with the simulator and demonstrate a task."""

# Copyright (c) 2022, ABB
# All rights reserved.
#
# Redistribution and use in source and binary forms, with
# or without modification, are permitted provided that
# the following conditions are met:
#
#   * Redistributions of source code must retain the
#     above copyright notice, this list of conditions
#     and the following disclaimer.
#   * Redistributions in binary form must reproduce the
#     above copyright notice, this list of conditions
#     and the following disclaimer in the documentation
#     and/or other materials provided with the
#     distribution.
#   * Neither the name of ABB nor the names of its
#     contributors may be used to endorse or promote
#     products derived from this software without
#     specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import agx
import agxSDK
import agxRender

from agxPythonModules.utils.callbacks import StepEventCallback as sec
from agxPythonModules.utils.callbacks import KeyboardCallback as Input
from agxPythonModules.utils.environment import application, simulation
from simulation.algoryx.behaviors.agx_interface import AGXInterface
import simulation.algoryx.agx_application as AgxApplication


class UserControls():
    """This class allows the user to control the End Effector during a demonstration."""

    def __init__(self, behaviors: AGXInterface, app: AgxApplication = None) -> None:

        self.color = agxRender.Color.Black()
        self.behaviors = behaviors
        self.step = 0.01
        self.running = True
        self.with_cam = False
        self.app = app.app if app is not None else application()

        # self.add_text()

    def add_text(self) -> None:
        """Define the text to display."""
        self.app.getSceneDecorator().setText(
            1, 'Keyboard controls:', self.color)
        self.app.getSceneDecorator().setText(
            2, '- Left/Right Arrows control X dimension.', self.color)
        self.app.getSceneDecorator().setText(
            3, '- Up/Down Arrows control Y dimension.', self.color)
        self.app.getSceneDecorator().setText(
            4, '- PageUp/PageDown Keys control Z dimension.', self.color)
        self.app.getSceneDecorator().setText(
            5, f'- +/- change the movement step: current = {self.step}', self.color)
        self.app.getSceneDecorator().setText(
            6, f'- Back Space to confirm placing position and release object.', self.color)
        if self.with_cam:
            self.app.getSceneDecorator().setText(
                7, f'Camera Eye: {self.app.getCameraData().eye}', self.color)
            self.app.getSceneDecorator().setText(
                8, f'Camera Center: {self.app.getCameraData().center}', self.color)
            self.app.getSceneDecorator().setText(
                9, f'Camera Up: {self.app.getCameraData().up}', self.color)

    def update_step(self) -> None:
        """Update the step in the text section."""
        self.app.getSceneDecorator().setText(
            5, f'- +/- change the movement step: current = {self.step}', self.color)
        if self.with_cam:
            self.app.getSceneDecorator().setText(
                7, f'Camera Eye: {self.app.getCameraData().eye}', self.color)
            self.app.getSceneDecorator().setText(
                8, f'Camera Center: {self.app.getCameraData().center}', self.color)
            self.app.getSceneDecorator().setText(
                9, f'Camera Up: {self.app.getCameraData().up}', self.color)

    def clear_text(self) -> None:
        """Clear the text after the demonstration."""
        self.app.getSceneDecorator().clearText()

    def handle_key(self, data) -> None:
        """Callback function for keyboard events related to the gripper."""

        t0 = simulation().getTimeStamp() + 0.2
        if data.isKeyDown:
            if data.key == agxSDK.GuiEventListener.KEY_Left:
                sec.callAt(t0, lambda: self.behaviors._move_gripper(
                    self.behaviors.gripper_ctrl.getPosition() + agx.Vec3(-self.step, 0.0, 0.0)))
            if data.key == agxSDK.GuiEventListener.KEY_Right:
                sec.callAt(t0, lambda: self.behaviors._move_gripper(
                    self.behaviors.gripper_ctrl.getPosition() + agx.Vec3(self.step, 0.0, 0.0)))
            if data.key == agxSDK.GuiEventListener.KEY_Page_Up:
                sec.callAt(t0, lambda: self.behaviors._move_gripper(
                    self.behaviors.gripper_ctrl.getPosition() + agx.Vec3(0.0, -self.step, 0.0)))
            if data.key == agxSDK.GuiEventListener.KEY_Page_Down:
                sec.callAt(t0, lambda: self.behaviors._move_gripper(
                    self.behaviors.gripper_ctrl.getPosition() + agx.Vec3(0.0, self.step, 0.0)))
            if data.key == agxSDK.GuiEventListener.KEY_Up:
                sec.callAt(t0, lambda: self.behaviors._move_gripper(
                    self.behaviors.gripper_ctrl.getPosition() + agx.Vec3(0.0, 0.0, self.step)))
            if data.key == agxSDK.GuiEventListener.KEY_Down:
                sec.callAt(t0, lambda: self.behaviors._move_gripper(
                    self.behaviors.gripper_ctrl.getPosition() + agx.Vec3(0.0, 0.0, -self.step)))
            if data.key == agxSDK.GuiEventListener.KEY_KP_Add:
                self.step = round((self.step + 0.01), 3)
                self.update_step()
            if data.key == agxSDK.GuiEventListener.KEY_KP_Subtract:
                self.step = round((self.step - 0.01), 3)
                self.update_step()
            if data.key == agxSDK.GuiEventListener.KEY_BackSpace:
                self.clear_text()
                self.running = False

    def setup_keyboard_listener(self) -> None:
        """Sets up listener for keyboard commands to control gripper."""
        Input.bind(name='Left', key=agxSDK.GuiEventListener.KEY_Left, callback=self.handle_key)
        Input.bind(name='Right', key=agxSDK.GuiEventListener.KEY_Right, callback=self.handle_key)
        Input.bind(name='Back', key=agxSDK.GuiEventListener.KEY_Up, callback=self.handle_key)
        Input.bind(name='Forward', key=agxSDK.GuiEventListener.KEY_Down, callback=self.handle_key)
        Input.bind(name='Up', key=agxSDK.GuiEventListener.KEY_Page_Up, callback=self.handle_key)
        Input.bind(name='Down', key=agxSDK.GuiEventListener.KEY_Page_Down, callback=self.handle_key)
        Input.bind(name='+', key=agxSDK.GuiEventListener.KEY_KP_Add, callback=self.handle_key)
        Input.bind(name='-', key=agxSDK.GuiEventListener.KEY_KP_Subtract, callback=self.handle_key)
        Input.bind(
            name='Enter', key=agxSDK.GuiEventListener.KEY_BackSpace, callback=self.handle_key)
