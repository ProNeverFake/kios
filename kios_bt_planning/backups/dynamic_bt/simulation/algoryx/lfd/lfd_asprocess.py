"""A LfD application with Algoryx through multiprocessing."""

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

import multiprocessing as mp
import os
from typing import Any, Dict, List

from simulation.algoryx.agx_application import CloudpickleWrapper
from simulation.algoryx.lfd import user_interaction
import yaml


class LfDProcess(mp.Process):

    def __init__(
        self,
        config_folder: str,
        interface: Any,
        args: Any = None
    ) -> None:

        with open(os.path.join(config_folder, 'sim_data.yaml')) as f:
            sim_data = yaml.safe_load(f)
            self.ref_objects = sim_data['demonstration']['reference_frames']
        with open(os.path.join(config_folder, 'sim_objects.yaml')) as f:
            self.obj_data = yaml.safe_load(f)

        self.event = mp.Event()

        self.interface = interface

        super().__init__(target=self.worker, args=args)

    def stop(self):
        self.event.set()
        self.join()

    def worker(
        self,
        conn: mp.connection.Connection,
        env_fn_wrapper: CloudpickleWrapper,
        args: List[str],
        targets: Dict = None
    ) -> None:
        """Run the genetic programming algorithm."""
        def user_control(app: Any, holding: str) -> None:
            self.user_input.running = True
            self.user_input.add_text()
            self.user_input.setup_keyboard_listener()
            while self.user_input.running:
                app.step(0.5)
            self.interface._release(holding)
            return None

        app = env_fn_wrapper.var(args)
        app.bringup(self.obj_data, targets, visual=True)
        app.reset(targets)
        self.interface.initialize()

        self.interface.remove_locks()
        self.interface.preempt_skill()
        self.interface.add_merger()

        self.user_input = user_interaction.UserControls(self.interface, app)

        running = True
        while running:
            if self.event.is_set():
                app.shutdown()
                break
            if conn.poll():
                data = conn.recv()
                if data[0] == 'get_sensor_data':
                    pos, rot = self.interface.get_item_in_frame(data[1][0], data[1][1])
                    conn.send((pos, rot))
                elif data[0] == 'pick':
                    self.interface.pick(data[1])
                    app.step(3.0)
                    conn.send(self.interface.holding)
                elif data[0] == 'lift':
                    self.interface.lift_gripper()
                    app.step(3.0)
                    conn.send(True)
                else:
                    user_control(app, self.interface.holding)
                    app.step(3.0)
                    self.user_input.clear_text()
                    conn.send(None)
