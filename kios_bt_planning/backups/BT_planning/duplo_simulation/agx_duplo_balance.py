"""
Simulation environment for stacking duplo bricks in a balance act
"""
import sys

import agx
import agxOSG
import agxPython
import agxIO

import duplo_simulation.agx_duplo_bricks as duplo_bricks
import duplo_simulation.agx_common_functions as common_functions

import rclpy #pylint: disable=import-error

def build_scene(rosid=10, visual=True):
    """ Creates the scene to simulate. """
    brick_types = [duplo_bricks.BrickTypes.SPECIAL8X2PLUS2X2, \
                   duplo_bricks.BrickTypes.STANDARD2X2, \
                   duplo_bricks.BrickTypes.FAST4X2]

    x_pos = [-0.05, 0.0, 0.05, 0.1, 0, 0.05, -0.05, 0, 0.05]
    y_pos = [-0.1, -0.1, -0.1, -0.1, -0.05, -0.05, 0, 0, 0]
    z_pos = [0.000001] * 9
    start_positions = []

    #Goal positions, unbalanced
    #x_pos = [0.0, 0.0, 0.05, 0.05, 0, 0.05,-0.05, 0, 0.05 ]
    #y_pos = [0.0, 0.0, -0.1, -0.1, -0.05, -0.05, 0, 0, 0]
    #z_pos = [0.0192+0.002, 0.0, 0.000001, 0.0, -0.05, -0.05, 0, 0, 0]

    #Goal positions, with red
    #x_pos = [0.0, 0.0, 0.0, 0.05, 0, 0.05,-0.05, 0, 0.05 ]
    #y_pos = [0.0, -0.1, 0.0, -0.1, -0.05, -0.05, 0, 0, 0]
    #z_pos = [0.0192, 0.0, 0.000001, 0.0, -0.05, -0.05, 0, 0, 0]

    for i in range(3):
        start_positions.append(agx.Vec3(x_pos[i], y_pos[i], z_pos[i]))

    scene_settings = common_functions.SceneSettings()
    scene_settings.brick_types = brick_types
    scene_settings.start_positions = start_positions
    scene_settings.timestep = 1/100
    scene_settings.visual = visual
    scene_settings.listener_pool = common_functions.ListenerPool(size=24)

    return common_functions.main_loop(scene_settings, rosid=rosid, ros_step=0.1)

def main(args):
    """
    Script main. Called when run with the native Python interpreter
    instead of with agxViewer.
    """
    app = agxOSG.ExampleApplication()
    arg_parser = agxIO.ArgumentParser([sys.executable] + args)
    app.addScene(arg_parser.getArgumentName(1), "build_scene", ord('1'), False)
    if app.init(arg_parser):
        app.run()
        rclpy.shutdown() # Shut down ROS2
    else:
        print("An error occurred while initializing agx application.")

if agxPython.getContext() is None:
    # pylint: disable=invalid-name
    init = agx.AutoInit()
    if len(sys.argv) > 1:
        instance_id = sys.argv[1]
    if len(sys.argv) > 2 and int(sys.argv[2]) > 0:
        build_scene(instance_id, visual=False)
    else:
        main(sys.argv)
    