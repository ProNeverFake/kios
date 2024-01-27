from typing import Dict, List, Any
import json
import numpy as np

from kios_robot.data_types import MiosObject, TaskScene, ReferenceRelation, Toolbox


class SceneFactory:
    task_scene: TaskScene = None

    def __init__(self):
        pass

    def create_scene_from_json(self, scene_json: json) -> TaskScene:
        # ! good luck
        if False:
            # create the reference objects first
            self.task_scene = TaskScene()

            for reference_object_json in scene_json["reference_objects"]:
                if reference_object_json["source"] == "pre-defined":
                    self.task_scene.object_map[
                        reference_object_json["object_name"]
                    ] = MiosObject(
                        name=reference_object_json["object_name"],
                        joint_pose=reference_object_json["joint_pose"],
                        O_T_EE=reference_object_json["O_T_EE"],
                        reference_object=None,
                    )
                elif reference_object_json["source"] == "mios":
                    raise NotImplementedError
                elif reference_object_json["source"] == "vision":
                    raise NotImplementedError
                else:
                    raise Exception("Undefined source!")

            # create the relative objects
            for object_name, reference_relation in scene_json[
                "relative_objects"
            ].items():
                # construct the reference relation
                raise NotImplementedError
                # ! BBWORK suspend here. need to figure out if using MiosObject is a good idea.

        # the default tool list
        tool_map = {}
        tool = Toolbox(
            name="parallel_box1",
            EE_HT_TCP=np.array(
                [
                    [1, 0, 0, 0.0],
                    [0, 1, 0, 0.0],
                    [0, 0, 1, 0.1],
                    [0, 0, 0, 1],
                ]
            ),
        )
        tool_map["parallel_box1"] = tool

        # create the scene
        self.task_scene = TaskScene(
            tool_map=tool_map,
        )

        return self.task_scene
