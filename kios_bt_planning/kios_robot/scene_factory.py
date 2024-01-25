from typing import Dict, List, Any
from kios_robot.data_types import MiosObject, TaskScene, ReferenceRelation
import json


class SceneFactory:
    task_scene: TaskScene = None

    def __init__(self):
        pass

    def create_scene_from_json(self, scene_json: json) -> TaskScene:
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
        for object_name, reference_relation in scene_json["relative_objects"].items():
            # construct the reference relation
            raise NotImplementedError
            # ! BBWORK suspend here. need to figure out if using MiosObject is a good idea.
