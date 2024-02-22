from typing import Dict, List, Any
import json
import numpy as np

from kios_robot.data_types import (
    MiosObject,
    TaskScene,
    ReferenceRelation,
    Toolbox,
    KiosObject,
)
from kios_scene.mongodb_interface import MongoDBInterface
import json
import os


class SceneFactory:
    task_scene: TaskScene = None

    def __init__(self):
        pass

    def create_scene_from_json(self, scene_json: json) -> TaskScene:
        # create the reference objects first
        mongodb_interface = MongoDBInterface()
        self.task_scene = TaskScene()

        for reference_object_json in scene_json["reference_objects"]:
            if reference_object_json["source"] == "pre-defined":
                self.task_scene.object_map[reference_object_json["object_name"]] = (
                    KiosObject.from_json(reference_object_json)
                )
            elif reference_object_json["source"] == "mios":
                mios_object = mongodb_interface.query_mios_object(
                    reference_object_json["object_name"]
                )
                kios_object = KiosObject.from_mios_object(mios_object)
                self.task_scene.object_map[kios_object.name] = kios_object
            elif reference_object_json["source"] == "tag_detection":
                raise NotImplementedError
            elif reference_object_json["source"] == "segmentation":
                raise NotImplementedError
            else:
                raise Exception("Undefined source!")

        # # create the relative objects
        # for object_name, reference_relation in scene_json["relative_objects"].items():
        #     # construct the reference relation
        #     raise NotImplementedError
        #     # ! BBWORK suspend here. need to figure out if using MiosObject is a good idea.

        # create the tools
        for tool_json in scene_json["tools"]:
            tool = Toolbox(
                name=tool_json["tool_name"],
                EE_T_TCP=np.array(tool_json["EE_T_TCP"]),
            )
            # print(tool)
            self.task_scene.tool_map[tool.name] = tool

        return self.task_scene

    def create_test_scene(self) -> TaskScene:
        file_dir = os.path.dirname(os.path.abspath(__file__))

        # Define the path to the JSON file
        json_file_path = os.path.join(file_dir, "test_scene.json")

        # Open the JSON file and load it as a JSON object
        with open(json_file_path, "r") as file:
            json_object = json.load(file)

        scene = self.create_scene_from_json(json_object)
        # print(scene)
        return scene


def test_sf():
    # Get the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))

    # Define the path to the JSON file
    json_file_path = os.path.join(current_dir, "test_scene.json")

    # Open the JSON file and load it as a JSON object
    with open(json_file_path, "r") as file:
        json_object = json.load(file)

    sf = SceneFactory()
    scene = sf.create_scene_from_json(json_object)
    print(scene)


if __name__ == "__main__":
    test_sf()
