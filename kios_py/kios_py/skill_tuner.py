"""
The skill tuner node of kios_py:
    1. a server for skillTune service, which is used by coach node to ground the skills in a plan.
    2. optimize the parameters of the skills iteratively.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import qos_profile_sensor_data

import json

from .resource.mongodb_client import MongoDBClient

from kios_interface.srv import TuneSkillRequest

class SkillTuner(Node):
    def __init__(self):
        super().__init__("skill_tuner")

        main_callback_group = ReentrantCallbackGroup()

        self.tune_skill_service_server_ = self.create_service(
            TuneSkillRequest,
            "tune_skill_service",
            self.tune_skill_server_callback,
            callback_group=main_callback_group,
        )

    def fetch_grounded_skills(self):
        pass
    
    def update_skill_grounding(self):
        pass

    def tune_skill_server_callback(self, request, response):
        plan = request.plan
        response.is_success = False

        result = self.tune_skill(plan)

        if result:
            self.hasNewPlan = True
            response.is_success = True
            response.error_message = ""
            return response
        else:
            response.error_message = "skill tuning failed, please check!"
            return response


    def tune_skill(plan) -> bool:
        # TODO
        pass


def main(args=None):
    rclpy.init(args=args)

    SkillTuner = SkillTuner()

    executor = MultiThreadedExecutor()
    executor.add_node(SkillTuner)

    executor.spin()

    SkillTuner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
