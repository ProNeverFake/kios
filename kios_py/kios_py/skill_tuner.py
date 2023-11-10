"""
The skill tuner node of kios_py:
    1. a server for skillTune service, which is used by coach node to ground the skills in a plan.
    2. optimize the parameters of the skills iteratively.
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

import json

from kios_interface.srv import TuneSkillRequest


class SkillTuner(Node):
    def __init__(self):
        super().__init__("skill_tuner")

        main_callback_group = MutuallyExclusiveCallbackGroup()

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
        self.get_logger().info("skill tuning test")

        if result:
            response.is_success = True
            response.error_message = "test msg"
            return response
        else:
            response.error_message = "skill tuning failed, please check!"
            return response

    def tune_skill(self, plan) -> bool:
        # TODO
        return True


def main(args=None):
    rclpy.init(args=args)

    skill_tuner = SkillTuner()

    rclpy.spin(skill_tuner)

    skill_tuner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
