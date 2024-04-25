from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
import numpy as np
import json

from kios_utils.math_utils import *
from tabulate import tabulate


@dataclass
class MiosSkill:
    skill_name: str
    skill_type: str
    skill_parameters: Dict[str, Any]
    retry: bool = field(default=False)
    retry_count: int = field(default=0)
    failure_pause: bool = field(default=False)
    isTrivial: bool = field(default=False)

    def __str__(self) -> str:
        return self.skill_name


@dataclass
class MiosCall:
    method_name: str
    method_payload: Dict[str, Any]
    retry: bool = field(default=False)
    retry_count: int = field(default=0)
    failure_pause: bool = field(default=False)
    isTrivial: bool = field(default=False)

    def __str__(self) -> str:
        return self.method_name


@dataclass
class KiosCall:
    method: object
    args: List[Any]
    call_name: str = field(default="default name")
    isTrivial: bool = field(default=False)

    def __str__(self) -> str:
        return self.call_name


# @dataclass


@dataclass
class MiosObject:
    name: str
    O_T_OB: np.ndarray
    O_T_TCP: np.ndarray
    OB_T_gp: np.ndarray
    OB_T_TCP: np.ndarray
    OB_I: np.ndarray
    q: List[float]
    grasp_width: float
    grasp_force: float
    mass: float
    geometry: json = field(default=None)

    @staticmethod
    def from_json(json: Dict[str, Any]) -> "MiosObject":
        O_T_TCP = (
            np.reshape(np.array(json["O_T_TCP"]), (4, 4)).T
            if "O_T_TCP" in json
            else None
        )
        return MiosObject(
            name=json["name"],
            O_T_OB=np.reshape(np.array(json["O_T_OB"]), (4, 4)).T,
            O_T_TCP=O_T_TCP,
            OB_T_gp=np.reshape(np.array(json["OB_T_gp"]), (4, 4)).T,
            OB_T_TCP=np.reshape(np.array(json["OB_T_TCP"]), (4, 4)).T,
            OB_I=np.reshape(np.array(json["OB_I"]), (3, 3)).T,
            q=json["q"],
            grasp_width=json["grasp_width"],
            grasp_force=json["grasp_force"],
            mass=json["mass"],
            geometry=json["geometry"],
        )

    @staticmethod
    def to_json(mios_object: "MiosObject") -> Dict[str, Any]:
        return {
            "name": mios_object.name,
            "O_T_OB": mios_object.O_T_OB.T.flatten().tolist(),
            "O_T_TCP": mios_object.O_T_TCP.T.flatten().tolist(),
            "OB_T_gp": mios_object.OB_T_gp.T.flatten().tolist(),
            "OB_T_TCP": mios_object.OB_T_TCP.T.flatten().tolist(),
            "OB_I": mios_object.OB_I.T.flatten().tolist(),
            "q": mios_object.q,
            "grasp_width": mios_object.grasp_width,
            "grasp_force": mios_object.grasp_force,
            "mass": mios_object.mass,
            "geometry": mios_object.geometry,
        }

    def __str__(self) -> str:
        table = [
            ["name", str(self.name)],
            ["O_T_OB", str(self.O_T_OB)],
            ["O_T_TCP", str(self.O_T_TCP)],
            ["OB_T_gp", str(self.OB_T_gp)],
            ["OB_T_TCP", str(self.OB_T_TCP)],
            ["OB_I", str(self.OB_I)],
            ["q", str(self.q)],
            ["grasp_width", str(self.grasp_width)],
            ["grasp_force", str(self.grasp_force)],
            ["mass", str(self.mass)],
            ["geometry", str(self.geometry)],
        ]
        return tabulate(table, headers=["Attribute", "Value"], tablefmt="plain")

    @staticmethod
    def generate_dummy(object_name: str) -> "MiosObject":
        return MiosObject(
            name=object_name,
            O_T_OB=np.eye(4),
            O_T_TCP=np.eye(4),
            OB_T_gp=np.eye(4),
            OB_T_TCP=np.eye(4),
            OB_I=np.eye(3),
            q=[0, 0, 0, 0, 0, 0, 0],
            grasp_width=0.0,
            grasp_force=0.0,
            mass=0.0,
            geometry=None,
        )


@dataclass
class MiosInterfaceResponse:
    has_finished: (
        bool  # * whether the task is fully conducted or not (has exception or not)
    )
    error_message: Optional[str]
    task_result: "MiosTaskResult" = field(default=None)

    @staticmethod
    def from_json(json_response: Dict[str, Any]) -> "MiosInterfaceResponse":
        """
        BB: the json should be response["result"] !!!
        """
        # instantiate the task result first
        task_result = None
        if isinstance(json_response, str):
            print(f"mios response: {json_response}")
            return MiosInterfaceResponse(
                has_finished="???",
                error_message="mios response is not a dictionary!",
                task_result=None,
            )
        if json_response.get("task_result") is not None:
            task_result = MiosTaskResult.from_json(json_response["task_result"])

        # instantiate the response
        return MiosInterfaceResponse(
            has_finished=json_response.get("result"),
            error_message=json_response.get("error"),
            task_result=task_result,
        )

    def __str__(self) -> str:
        table = [
            ["has_finished", self.has_finished],
            ["error_message", self.error_message],
            ["task_result", self.task_result],
        ]
        return tabulate(table, headers=["Attribute", "Value"], tablefmt="plain")


@dataclass
class MiosTaskResult:
    has_exception: bool
    has_external_stop: bool
    has_succeeded: bool
    error_reasons: List[str] = field(
        default_factory=[]
    )  # ? what is this? usually user stop can invoke this
    custom_results: Dict[str, Any] = field(default=None)
    skill_results: Dict[str, Any] = field(default=None)

    @staticmethod
    def from_json(json_task_result: Dict[str, Any]) -> "MiosTaskResult":
        return MiosTaskResult(
            error_reasons=json_task_result["error"],
            has_exception=json_task_result["exception"],
            has_external_stop=json_task_result["external_stop"],
            has_succeeded=json_task_result["success"],
            custom_results=json_task_result["results"],
            # skill_results=json_task_result["skill_results"],
        )

    def __str__(self) -> str:
        table = [
            ["has_exception", self.has_exception],
            ["has_external_stop", self.has_external_stop],
            ["has_succeeded", self.has_succeeded],
            ["error_reasons", self.error_reasons],
            ["custom_results", self.custom_results],
            ["skill_results", self.skill_results],
        ]
        return tabulate(table, headers=["Attribute", "Value"], tablefmt="plain")


#####################################################################################


@dataclass
class KiosObject:
    name: str
    source: str
    O_T_TCP: np.ndarray
    O_T_EE: np.ndarray
    joint_pose: List[float] = field(default=None)
    reference_object: None = field(default=None)

    def __str__(self) -> str:
        table = [
            ["name", self.name],
            ["source", self.source],
            ["joint_pose", self.joint_pose],
            ["O_T_TCP", self.O_T_TCP.tolist()],
            ["O_T_EE", self.O_T_EE.tolist()],
            ["reference_object", self.reference_object],
        ]
        return tabulate(table, headers=["Attribute", "Value"], tablefmt="plain")

    @staticmethod
    def from_mios_object(mios_object: MiosObject) -> "KiosObject":
        return KiosObject(
            name=mios_object.name,
            source="mios",
            joint_pose=mios_object.q,
            O_T_TCP=mios_object.O_T_TCP,
            O_T_EE=mios_object.O_T_OB,
            reference_object=None,
        )

    @staticmethod
    def from_json(json: Dict[str, Any]) -> "KiosObject":
        return KiosObject(
            name=json["object_name"],
            source=json["source"],
            joint_pose=json["joint_pose"] if "joint_pose" in json else None,
            O_T_TCP=np.array(json["O_T_TCP"]) if "O_T_TCP" in json else None,
            O_T_EE=np.array(json["O_T_EE"]) if "O_T_EE" in json else None,
            reference_object=(
                json["reference_object"] if "reference_object" in json else None
            ),
        )

    @staticmethod
    def from_relation(name: str, relation: "ReferenceRelation") -> "KiosObject":
        if relation.relative_joint_pose is None:
            joint_pose = None
        else:
            joint_pose = (
                relation.reference_object.joint_pose + relation.relative_joint_pose
            )

        O_T_TCP = relation.reference_object.O_T_TCP.dot(relation.relative_HT)

        return KiosObject(
            name=name,
            source="relation",
            joint_pose=joint_pose,
            O_T_TCP=O_T_TCP,
            reference_object=relation.reference_object,
        )


# ! BBWORK suspend here. need to figure out if using MiosObject is a good idea.
@dataclass
class ReferenceRelation:
    name: str  # ! this is not unique!
    reference_object: KiosObject
    relative_HT: np.ndarray  # from reference object to this object
    relative_joint_pose: List[float] = field(
        default=None
    )  # from reference object to this object

    def __init__(
        self,
        reference_object: KiosObject,
        relative_joint_pose: List[float] = None,
        relative_cartesian_pose: np.ndarray = None,
        relative_HT: np.ndarray = None,
    ):
        if reference_object.isinstance(KiosObject):
            self.reference_object = reference_object
        else:
            raise Exception("reference_object is not a MiosObject!")

        self.relative_joint_pose = relative_joint_pose

        if relative_HT is not None:
            self.relative_HT = relative_HT
        elif relative_cartesian_pose is not None:
            self.relative_HT = HT_from_xyzrpy(relative_cartesian_pose)
        else:
            raise Exception(
                "relative_HT and relative_cartesian_pose are both None! At least one of them should be set!"
            )

    @staticmethod  # * not sure necessary
    def from_json(json: Dict[str, Any]) -> "ReferenceRelation":
        return ReferenceRelation(
            reference_object=json[
                "reference_object"
            ],  # ! BUG, should be an object instead of a string.
            relative_joint_pose=json["relative_joint_pose"],
            relative_HT=json["relative_HT"],
            relative_cartesian_pose=json["relative_cartesian_pose"],
        )


@dataclass
class Toolbox:
    name: str
    # * for future you should consider using these parameters to invoke the gripper-related skills
    EE_finger_width_max: float = field(default=0.08)
    EE_finger_width_min: float = field(default=0.0)
    tool_mass: float = field(default=0.0)  # * for set load.
    load_width: float = field(
        default=0.042
    )  # the width the hand to reach in order to load this tool
    unload_width: float = field(
        default=0.08
    )  # the width the hand to reach in order to unload this tool
    grasp_force: float = field(default=70.0)  # the force the hand to grasp this tool
    grasp_speed: float = field(default=0.1)  # the speed the hand to grasp this tool
    grasp_eps_in: float = field(default=0.005)  # not used yet
    grasp_eps_out: float = field(default=0.005)
    # kinematics parameters
    EE_T_TCP: np.ndarray = field(default=np.eye(4))

    def __str__(self):
        table = [
            ["name", self.name],
            ["EE_finger_width_max", self.EE_finger_width_max],
            ["EE_finger_width_min", self.EE_finger_width_min],
            ["tool_mass", self.tool_mass],
            ["load_width", self.load_width],
            ["unload_width", self.unload_width],
            ["grasp_force", self.grasp_force],
            ["grasp_speed", self.grasp_speed],
            ["grasp_eps_in", self.grasp_eps_in],
            ["grasp_eps_out", self.grasp_eps_out],
            ["EE_T_TCP", self.EE_T_TCP.tolist()],
        ]
        return tabulate(table, headers=["Attribute", "Value"], tablefmt="plain")

    @staticmethod
    def from_json(json: Dict[str, Any]) -> "Toolbox":
        return Toolbox(
            name=json["name"],
            EE_finger_width_max=json["EE_finger_width_max"],
            EE_finger_width_min=json["EE_finger_width_min"],
            tool_mass=json["tool_mass"],
            load_width=json["load_width"],
            unload_width=json["unload_width"],
            grasp_force=json["grasp_force"],
            grasp_speed=json["grasp_speed"],
            grasp_eps_in=json["grasp_eps_in"],
            grasp_eps_out=json["grasp_eps_out"],
            EE_T_TCP=np.reshape(np.array(json["EE_T_TCP"]), (4, 4)).T,
        )

    @staticmethod
    def to_json(toolbox: "Toolbox") -> Dict[str, Any]:
        return {
            "name": toolbox.name,
            "EE_finger_width_max": toolbox.EE_finger_width_max,
            "EE_finger_width_min": toolbox.EE_finger_width_min,
            "tool_mass": toolbox.tool_mass,
            "load_width": toolbox.load_width,
            "unload_width": toolbox.unload_width,
            "grasp_force": toolbox.grasp_force,
            "grasp_speed": toolbox.grasp_speed,
            "grasp_eps_in": toolbox.grasp_eps_in,
            "grasp_eps_out": toolbox.grasp_eps_out,
            "EE_T_TCP": toolbox.EE_T_TCP.T.flatten().tolist(),
        }


@dataclass
class TaskScene:
    tool_map: Dict[str, Toolbox] = field(default_factory=dict)
    object_map: Dict[str, KiosObject] = field(default_factory=dict)
    reference_map: Dict[str, ReferenceRelation] = field(default_factory=dict)

    def __str__(self) -> str:
        table = [
            ["tools", self.tool_map],
            ["objects", self.object_map],
            ["references", self.reference_map],
        ]
        return tabulate(table, headers=["Attribute", "Value"], tablefmt="plain")

    def get_object(self, object_name: str) -> Optional[KiosObject] | None:
        """
        get object from the scene dictionary, return None if not found
        """
        return self.object_map.get(object_name, None)

    def get_tool(self, tool_name: str = None) -> Toolbox:
        if tool_name is None:
            return self.tool_map.get("no_tool")
        tool = self.tool_map.get(tool_name)
        if tool is None:
            from pprint import pprint

            pprint("current available tools are:")
            pprint(self.tool_map.keys())
            raise Exception(f"Tool {tool_name} is not in the scene!")
        return tool


@dataclass
class RobotState:
    joint_pose: List[float]
    O_T_EE: np.ndarray
    EE_T_TCP: np.ndarray
    O_T_TCP: np.ndarray
    status: str
    current_task: str
    gripper_width: float
    TF_T_EE_d: np.ndarray = field(default=None)
    q_d: List[float] = field(default=None)

    def __str__(self) -> str:
        table = [
            ["joint_pose", self.joint_pose],
            [
                "O_T_EE",
                "\n".join(["\t".join(map(str, row)) for row in self.O_T_EE.tolist()]),
            ],
            [
                "EE_T_TCP",
                "\n".join(["\t".join(map(str, row)) for row in self.EE_T_TCP.tolist()]),
            ],
            [
                "O_T_TCP",
                "\n".join(["\t".join(map(str, row)) for row in self.O_T_TCP.tolist()]),
            ],
            ["status", self.status],
            ["current_task", self.current_task],
            ["gripper_width", self.gripper_width],
            # ? what is this?
            # ["q_d", self.q_d],
            # ["TF_T_EE_d", "\n".join(["\t".join(map(str, row)) for row in self.TF_T_EE_d.tolist()])],
        ]
        return tabulate(table, headers=["Attribute", "Value"], tablefmt="plain")

    @staticmethod
    def from_json(json: Dict[str, Any]) -> "RobotState":
        return RobotState(
            joint_pose=json["q"],
            O_T_EE=np.reshape(np.array(json["O_T_EE"]), (4, 4)).T,
            EE_T_TCP=np.reshape(np.array(json["EE_T_TCP"]), (4, 4)).T,
            O_T_TCP=np.reshape(np.array(json["O_T_TCP"]), (4, 4)).T,
            status=json["status"],
            current_task=json["current_task"],
            gripper_width=json["gripper_width"],
            # q_d=json["q_d"],
            # TF_T_EE_d=np.reshape(np.array(json["TF_T_EE_d"]), (4, 4)).T,
        )

    @staticmethod
    def to_json(robot_state: "RobotState") -> Dict[str, Any]:
        return {
            "q": robot_state.joint_pose,
            "O_T_EE": robot_state.O_T_EE.T.flatten().tolist(),
            "EE_T_TCP": robot_state.EE_T_TCP.T.flatten().tolist(),
            "O_T_TCP": robot_state.O_T_TCP.T.flatten().tolist(),
            "status": robot_state.status,
            "current_task": robot_state.current_task,
            "gripper_width": robot_state.gripper_width,
            # "q_d": robot_state.q_d,
            # "TF_T_EE_d": robot_state.TF_T_EE_d.T.flatten().tolist(),
        }


@dataclass
class ParsedAction:
    action_name: str
    args: List[str]

    @staticmethod
    def from_string(action_string: str) -> "ParsedAction":
        # ! haven't tested yet
        action_name, *args = action_string.split()
        return ParsedAction(action_name, args)
