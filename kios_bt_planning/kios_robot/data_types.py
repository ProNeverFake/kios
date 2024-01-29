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


@dataclass
class MiosCall:
    method_name: str
    method_payload: Dict[str, Any]


@dataclass
class MiosObject:
    name: str
    O_T_OB: np.ndarray
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
        return MiosObject(
            name=json["name"],
            O_T_OB=np.reshape(np.array(json["O_T_OB"]), (4, 4)).T,
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
            "OB_T_gp": mios_object.OB_T_gp.T.flatten().tolist(),
            "OB_T_TCP": mios_object.OB_T_TCP.T.flatten().tolist(),
            "OB_I": mios_object.OB_I.T.flatten().tolist(),
            "q": mios_object.q,
            "grasp_width": mios_object.grasp_width,
            "grasp_forch": mios_object.grasp_forch,
            "mass": mios_object.mass,
            "geometry": mios_object.geometry,
        }

    def __str__(self) -> str:
        table = [
            ["name", str(self.name)],
            ["O_T_OB", str(self.O_T_OB)],
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


@dataclass
class MiosInterfaceResponse:
    has_finished: bool  # * whether the task is fully conducted or not (has exception or not)
    error_message: Optional[str]
    task_result: "MiosTaskResult" = field(default=None)

    @staticmethod
    def from_json(json_response: Dict[str, Any]) -> "MiosInterfaceResponse":
        """
        BB: the json should be response["result"] !!!
        """
        # instantiate the task result first
        task_result = None
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
    joint_pose: List[float]
    O_T_EE: np.ndarray
    reference_object: None

    def __init__(
        self,
        name: str,
        joint_pose: List[float],
        O_T_EE: np.ndarray,
        reference_object: None = None,
    ):
        if name is not None:
            self.name = name
        else:
            raise Exception("name is not set!")

        if joint_pose is not None:
            self.joint_pose = joint_pose

        if O_T_EE is not None:
            self.O_T_EE = O_T_EE
        else:
            raise Exception("O_T_EE is not set!")

        if reference_object is not None:
            self.reference_object = reference_object

    @staticmethod
    def from_json(json: Dict[str, Any]) -> "KiosObject":
        return KiosObject(
            name=json["name"],
            joint_pose=json["joint_pose"],
            O_T_EE=json["O_T_EE"],
            reference_object=json["reference_object"],
        )

    @staticmethod
    def from_relation(name: str, relation: "ReferenceRelation") -> "KiosObject":
        if relation.relative_joint_pose is None:
            joint_pose = None
        else:
            joint_pose = (
                relation.reference_object.joint_pose + relation.relative_joint_pose
            )

        O_T_EE = relation.reference_object.O_T_EE.dot(relation.relative_HT)

        return KiosObject(
            name=name,
            joint_pose=joint_pose,
            O_T_EE=O_T_EE,
            reference_object=relation.reference_object,
        )


# ! BBWORK suspend here. need to figure out if using MiosObject is a good idea.
@dataclass
class ReferenceRelation:
    reference_object: KiosObject
    relative_joint_pose: List[
        float
    ]  # from reference object to this object, the incremental joint pose
    relative_HT: np.ndarray  # from reference object to this object

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
    EE_HT_TCP: np.ndarray = field(default=np.eye(4))


class TaskScene:
    Tool_map: Dict[str, Toolbox]
    object_map: Dict[str, KiosObject] = field(default_factory=dict)
    reference_map: Dict[str, ReferenceRelation] = field(default_factory=dict)
