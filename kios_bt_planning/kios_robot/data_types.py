from dataclasses import dataclass
from typing import List, Optional, Dist, Any
import numpy as np

from kios_utils.math_utils import *


@dataclass
class MiosResponse:
    result: dict
    error: Optional[str]


@dataclass
class MiosObject:
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
        self.name = name
        self.joint_pose = joint_pose
        self.O_T_EE = O_T_EE
        self.reference_object = reference_object

    @staticmethod
    def from_relation(name: str, relation: "ReferenceRelation") -> "MiosObject":
        if relation.relative_joint_pose is None:
            joint_pose = None
        else:
            joint_pose = (
                relation.reference_object.joint_pose + relation.relative_joint_pose
            )

        O_T_EE = relation.reference_object.O_T_EE.dot(relation.relative_HT)

        MiosObject(name, joint_pose, O_T_EE)


@dataclass
class ReferenceRelation:
    reference_object: MiosObject
    relative_joint_pose: List[
        float
    ]  # from reference object to this object, the incremental joint pose
    relative_HT: np.ndarray  # from reference object to this object

    def __init__(
        self,
        reference_object: MiosObject,
        relative_joint_pose: List[float] = None,
        relative_cartesian_pose: np.ndarray = None,
        relative_HT: np.ndarray = None,
    ):
        if reference_object.isinstance(MiosObject):
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
