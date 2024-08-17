import numpy as np
from typing import get_args, Optional
import copy
from transformations import quaternion_matrix, euler_matrix, euler_from_matrix
from pydantic_mujoco.model import Mujoco


ElementWithPose = (
    Mujoco.Body
    | Mujoco.Default.Geom
    | Mujoco.Body.Geom
    | Mujoco.Default.Camera
    | Mujoco.Body.Camera
    | Mujoco.Body.Site
    | Mujoco.Default.Site
    | Mujoco.Body.Inertial
)


def tostring(arr: np.ndarray):
    return " ".join(str(e) for e in arr.flatten())


def get_axis(self: Mujoco.Body.Joint):
    return np.fromstring(self.axis_, sep=" ")


def set_axis(self: Mujoco.Body.Joint, axis: np.ndarray):
    self.axis_ = tostring(axis)


def get_position(elem: ElementWithPose):
    if elem.pos_ is not None:
        return np.fromstring(elem.pos_, sep=" ")
    else:
        return np.zeros(3)


def set_position(elem: ElementWithPose, position: np.ndarray):
    if np.any(position):
        elem.pos_ = tostring(position)
    else:
        elem.pos_ = None


def get_rotation(elem: ElementWithPose):
    if elem.quat_ is not None:
        quat = np.fromstring(elem.quat_, sep=" ")
        return quaternion_matrix(quat)[:3, :3]

    elif elem.euler_ is not None:
        euler = np.fromstring(elem.euler_, sep=" ")
        return euler_matrix(*euler, "rxyz")[:3, :3]

    elif elem.xyaxes_ is not None:
        xyaxes = np.fromstring(elem.xyaxes_, sep=" ")
        Parent_R_Elem = np.eye(3)
        Parent_R_Elem[:3, :2] = xyaxes.reshape(3, 2)
        Parent_R_Elem[:3, 2] = np.cross(Parent_R_Elem[:3, 0], Parent_R_Elem[:3, 1])
        return Parent_R_Elem

    else:
        return np.eye(3)


def set_rotation(elem: ElementWithPose, Parent_R_Elem: np.ndarray):
    elem.quat_ = None
    elem.euler_ = None
    elem.xyaxes_ = None

    if not np.allclose(Parent_R_Elem, np.eye(3)):
        Parent_T_Elem = np.eye(4)
        Parent_T_Elem[:3, :3] = Parent_R_Elem
        elem.euler_ = tostring(np.array(euler_from_matrix(Parent_T_Elem, "rxyz")))


def get_pose(elem: ElementWithPose):
    Parent_T_Elem = np.eye(4)
    Parent_T_Elem[:3, :3] = get_rotation(elem)
    Parent_T_Elem[:3, 3] = get_position(elem)
    return Parent_T_Elem


def set_pose(elem: ElementWithPose, Parent_T_Elem: np.ndarray):
    set_position(elem, Parent_T_Elem[:3, 3])
    set_rotation(elem, Parent_T_Elem[:3, :3])


def bodies(self: Mujoco.Body):
    queue = copy.copy(self.body_)

    while queue:
        body = queue.pop()
        queue += body.body_
        yield body


def get_body(self: Mujoco, name: str) -> Optional[Mujoco.Body]:
    try:
        return next(body for body in self.worldbody_.bodies() if body.name_ == name)
    except StopIteration:
        return None


for cls in get_args(ElementWithPose):
    cls.pose = property(get_pose, set_pose)

for cls in get_args(ElementWithPose):
    cls.position = property(get_position, set_position)

for cls in get_args(ElementWithPose):
    cls.rotation = property(get_rotation, set_rotation)

Mujoco.Body.Joint.axis = property(get_axis, set_axis)
Mujoco.Body.Joint.position = property(get_position, set_position)
Mujoco.Body.bodies = bodies
Mujoco.get_body = get_body
