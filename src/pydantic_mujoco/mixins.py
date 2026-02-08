import copy
import os
import shutil
from pathlib import Path
import subprocess

import graphviz
import numpy as np
from lxml import etree as ElementTree
from transformations import quaternion_matrix, euler_matrix, euler_from_matrix


def _tostring(arr: np.ndarray) -> str:
    return " ".join(str(e) for e in arr.flatten())


class PositionMixin:
    @property
    def position(self) -> np.ndarray:
        if self.pos_ is not None:
            return np.fromstring(self.pos_, sep=" ")
        else:
            return np.zeros(3)

    @position.setter
    def position(self, position: np.ndarray):
        if np.any(position):
            self.pos_ = _tostring(position)
        else:
            self.pos_ = None


class PoseMixin(PositionMixin):
    @property
    def rotation(self) -> np.ndarray:
        if self.quat_ is not None:
            quat = np.fromstring(self.quat_, sep=" ")
            return quaternion_matrix(quat)[:3, :3]

        elif self.euler_ is not None:
            euler = np.fromstring(self.euler_, sep=" ")
            return euler_matrix(*euler, "rxyz")[:3, :3]

        elif self.xyaxes_ is not None:
            xyaxes = np.fromstring(self.xyaxes_, sep=" ")
            Parent_R_Elem = np.eye(3)
            Parent_R_Elem[:3, :2] = xyaxes.reshape(3, 2)
            Parent_R_Elem[:3, 2] = np.cross(Parent_R_Elem[:3, 0], Parent_R_Elem[:3, 1])
            return Parent_R_Elem

        else:
            return np.eye(3)

    @rotation.setter
    def rotation(self, Parent_R_Elem: np.ndarray):
        self.quat_ = None
        self.euler_ = None
        self.xyaxes_ = None

        if not np.allclose(Parent_R_Elem, np.eye(3)):
            Parent_T_Elem = np.eye(4)
            Parent_T_Elem[:3, :3] = Parent_R_Elem
            self.euler_ = _tostring(np.array(euler_from_matrix(Parent_T_Elem, "rxyz")))

    @property
    def pose(self) -> np.ndarray:
        Parent_T_Elem = np.eye(4)
        Parent_T_Elem[:3, :3] = self.rotation
        Parent_T_Elem[:3, 3] = self.position
        return Parent_T_Elem

    @pose.setter
    def pose(self, Parent_T_Elem: np.ndarray):
        self.position = Parent_T_Elem[:3, 3]
        self.rotation = Parent_T_Elem[:3, :3]


class JointMixin(PositionMixin):
    @property
    def axis(self) -> np.ndarray:
        return np.fromstring(self.axis_, sep=" ")

    @axis.setter
    def axis(self, axis: np.ndarray):
        self.axis_ = _tostring(axis)


class BodyMixin:
    def bodies(self):
        queue = copy.copy(self.body_)

        while queue:
            body = queue.pop()
            queue += body.body_
            yield body


class MujocoMixin:
    @classmethod
    def load(cls, path: Path):
        etree = ElementTree.parse(path)

        model = cls.from_xml_tree(etree.getroot())

        model._filename = path

        for tendon in etree.findall("tendon"):
            for spatial in tendon.findall("spatial"):
                model._tendon_order[spatial.attrib["name"]] = [
                    f"{key}/{val}"
                    for element in spatial
                    for key, val in element.attrib.items()
                    if key in ["geom", "site"]
                ]

        for body in etree.findall(".//body"):
            if len(body.findall("joint")):
                model._joint_order[body.attrib["name"]] = [
                    joint.attrib["name"] for joint in body.findall("joint")
                ]

        # We'll expect that geometry files live in a directory with the name {self._filename}_geometry
        os.makedirs(path.parent / (path.stem + "_geometry"), exist_ok=True)

        for asset in model.asset_:
            for mesh in asset.mesh_:
                geom_file = Path(path.stem + "_geometry") / Path(mesh.file_).name

                if Path(mesh.file_) != geom_file:
                    shutil.copy(path.parent / mesh.file_, path.parent / geom_file)
                    mesh.file_ = geom_file

        return model

    def save(self, path: Path):
        os.makedirs(path.parent / (path.stem + "_geometry"), exist_ok=True)

        for asset in self.asset_:
            for mesh in asset.mesh_:
                geom_dir = Path(path.stem + "_geometry") / Path(mesh.file_).name

                try:
                    shutil.copy(self._filename.parent / mesh.file_, path.parent / geom_dir)
                    mesh.file_ = geom_dir
                except shutil.SameFileError:
                    pass

        self._filename = path

        etree = self.to_xml_tree(skip_empty=True)

        for tendon in etree.findall("tendon"):
            for spatial in tendon.findall("spatial"):
                original_order = self._tendon_order[spatial.attrib["name"]]
                current_order = [
                    f"{key}/{val}"
                    for element in spatial
                    for key, val in element.attrib.items()
                    if key in ["geom", "site"]
                ]

                spatial[:] = [spatial[original_order.index(element)] for element in current_order]

        for body in etree.findall(".//body"):
            if body.attrib["name"] in self._joint_order:
                joints = [e for e in body if e.tag == "joint"]
                original_order = self._joint_order[body.attrib["name"]]
                current_order = [joint.attrib["name"] for joint in joints]
                body[:] = [joints[current_order.index(e)] for e in original_order] + [
                    e for e in body if e.tag != "joint"
                ]

        ElementTree.indent(etree.getroottree(), space="\t", level=0)
        etree.getroottree().write(path)

    def to_dot(self, filename: Path = Path("/tmp/graph")):
        dot = graphviz.Digraph()

        queue = [("world_body", self.worldbody_)]

        while queue:
            name, body = queue.pop()
            dot.node(name)

            for child in body.body_:
                dot.edge(name, child.name_)

            queue += [(body.name_, body) for body in body.body_]

        sites = {site.name_: body.name_ for body in self.worldbody_.bodies() for site in body.site_}

        for tendon in self.tendon_:
            for spatial in tendon.spatial_:
                for i in range(len(spatial.site_) - 1):
                    dot.edge(
                        sites[spatial.site_[i].site_],
                        sites[spatial.site_[i + 1].site_],
                        color="red",
                        style="dotted",
                    )

        dot.view(filename)

    def make_copy(self, path: Path):
        tmp = copy.deepcopy(self)
        tmp.save(path)
        return tmp

    def get_body(self, name: str):
        try:
            return next(body for body in self.worldbody_.bodies() if body.name_ == name)
        except StopIteration:
            return None

    def simulate(self, out_path=Path("/tmp/mujoco_model.xml")):
        self.save(out_path)
        subprocess.run(f"python -m mujoco.viewer --mjcf={out_path}".split(" "))
