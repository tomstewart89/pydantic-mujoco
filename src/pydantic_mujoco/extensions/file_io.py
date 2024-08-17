import graphviz
import subprocess
import shutil
import os
import copy
from pathlib import Path
from lxml import etree as ElementTree
from pydantic_mujoco.model import Mujoco


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


def save(self: Mujoco, path: Path):
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


def to_dot(self: Mujoco, filename: Path = Path("/tmp/graph")):
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


def make_copy(self: Mujoco, path: Path):
    tmp = copy.deepcopy(self)
    tmp.save(path)
    return tmp


Mujoco.load = classmethod(load)
Mujoco.save = save
Mujoco.make_copy = make_copy
Mujoco.to_dot = to_dot
