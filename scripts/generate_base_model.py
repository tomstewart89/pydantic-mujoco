import mujoco
from pathlib import Path
from dataclasses import dataclass, field
from typing import List, Dict
from bs4 import BeautifulSoup
from jinja2 import FileSystemLoader, Environment


@dataclass
class Element:
    tag: str
    attributes: List[Dict[str, str]]
    nesting: int
    type: int
    children: List["Element"] = field(default_factory=list)


soup = BeautifulSoup(mujoco.mj_printSchema(True, True), features="lxml")
environment = Environment(loader=FileSystemLoader("config/templates"), trim_blocks=True)
template = environment.get_template("pydantic_xml.txt")

elements = []

for element in soup("tr"):
    name_tag, type_tag, attributes_tag = element("td")
    indentation = int(name_tag.attrs["style"].split(":")[-1])
    tag = name_tag.text.replace("(world)", "")

    attr_text = attributes_tag.text.strip()
    attributes = [] if attr_text == "no attributes" else attr_text.split(" ")

    nesting = int((indentation - 5) / 15)
    siblings = elements

    for i in range(nesting):
        siblings = siblings[-1].children

    siblings.append(Element(tag, attributes, nesting, type_tag.text))

Path("src/pydantic_mujoco/model.py").write_text(template.render(elements=elements))
