# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

pydantic_mujoco provides a Pydantic `BaseModel` for programmatically manipulating MuJoCo MJCF (XML) files. It auto-generates a deeply nested Pydantic model from MuJoCo's XML schema, then monkey-patches additional methods for file I/O, pose manipulation, and visualization.

## Commands

```bash
# Install in development mode
pip install -e .

# Install dependencies
pip install -r requirements.txt

# Regenerate model.py from MuJoCo's schema (requires mujoco, bs4, jinja2)
python scripts/generate_base_model.py

# Run the example script
python scripts/example.py

# Format code
black --line-length 100 .
```

There is no test suite or linter configured.

## Architecture

### Code Generation Pipeline

`scripts/generate_base_model.py` → `templates/pydantic_xml.txt` → `src/pydantic_mujoco/model.py`

1. **`scripts/generate_base_model.py`** calls `mujoco.mj_printSchema()` to get MuJoCo's XML schema as HTML, parses it with BeautifulSoup into a tree of `Element` dataclasses, then renders the Jinja2 template.
2. **`templates/pydantic_xml.txt`** is the Jinja2 template that produces the Pydantic model classes. Note: the template references `config/templates` as its loader path and has a stale import (`from myobody.extensions import *`) at the bottom that doesn't appear in the generated output.
3. **`src/pydantic_mujoco/model.py`** is the ~1960-line **generated** file. Do not edit manually — regenerate it instead. It defines `Mujoco` and all nested element classes (Compiler, Option, Body, Actuator, etc.) using `pydantic_xml.BaseXmlModel`.

### Extension System (Monkey-Patching)

Extensions in `src/pydantic_mujoco/extensions/` add methods to the generated model classes at import time via monkey-patching:

- **`file_io.py`** — Patches `Mujoco.load()` (classmethod), `Mujoco.save()`, `Mujoco.make_copy()`, and `Mujoco.to_dot()`. Handles XML parsing via lxml, geometry file management (copies STL files into `{model_name}_geometry/` directories), and preserves joint/tendon ordering across save/load cycles.
- **`pose_properties.py`** — Patches `.pose`, `.position`, `.rotation` properties onto Body, Geom, Camera, Site, Inertial classes and `.axis` property onto Joint. Uses numpy and the `transformations` library. Supports quaternion, euler, and xyaxes rotation representations. Also patches `Mujoco.Body.bodies()` (BFS iterator over kinematic tree) and `Mujoco.get_body(name)`.

### Key Conventions

- All XML attribute fields use a trailing underscore (e.g., `name_`, `pos_`, `quat_`) to avoid Python keyword conflicts.
- All attribute values are stored as strings, matching the MJCF XML representation. Numeric conversion happens in extension code (e.g., `np.fromstring(elem.pos_, sep=" ")`).
- Child element cardinality types from the schema: `!` (required), `?` (optional), `*` (list), `R` (recursive/self-referencing like body).
- `Mujoco.Worldbody` inherits from `Mujoco.Body` with a different XML tag.
- Private attributes `_filename`, `_joint_order`, `_tendon_order` on `Mujoco` track file path and element ordering for round-trip fidelity.

### Dependencies

graphviz, lxml, pydantic_xml, transformations, mujoco, bs4, jinja2
