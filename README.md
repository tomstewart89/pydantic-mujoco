# pydantic_mujoco

A pydantic `BaseModel` to make it easier to programatically manipulate mujoco .MJCF files.

# Example Usage:

```python
from pydantic_mujoco.model import Mujoco as MujocoModel

model = MujocoModel.load(Path("models/gait2354.xml"))

# Print the names of all the bodies in the model
print([body.name_ for body in model.worldbody_.bodies()])

# Get the pose of a body as a numpy array
model.worldbody_.body_[0].pose

# Show the kinematic tree in graphviz
model.to_dot()
```
And here's what the generated graph looks like:
![graph](https://github.com/user-attachments/assets/4254a9c1-ad0b-4644-be4d-2d089a766c91)


# How it Works:

We use Mujoco's [mj_printSchema](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-printschema) to generate its XML schema in HTML. That HTML is parsed and then used to render a jinja2 template descrbing a pydantic base model (this all happens in `scripts/generate_base_model.py`).

To make the resulting model easier to work with, we add a few additional methods for loading / saving the model as well as walking its kinematic tree etc.

