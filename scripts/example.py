from pathlib import Path
from pydantic_mujoco.model import Mujoco as MujocoModel

if __name__ == "__main__":
    base_model = MujocoModel.load(Path("models/gait2354.xml"))

    base_model.to_dot()

    base_model.simulate()
