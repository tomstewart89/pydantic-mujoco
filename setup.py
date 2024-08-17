from setuptools import setup, find_packages

setup(
    name="pydantic_mujoco",
    pakcage_dir={"": "src"},
    packages=find_packages(where="src"),
)
