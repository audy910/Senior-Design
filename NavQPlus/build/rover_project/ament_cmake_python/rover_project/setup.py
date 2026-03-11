from setuptools import find_packages
from setuptools import setup

setup(
    name='rover_project',
    version='0.0.0',
    packages=find_packages(
        include=('rover_project', 'rover_project.*')),
)
