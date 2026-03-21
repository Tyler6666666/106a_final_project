from setuptools import find_packages
from setuptools import setup

setup(
    name='tello_msgs',
    version='0.1.0',
    packages=find_packages(
        include=('tello_msgs', 'tello_msgs.*')),
)
