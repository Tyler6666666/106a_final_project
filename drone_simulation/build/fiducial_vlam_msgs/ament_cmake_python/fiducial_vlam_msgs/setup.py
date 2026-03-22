from setuptools import find_packages
from setuptools import setup

setup(
    name='fiducial_vlam_msgs',
    version='0.1.0',
    packages=find_packages(
        include=('fiducial_vlam_msgs', 'fiducial_vlam_msgs.*')),
)
