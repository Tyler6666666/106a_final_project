from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'my_drone_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models/aruco_marker/materials/textures'), glob('models/aruco_marker/materials/textures/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tyler',
    maintainer_email='tyler@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_controller = my_drone_vision.aruco_controller:main',
            'aruco_detector_3d = my_drone_vision.aruco_detector_3d:main',
            'aruco_controller_3d = my_drone_vision.aruco_controller_3d:main',
            'tello_direct_io = my_drone_vision.tello_direct_io:main',
            'face_detector = my_drone_vision.face_detector:main',
            'face_controller = my_drone_vision.face_controller:main',
        ],
    },
)
