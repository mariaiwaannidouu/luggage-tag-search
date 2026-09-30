"""Εγκατάσταση του πακέτου my_ur5e_controller (ament_python).

Κόμβοι (ros2 run my_ur5e_controller <όνομα>):
  move_arm3            - ο "εγκέφαλος": κινεί τον βραχίονα και ψάχνει το tag ΠΡΟΟΡΙΣΜΟΥ
  aruco_multi_detector - βρίσκει τα ArUco tags στην κάμερα -> /luggage/tag_detections
  luggage_mover        - (μόνο προσομοίωση) κινεί τις 3 βαλίτσες στο Gazebo σαν ταινία
  collision_publisher  - δημοσιεύει τις βαλίτσες ως εμπόδια στο MoveIt (/planning_scene)
"""
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_ur5e_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
        # URDF/xacro του βραχίονα με την κάμερα
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro'))),
        # Τα meshes ΠΡΕΠΕΙ να εγκαθίστανται: το xacro τα ζητάει με
        # $(find my_ur5e_controller)/meshes/... (χωρίς /home/... paths).
        (os.path.join('share', package_name, 'meshes'),
            glob(os.path.join('meshes', '*'))),
        # Ρυθμίσεις βραχίονα (ur5e.yaml, gofa12.yaml)
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ioathemar',
    maintainer_email='mariaiwaannidouu@gmail.com',
    description='Αναζήτηση tag προορισμού σε χερούλι βαλίτσας με βραχίονα + κάμερα',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'move_arm3 = my_ur5e_controller.move_arm3:main',
            'aruco_multi_detector = my_ur5e_controller.aruco_multi_detector:main',
            'luggage_mover = my_ur5e_controller.luggage_mover:main',
            'collision_publisher = my_ur5e_controller.collision_publisher:main',
        ],
    },
)
