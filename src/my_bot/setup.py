from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # CHANGED: Now looks for ANY .py file in the launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # Install description files (URDF/Xacro)
        (os.path.join('share', package_name, 'description'), glob('description/*.xacro')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Articubot One migration for ROS 2 Jazzy',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)