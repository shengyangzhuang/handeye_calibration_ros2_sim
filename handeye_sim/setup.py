from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'handeye_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='szhuang',
    maintainer_email='heyeason@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = handeye_sim.robot_state_estimation:main',
            'handeye = handeye_sim.handeye_estimation:main',
            'aruco = handeye_sim.aruco_estimation:main',
            'eye2hand = handeye_sim.publish_eye2hand:main',
            'publish_marker = handeye_sim.publish_marker:main',
            'plot_joint = handeye_sim.joint_plotter:main',
            'joint_positions = handeye_sim.joint_positions:main',
            'transform_listener = handeye_sim.transform_listener:main'
        ],
    },
)
