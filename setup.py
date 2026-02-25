from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'harurobo2026'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*_launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aratahorie',
    maintainer_email='aratahorie89@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest',],
    },
    entry_points={
        'console_scripts': [
            "subscribe_twist_node = harurobo2026.subscribe_twist_node:main",
            "publish_twist_node = harurobo2026.publish_twist_node:main",
            "publish_feedback_node = harurobo2026.publish_feedback_node:main",
            "dyna_handler_node = ah_ros2_dynamixel.dyna_handler_node:main",
            "control_yagura_node = harurobo2026.control_yagura_node:main",
            "control_ring_node = harurobo2026.control_ring_node:main",
        ],
    },
)
