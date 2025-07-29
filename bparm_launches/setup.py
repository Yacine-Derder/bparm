# File: bparm_launches/setup.py
import os
from glob import glob
from setuptools import setup

package_name = 'bparm_launches'

launch_files = glob(os.path.join('launch', '*.launch.py'))

data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'share/{package_name}', ['package.xml']),
    (f'share/{package_name}/launch', launch_files),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yacine',
    maintainer_email='yacine@example.com',
    description='Central launch package for BPArm-related systems',
    license='MIT',
)