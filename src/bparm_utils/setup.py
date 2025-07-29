from setuptools import setup

package_name = 'bparm_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Utility nodes for the bparm robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'camera_pose_listener = bparm_utils.camera_pose_listener:main',
        ],
    },
)
