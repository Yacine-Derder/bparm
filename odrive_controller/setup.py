from setuptools import setup

package_name = 'odrive_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/odrive_controller']),
        ('share/odrive_controller', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yacine Derder',
    maintainer_email='yderder@caltech.edu',
    description='ROS 2 node that sends slider values to ODrives via CAN',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odrive_controller_node = odrive_controller.odrive_controller_node:main'
        ],
    },
)
