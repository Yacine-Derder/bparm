from setuptools import setup

package_name = 'slider_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Yacine Derder',
    maintainer_email='yderder@caltech.com',
    description='Slider + Xbox controller GUI that publishes ROS 2 messages',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gui_node = slider_gui.gui_node:main',
        ],
    },
)
