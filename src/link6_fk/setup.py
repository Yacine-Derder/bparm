from setuptools import find_packages, setup

package_name = 'link6_fk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/5.1.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bparm',
    maintainer_email='bparm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_node = link6_fk.fk_node:main',
        ],
    },
)
