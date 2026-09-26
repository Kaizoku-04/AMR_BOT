from glob import glob
from setuptools import find_packages, setup

package_name = 'open_amr_arm_cell'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*')),
        ('share/' + package_name + '/srdf', glob('srdf/*')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohannad Rababah',
    maintainer_email='dodorapapah@gmail.com',
    description='UR10e palletizing cell: geometry, description, MoveIt/Pilz config, reach study, cell controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'reach_study = open_amr_arm_cell.reach_study:main',
        'tracking_test = open_amr_arm_cell.tracking_test:main',
    ]},
)
