from glob import glob
from setuptools import find_packages, setup

package_name = 'open_amr_swarm'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohannad Rababah',
    maintainer_email='dodorapapah@gmail.com',
    description='Decentralized swarm layer for the OpenAMR fleet',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'swarm_agent = open_amr_swarm.agent:main',
        'mission_generator = open_amr_swarm.mission_generator:main',
    ]},
)
