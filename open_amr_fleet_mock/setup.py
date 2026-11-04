from glob import glob
from setuptools import find_packages, setup

package_name = 'open_amr_fleet_mock'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/scenarios', glob('scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mohannad Rababah',
    maintainer_email='dodorapapah@gmail.com',
    description='Message-level mock of the OpenAMR fleet and arm stations',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'mock_fleet = open_amr_fleet_mock.node:main',
    ]},
)
