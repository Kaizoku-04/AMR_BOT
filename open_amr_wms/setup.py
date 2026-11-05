import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'open_amr_wms'
web = [f for f in glob('open_amr_wms/web/**/*', recursive=True) if os.path.isfile(f)]
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={'open_amr_wms': [os.path.relpath(f, 'open_amr_wms') for f in web]},
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Laith',
    maintainer_email='dodorapapah@gmail.com',
    description='Warehouse management for the OpenAMR Swarm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'wms = open_amr_wms.node:main',
        
    ]},
)
