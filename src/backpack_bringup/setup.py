import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'backpack_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include the config and launch files.
        (os.path.join('share', package_name,  'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='52189+timgentry@users.noreply.github.com',
    description='Brings up (aka launches) the backpack robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
