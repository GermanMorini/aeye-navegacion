from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'navegacion_gps'


def regular_files(pattern: str):
    return [path for path in glob(pattern) if os.path.isfile(path)]


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), regular_files('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), regular_files('config/*')),
        (os.path.join('share', package_name, 'models'), regular_files('models/*')),
        (os.path.join('share', package_name, 'worlds'), regular_files('worlds/*')),
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps'),
         regular_files('models/turtlebot_waffle_gps/*')),
    ],
    install_requires=['setuptools', 'PyYAML', 'numpy'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pedro.gonzalez@eia.edu.co',
    description='Demo package for following GPS waypoints with nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_utils = navegacion_gps.gazebo_utils:main',
            'zones_manager = navegacion_gps.zones_manager:main',
            'nav_command_server = navegacion_gps.nav_command_server:main',
            'nav_snapshot_server = navegacion_gps.nav_snapshot_server:main',
            'nav_observability = navegacion_gps.nav_observability:main',
        ],
    },
)
