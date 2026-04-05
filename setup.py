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
            'ackermann_odometry = navegacion_gps.ackermann_odometry:main',
            'cmd_vel_ackermann_bridge_v2 = navegacion_gps.cmd_vel_ackermann_bridge_v2:main',
            'gazebo_utils = navegacion_gps.gazebo_utils:main',
            'datum_setter = navegacion_gps.datum_setter:main',
            'zones_manager = navegacion_gps.zones_manager:main',
            'nav_command_server = navegacion_gps.nav_command_server:main',
            'nav_snapshot_server = navegacion_gps.nav_snapshot_server:main',
            'pixhawk_odometry = navegacion_gps.pixhawk_odometry:main',
            'polygon_stamped_republisher = navegacion_gps.polygon_stamped_republisher:main',
            'sim_drive_telemetry = navegacion_gps.sim_drive_telemetry:main',
            'sim_sensor_normalizer_v2 = navegacion_gps.sim_sensor_normalizer_v2:main',
            'dual_gps_heading_sim = navegacion_gps.dual_gps_heading_sim:main',
            'dual_gps_heading_real = navegacion_gps.dual_gps_heading_real:main',
            'imu_pose_republisher = navegacion_gps.imu_pose_republisher:main',
            'tracking_path_debug = navegacion_gps.tracking_path_debug:main',
        ],
    },
)
