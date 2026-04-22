from setuptools import find_packages, setup

package_name = 'scout_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/camera_bridge.launch.py',
            'launch/camera_hud.launch.py',
            'launch/lidar_bridge.launch.py',
            'launch/swarm_mission.launch.py',
            'launch/full_e2e_mission.launch.py',
            'launch/isaac_e2e_mission.launch.py',
            'launch/obstacle_avoidance_test.launch.py',
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/agricultural_field.world',
            'worlds/obstacle_course.world',
        ]),
        ('share/' + package_name + '/config', [
            'config/obstacle_avoidance.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tj',
    maintainer_email='st78906@upce.cz',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'position_monitor = scout_control.position_monitor:main',
            'offboard_control = scout_control.offboard_control:main',
            'perimeter_flight = scout_control.perimeter_flight:main',
            'grid_generator    = scout_control.grid_generator:main',
            'field_commander   = scout_control.field_commander:main',
            'terrain_follower  = scout_control.terrain_follower:main',
            'home_manager      = scout_control.home_manager:main',
            'manual_commander  = scout_control.manual_commander:main',
            'camera_hud         = scout_control.camera_hud:main',
            'swarm_coordinator  = scout_control.swarm_coordinator:main',
            'swarm_agent        = scout_control.swarm_agent:main',
            'ml_interface            = scout_control.ml_interface:main',
            'spray_controller        = scout_control.spray_controller:main',
            'cell_data_recorder      = scout_control.cell_data_recorder:main',
            'manual_controller       = scout_control.manual_controller:main',
            'field_setup_coordinator = scout_control.field_setup_coordinator:main',
            'mission_launcher        = scout_control.mission_launcher:main',
            'obstacle_detector            = scout_control.obstacle_detector:main',
            'obstacle_avoidance_runtime   = scout_control.obstacle_avoidance_runtime:main',
            'obstacle_avoidance_mission   = scout_control.obstacle_avoidance_mission:main',
            'obstacle_viz                 = scout_control.obstacle_viz:main',
            'scan_cloud_viz               = scout_control.scan_cloud_viz:main',
            'gimbal_cam_viz               = scout_control.gimbal_cam_viz:main',
            'gcs_bridge = scout_control.gcs_bridge:main',
        ],
    },
)
