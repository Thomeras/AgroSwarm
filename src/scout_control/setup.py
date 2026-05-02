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
            'launch/mapping_mission.launch.py',
            'launch/precision_landing_test.launch.py',
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/agricultural_field.world',
            'worlds/obstacle_course.world',
            'worlds/swarm_field.world',
        ]),
        ('share/' + package_name + '/config', [
            'config/obstacle_avoidance.rviz',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tj',
    maintainer_email='st78906@upce.cz',
    description='ROS2 control stack for the Scout autonomous agricultural drone swarm prototype',
    license='UNLICENSED',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # core
            'swarm_agent               = scout_control.core.swarm_agent:main',
            'swarm_coordinator         = scout_control.core.swarm_coordinator:main',
            'field_setup_coordinator   = scout_control.core.field_setup_coordinator:main',
            'home_manager              = scout_control.core.home_manager:main',
            'mission_launcher          = scout_control.core.mission_launcher:main',
            'gcs_bridge                = scout_control.core.gcs_bridge:main',
            'spray_controller          = scout_control.core.spray_controller:main',
            'cell_data_recorder        = scout_control.core.cell_data_recorder:main',
            'ml_interface              = scout_control.core.ml_interface:main',
            'obstacle_avoidance_runtime = scout_control.core.obstacle_avoidance_runtime:main',
            'mapping_mission           = scout_control.missions.mapping_mission:main',
            'field_model_builder       = scout_control.mapping.field_model_builder:main',
            'precision_landing         = scout_control.vision.precision_landing:main',
            # utils
            'grid_generator            = scout_control.utils.grid_generator:main',
            'task_allocator            = scout_control.utils.task_allocator:main',
            # viz
            'camera_hud                = scout_control.viz.camera_hud:main',
            'obstacle_viz              = scout_control.viz.obstacle_viz:main',
            'gimbal_cam_viz            = scout_control.viz.gimbal_cam_viz:main',
            'scan_cloud_viz            = scout_control.viz.scan_cloud_viz:main',
            # manual
            'field_setup_tool          = scout_control.manual.field_setup_tool:main',
            'manual_controller         = scout_control.manual.manual_controller:main',
            'legacy_manual_controller  = scout_control.manual.legacy_manual_controller:main',
        ],
    },
)
