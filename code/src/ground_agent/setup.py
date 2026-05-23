from setuptools import setup

package_name = 'ground_agent' 

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': '.'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Somi Murthy',
    maintainer_email='ucabskr@ucl.ac.uk.com',
    description='Ground agent node package for multi-agent mining project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_handler_node = ground_agent.nodes.mission_handler_node:main',
            'navigation_node = ground_agent.nodes.navigation_node:main',
            'dust_plume_hazard_orbit_twist_commander_node = ground_agent.nodes.dust_plume_hazard_orbit_twist_commander_node:main',    
            'path_planner_node = ground_agent.nodes.path_planner_node:main',
            'on_arrival_task_node = ground_agent.nodes.on_arrival_task_node:main',
            'initial_inspection_node = ground_agent.nodes.initial_inspection_node:main',
            'dust_sensor_relay_node = ground_agent.nodes.dust_sensor_relay_node:main',
            'fibrous_hazard_orbit_twist_commander_node = ground_agent.nodes.fibrous_hazard_orbit_twist_commander_node:main',
            'dust_plume_density_estimation_node = ground_agent.nodes.dust_plume_density_estimation_node:main',

            'tf_publisher_node = ground_agent.nodes.tf_publisher_node:main',
            'tf_listener_node = ground_agent.nodes.tf_listener_node:main',
            'twist_relay_node = ground_agent.nodes.twist_relay_node:main',
            'gps_covariance_injector_node = ground_agent.nodes.gps_covariance_injector_node:main',
            'imu_covariance_injector_node = ground_agent.nodes.imu_covariance_injector_node:main',
        ],
    },
)
