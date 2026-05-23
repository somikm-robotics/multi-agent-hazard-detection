from setuptools import setup

package_name = 'aerial_agent'

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
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='Aerial agent node package for multi-agent mining project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hazard_detection_node = aerial_agent.nodes.hazard_detection_node:main',
            'path_planner_node = aerial_agent.nodes.path_planner_node:main',
            'notifier_node = aerial_agent.nodes.notifier_node:main',
            'tf_publisher_node = aerial_agent.nodes.tf_publisher_node:main',
            'crazyflie_patrol_node = aerial_agent.nodes.crazyflie_patrol_node:main',
            'toxicity_measurement_node = aerial_agent.nodes.toxicity_measurement_node:main',
        ],
    },
)
