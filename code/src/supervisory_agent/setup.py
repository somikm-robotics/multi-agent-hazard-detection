from setuptools import setup

package_name = 'supervisory_agent' 

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
    description='Supervisory agent node package for multi-agent mining project',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_overrider_node = supervisory_agent.nodes.mission_overrider_node:main',
        ],
    },
)
