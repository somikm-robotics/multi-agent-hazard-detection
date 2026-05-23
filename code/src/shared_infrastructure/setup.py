from setuptools import setup

package_name = 'shared_infrastructure'

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
    description='Shared utilities and service definitions for multi-agent system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shutdown_listener_node = shared_infrastructure.nodes.shutdown_listener_node:main',
            
        ],
    },
)
