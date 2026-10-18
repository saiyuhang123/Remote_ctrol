from setuptools import setup

package_name = 'robot_gateway'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='dev@example.com',
    description='WebSocket gateway with control ownership and local patrol task proxy',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gateway = robot_gateway.gateway_node:main',
        ],
    },
)
