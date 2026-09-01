from setuptools import setup

package_name = 'robot_gateway'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='dev@example.com',
    description='遥控网关：WebSocket -> /cmd_vel，心跳失控保护',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gateway = robot_gateway.gateway_node:main',
        ],
    },
)
