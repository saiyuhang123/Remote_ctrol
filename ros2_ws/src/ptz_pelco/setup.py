from setuptools import setup

package_name = 'ptz_pelco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ptz_pelco.yaml']),
        ('share/' + package_name + '/launch', ['launch/ptz_pelco.launch.py']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='jkbot',
    maintainer_email='jkbot@todo.todo',
    description='PELCO-D PTZ over RS485 (absolute pose + jog), replaces the SDK PTZ path',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'ptz_pelco_node = ptz_pelco.node:main',
        ],
    },
)
