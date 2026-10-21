from setuptools import find_packages, setup

package_name = 'fastlio_loop_closure'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jkbot',
    maintainer_email='jkbot@example.com',
    description='Live 6DoF keyframe loop closure backend for FAST-LIO',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'live_loop_closure = fastlio_loop_closure.live_loop_closure:main',
        ],
    },
)
