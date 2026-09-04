from setuptools import setup

package_name = 'capture_server'

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
    description='图片抓拍服务：手动/自动抓拍当前视频帧，自动保存归档',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'capture_server = capture_server.capture_server_node:main',
        ],
    },
)
