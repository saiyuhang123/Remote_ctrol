from setuptools import setup
import os
from glob import glob

package_name = 'detector_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob('models/*.onnx') + glob('models/*.pt')),
        ('share/' + package_name + '/models/yolov8n_openvino_model',
         glob('models/yolov8n_openvino_model/*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='dev@example.com',
    description='图像识别节点：行人/人脸/车牌（纯CPU）',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'detector = detector_node.detector_node:main',
        ],
    },
)
