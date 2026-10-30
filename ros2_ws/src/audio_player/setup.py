from setuptools import setup
from glob import glob

package_name = 'audio_player'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.json')),
        ('share/' + package_name + '/prompts', glob('prompts/*.wav')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='dev@example.com',
    description='固定提示音、运行状态提醒与机器人音频播放',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'tts_node = audio_player.tts_node:main',
        ],
    },
)
