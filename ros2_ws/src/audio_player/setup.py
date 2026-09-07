from setuptools import setup

package_name = 'audio_player'

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
    description='语音播报与智能提示：TTS 合成 + 机器人端播放',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'tts_node = audio_player.tts_node:main',
        ],
    },
)
