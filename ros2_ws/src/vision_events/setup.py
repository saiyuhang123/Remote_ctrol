from glob import glob
from setuptools import setup

setup(name='vision_events', version='0.1.0', packages=['vision_events'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/vision_events']),
                  ('share/vision_events', ['package.xml']),
                  ('share/vision_events/launch', glob('launch/*.py')),
                  ('share/vision_events/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='jkbot', maintainer_email='dev@example.com', license='Apache-2.0',
      description='Independent local vision event recording',
      entry_points={'console_scripts': ['recorder = vision_events.node:main',
                                        'vision_cli = vision_events.cli:main']})
