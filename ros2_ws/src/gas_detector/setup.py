from setuptools import setup

setup(name='gas_detector', version='0.1.0', packages=['gas_detector'],
      py_modules=['gq_otk9633'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/gas_detector']),
                  ('share/gas_detector', ['package.xml', 'README.md', 'gq_otk9633_使用说明.md'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='jkbot', maintainer_email='dev@example.com', license='Proprietary',
      description='Independent read-only gas acquisition',
      entry_points={'console_scripts': ['gas_monitor = gas_detector.node:main']})
