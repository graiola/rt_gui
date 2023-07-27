from setuptools import setup

package_name = 'rt_gui_ros2'
submodules = "rt_gui_ros2/bindings"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gennaro Raiola',
    maintainer_email='gennaro.raiola@gmail.com',
    description='rt_gui_ros2 python bindings',
    license='Licensed under the Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = mypackage.main:main',
        ],
    },
)
