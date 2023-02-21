import os
from glob import glob
from setuptools import setup

package_name = 'py_service_tutorial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kimsooyoung',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_spawn_client = py_service_tutorial.turtle_spawn:main',
            'turtle_circle_server = py_service_tutorial.turtle_circle_server:main',
            'take_picture_server = py_service_tutorial.take_picture_server:main',
            'turtle_circle_server_advanced = py_service_tutorial.turtle_circle_server_advanced:main',
            'turtle_jail = py_service_tutorial.turtle_jail:main',
            'spawn_model = py_service_tutorial.spawn_model:main',
        ],
    },
)
