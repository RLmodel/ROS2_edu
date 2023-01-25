from setuptools import setup

package_name = 'my_turtlesim_pkg'

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
    maintainer='byb76',
    maintainer_email='tge1375@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_turtle_node    = my_turtlesim_pkg.my_turtle_node:main',
            'turtle_subscriber = my_turtlesim_pkg.my_turtlesim_subscriber:main'
        ],
    },
)
