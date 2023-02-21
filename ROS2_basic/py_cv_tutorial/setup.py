from setuptools import setup

package_name = 'py_cv_tutorial'

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
            'img_pub = py_cv_tutorial.img_pub:main',
            'img_sub = py_cv_tutorial.img_sub:main',
            'rosbag2_to_timedimg = py_cv_tutorial.rosbag2_to_timedimg:main',
        ],
    },
)
