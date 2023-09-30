from setuptools import setup

package_name = 'demo_nav'

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
    maintainer='rlmodel',
    maintainer_email='ybbaek@rlmodel.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_pose    = demo_nav.get_amclpose:main',
            'update_pose = demo_nav.updatepose:main',
        ],
    },
)
