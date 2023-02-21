from setuptools import setup

package_name = 'py_topic_tutorial'

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
    maintainer='yunbumbaek',
    maintainer_email='ybbaek@rlmodel.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_pub_node           = py_topic_tutorial.topic_example_1_publisher:main',
            'topic_sub_node           = py_topic_tutorial.topic_example_2_subscriber:main',
            'parking_node             = py_topic_tutorial.topic_example_3_pub_and_sub:main',
            'qos_example_publisher    = py_topic_tutorial.qos_example_publisher:main',
            'qos_example_subscriber   = py_topic_tutorial.qos_example_subscriber:main',
            'qos_strtopic_pub         = py_topic_tutorial.qos_str_topic_pub:main',
            'qos_strtopic_sub         = py_topic_tutorial.qos_str_topic_sub:main',
            'twistcmd_pub             = py_topic_tutorial.tp_twistcmd_pub:main',   
            'laserscan_sub            = py_topic_tutorial.tp_laserscan_sub:main',       
        ],
    },
)
