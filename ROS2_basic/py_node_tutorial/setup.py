from setuptools import setup

package_name = 'py_node_tutorial'

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
            'example_node_1 = py_node_tutorial.node_example_1:main',
            'example_node_2 = py_node_tutorial.node_example_2:main',
            'example_node_3 = py_node_tutorial.node_example_3:main',
            'example_node_4 = py_node_tutorial.node_example_4:main',
            'example_node_5 = py_node_tutorial.node_example_5:main',
            'test_node1     = py_node_tutorial.ex_node1:main',
            'test_node2     = py_node_tutorial.ex_node2:main',
            'test_node3     = py_node_tutorial.ex_node3:main',
            'test_node4     = py_node_tutorial.ex_node4:main',
            'test_node5     = py_node_tutorial.ex_node5:main',
            'test_node6     = py_node_tutorial.ex_node6:main',
        ],
    },
)
