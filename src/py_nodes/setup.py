from setuptools import setup

package_name = 'py_nodes'

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
    maintainer='bram',
    maintainer_email='bram@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',

    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'flow_find = py_nodes.flow_node:main',
        	'point = py_nodes.point_node:main',
        ],
    },
)
