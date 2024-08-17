from setuptools import setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'turtlebot3_control.turtlebot3_control_node.py',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Karan',
    maintainer_email='karansspk@gmail.com',
    description='ROS 2 package for turtle control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_control_node = turtlebot3_control.turtlebot3_control_node:main'
        ],
    },
)
