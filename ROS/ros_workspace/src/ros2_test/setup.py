from setuptools import setup

package_name = 'ros2_test'

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
    maintainer='Markus Knapp',
    maintainer_email='ros@direcs.de',
    description='RVR ROS2 test package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'batteryPublisher = ros2_test.batteryPublisher:main',
        ],
    },
)
