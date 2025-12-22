from setuptools import setup

package_name = 'r2_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/r2_driver.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tee',
    maintainer_email='you@example.com',
    description='ROS2 driver node for Yahboom Rosmaster R2 (rear motors + steering)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'r2_driver_node = r2_driver.r2_driver_node:main',
        ],
    },
)
