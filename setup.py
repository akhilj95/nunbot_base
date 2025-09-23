from setuptools import find_packages, setup

package_name = 'nunbot_base'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='labtech',
    maintainer_email='akhilmjohn@gmail.com',
    description='Package for omni3md ROS2 node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nunbot_base_node = nunbot_base.nunbot_base_node:main',
        ],
    },
)
