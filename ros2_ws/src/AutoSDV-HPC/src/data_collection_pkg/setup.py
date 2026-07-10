from setuptools import setup
import os
from glob import glob

package_name = 'data_collection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include your specific .pyc file
        (os.path.join('lib', package_name), 
         [os.path.join(package_name, 'data_collection_func_lib.cpython-310.pyc')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Data collection package for ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_collection_node = data_collection_pkg.data_collection_node:main',
            'drive_node = data_collection_pkg.drive_node:main',
        ],
    },
)