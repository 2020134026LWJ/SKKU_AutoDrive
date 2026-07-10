from setuptools import find_packages, setup

package_name = 'ota_update_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sungbhin',
    maintainer_email='osb8252@gmail.com',
    description='OTA Update Manager for ROS2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ota_manager = ota_update_manager.ota_manager_node:main',
        ],
    },
)
