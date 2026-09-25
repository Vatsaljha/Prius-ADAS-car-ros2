from setuptools import setup

package_name = 'prius_adas_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=[
        'setuptools'
    ],
    zip_safe=True,
    description='Navigation stack for OSRF Prius ADAS simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'prius_odom_bridge = prius_adas_navigation.prius_odom_bridge:main',
            'laser_scan_merger = prius_adas_navigation.laser_scan_merger:main',
        ],
    },
)