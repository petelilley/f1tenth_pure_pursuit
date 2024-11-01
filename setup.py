from setuptools import find_packages, setup

package_name = 'f1tenth_pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Peter Lilley',
    maintainer_email='peterlil@buffalo.edu',
    description='Pure pursuit algorithm implementation for F1Tenth racecar',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = f1tenth_pure_pursuit.pure_pursuit:main',
            'test_waypoint_publisher = f1tenth_pure_pursuit.test_waypoint_publisher:main'
        ],
    },
)
