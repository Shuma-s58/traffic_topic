from setuptools import find_packages, setup

package_name = 'traffic_topic'

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
    maintainer='Shuma-s58',
    maintainer_email='shuma.suzuki.2021@outlook.jp',
    description='a package for tukuba2024',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'shell_publisher = traffic_topic.shell_publisher:main',
            'traffic_observer = traffic_topic.traffic_observer:main',
            'traffic_judgementer = traffic_topic.traffic_judgementer:main',
        ],
    },
)
