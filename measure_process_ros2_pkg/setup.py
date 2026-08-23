from setuptools import setup

package_name = 'measure_process_ros2_pkg'

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
    maintainer='Francisco Jose Manas',
    maintainer_email='fjmanas@dia.uned.es',
    description='ROS 2 node that publishes CPU and memory usage statistics for monitored system processes.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'measure_process = measure_process_ros2_pkg.measure_process:main'
        ],
    },
)
