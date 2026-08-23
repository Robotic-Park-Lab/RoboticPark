from setuptools import find_packages, setup

package_name = 'mars_supervisor_pkg'

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
    maintainer='Francisco Jose Manas',
    maintainer_email='fjmanas@dia.uned.es',
    description='Supervisor node that orchestrates RoboticPark multi-robot experiments from a YAML config: spawning and monitoring agents.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'supervisor_node = mars_supervisor_pkg.supervisor_node:main',
        ],
    },
)
