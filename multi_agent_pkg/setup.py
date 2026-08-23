from setuptools import find_packages, setup

package_name = 'multi_agent_pkg'

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
    description='Multi-agent control package for RoboticPark: affine-formation (herding) control, Lagrange-multiplier-based formation control, and supporting agent nodes.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_node = multi_agent_pkg.basic_node:main',
            'lagrange_multipliers = multi_agent_pkg.lagrange_multipliers:main',
            'affine_formation_node = multi_agent_pkg.affine_formation_node:main'
        ],
    },
)
