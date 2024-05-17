from setuptools import find_packages, setup

package_name = 'path_planner'
submodule = 'path_planner/vfh_submodules'
setup(
    name=package_name,
    version='0.0.0',
    packages=[*find_packages(exclude=['test']), submodule],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unhappysquid',
    maintainer_email='imad.issafras@mail.mcgill.ca',
    description='Ros package with local and global planning nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['local_planner = path_planner.local_planner:main'],
    },
)
