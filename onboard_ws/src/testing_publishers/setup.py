from setuptools import find_packages, setup

package_name = 'testing_publishers'

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
    maintainer='drone',
    maintainer_email='imad.issafras@mail.mcgill.ca',
    description='Package with nodes to test nodes and make sure',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drop_bottle_pub = testing_publishers.publisher_member_function:main',
        ],
    },
)
