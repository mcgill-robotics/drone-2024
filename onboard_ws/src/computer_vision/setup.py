from setuptools import find_packages, setup

package_name = 'computer_vision'
submodule = 'computer_vision/utils'

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
    description='Package with computer vision related node',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['dummy_cv = computer_vision.dummy_cv:main'],
    },
)
