from setuptools import find_packages, setup

package_name = 'scout_base_py'

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
    maintainer='Jaime Tellie',
    maintainer_email='jaime.tellie@gmail.com',
    description='Scout base bridge (Python): ROS 2 to UNO R4 over USB serial',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'base_bridge = scout_base_py.base_bridge:main'
        ],
    },
)
