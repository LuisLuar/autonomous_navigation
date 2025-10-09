from setuptools import find_packages, setup

package_name = 'robot_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/asus_xtion.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raynel',
    maintainer_email='luchitov2001@gmail.com',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
