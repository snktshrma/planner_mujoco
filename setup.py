from setuptools import find_packages, setup

package_name = 'robot_control'

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
    maintainer='Sanket Sharma',
    maintainer_email='sharma.sanket272@gmail.com',
    description='Package for publishing state and vision data from the Panda robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'publisher = robot_control.publisher:main',
        ],
    },
)
