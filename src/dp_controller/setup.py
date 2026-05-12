from setuptools import find_packages, setup

package_name = 'dp_controller'

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
    maintainer='Andreas A. Vannebo',
    maintainer_email='andrava@stud.ntnu.no',
    description='PID controller for station keeping of vessel. Takes 3DoF into consideration. Takes in positional data and sends general force/torque vector.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pid_controller = dp_controller.pid_controller_node:main'
        ],
    },
)
