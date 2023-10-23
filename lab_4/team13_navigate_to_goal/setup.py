from setuptools import find_packages, setup

package_name = 'team13_navigate_to_goal'

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
    maintainer='aryan',
    maintainer_email='asarswat8@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'print_fixed_odometry = team13_navigate_to_goal.print_fixed_odometry:main',
            'getObjectRange = team13_navigate_to_goal.getObjectRange:main',
            'goToGoal = team13_navigate_to_goal.goToGoal:main',   
        ],
    },
)
