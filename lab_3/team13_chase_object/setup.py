from setuptools import find_packages, setup

package_name = 'team13_chase_object'

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
    maintainer='burger',
    maintainer_email='aryansarswat2000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_object = team13_chase_object.detect_object:main',
            'get_object_range = team13_chase_object.get_object_range:main',
            'chase_object = team13_chase_object.chase_object:main',            
        ],
    },
)
