from setuptools import find_packages, setup

package_name = 'my_robot_pkg'

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
    maintainer='anas',
    maintainer_email='mokdadanas13@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello_node = my_robot_pkg.hello_node:main',
            'publisher_node = my_robot_pkg.publisher_node:main',
            'subscriber_node = my_robot_pkg.subscriber_node:main',
            'vision_node = my_robot_pkg.vision_node:main',
            'fake_arm_node = my_robot_pkg.fake_arm_node:main',
            'control_node = my_robot_pkg.control_node:main',
        ],
    },
)
