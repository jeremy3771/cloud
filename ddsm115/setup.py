from setuptools import find_packages, setup

package_name = 'ddsm115'

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
    maintainer='cae',
    maintainer_email='cae@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receive_direction = ddsm115.receive_direction:main',
             'keyboard_input = ddsm115.keyboard_input:main',
             'stop_wheel_motor = ddsm115.stop_wheel_motor:main',
             'run_motor = ddsm115.run_motor:main',
        ],
    },
)
