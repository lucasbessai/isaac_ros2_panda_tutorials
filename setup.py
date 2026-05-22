from setuptools import find_packages, setup

package_name = 'panda_controllers'

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
    maintainer='fpiadmin',
    maintainer_email='lucas.bessai03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'position_pd_controller = panda_controllers.panda_position_pd_controller:main',
            'panda_monitor = panda_controllers.panda_monitor:main',
        ],
    },
)
