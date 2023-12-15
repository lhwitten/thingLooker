from setuptools import find_packages, setup

package_name = 'thingLooker'

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
    maintainer='lwitten',
    maintainer_email='lhwitten@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'explore=thingLooker.explore:main'
        'get_nerf_data=thingLooker.get_nerf_data:main'
        ],
    },
)
