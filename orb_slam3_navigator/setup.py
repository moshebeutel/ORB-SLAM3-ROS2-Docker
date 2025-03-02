from setuptools import find_packages, setup

package_name = 'orb_slam3_navigator'

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
    maintainer='Daphna',
    maintainer_email='daphnaas@matrix.co.il',
    description='A simple navigation unit, using tf2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'orb_slam3_navigator = orb_slam3_navigator.orb_slam3_movement_controller:main',
    ],
},
)
