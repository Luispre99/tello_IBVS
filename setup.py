from setuptools import setup

package_name = 'tello_ibvs'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tello',
    maintainer_email='ple4ga@bosch.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'takeoff_and_land = tello_ibvs.aruco_land:main',
            'visual_servoing = tello_ibvs.tello_visual_servoing:main',
            'teleop_keys = tello_ibvs.tello_teleop_keys:main'
        ],
    },
)
