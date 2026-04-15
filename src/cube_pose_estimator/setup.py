from setuptools import find_packages, setup

package_name = 'cube_pose_estimator'

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
    maintainer='sirslab',
    maintainer_email='elascala@outlook.it',
    description='Cube pose estimation from point cloud',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'cube_pose_node = cube_pose_estimator.cube_pose_node:main',
        ],
    },
)
