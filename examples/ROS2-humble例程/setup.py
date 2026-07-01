from setuptools import setup

package_name = 'dm_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.modules'],
    data_files=[
        ('share/ament_index/resource_index/packages',
        [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', ['launch/dm_imu.launch.py']),
        (f'share/{package_name}/launch', ['launch/dm_imu_rviz.launch.py']),
        (f'share/{package_name}/config', ['config/params.yaml']),
        (f'share/{package_name}/rviz', ['rviz/imu.rviz']),
    ],
    install_requires=[
        'setuptools',
        # 如果不用 apt 的 python3-serial，而想用 pip，请取消注释：
        # 'pyserial',
    ],
    zip_safe=True,
    maintainer='Kielas',
    maintainer_email='c1470759@outlook.com',
    description='DM IMU driver (ROS 2 Humble, Python + pyserial)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 节点入口；下一步我会在 dm_imu/node.py 里实现 main()
            'dm_imu_node = dm_imu.node:main',
        ],
    },
)
