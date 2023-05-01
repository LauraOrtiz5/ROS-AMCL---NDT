from setuptools import setup

package_name = 'save_pointcloud'

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
    maintainer='laura',
    maintainer_email='laura@todo.todo',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'savepc = save_pointcloud.save_pointcloud:main',
        	'subspc = save_pointcloud.subscribe_to_scan:main',
        	'subsmap = save_pointcloud.subs_to_map:main',
        	'tf = save_pointcloud.tflisten:main',
        	'scan_table = save_pointcloud.save_pointcloud_table:main',
        	'combi = save_pointcloud.combi:main',
        	'columns = save_pointcloud.combi_columns:main',
        	'tfcol = save_pointcloud.tf_columns:main',
        	'transform = save_pointcloud.transform:main',
        	'general = save_pointcloud.General:main',
        	'initialpose = save_pointcloud.InitialPose:main',
        ],
    },
)
