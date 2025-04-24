from setuptools import setup
import os

package_name = 'odom_upsampling'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/params.yaml'])
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='kareemmagdy420@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_upsampling = odom_upsampling.upsampling_node:main',
        ],
    },
)
