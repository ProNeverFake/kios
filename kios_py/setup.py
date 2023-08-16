from setuptools import setup
from glob import glob
import os

package_name = 'kios_py'
module_name = 'kios_py/resource'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, module_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/resource',
         [package_name + '/resource/' + 'parameter.json'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='blackbird',
    maintainer_email='jicong_ao@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                # 'bt_udp_node = kios_py.bt_udp_node:main',
                # 'test_node_py = kios_py.test_node_py:main',
                # 'bt_medium_node = kios_py.bt_medium_node:main',
                'state_reader = kios_py.state_reader:main',
                'mongo_reader = kios_py.mongo_reader:main'
        ],
    },

)
