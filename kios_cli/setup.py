from setuptools import find_packages, setup

package_name = 'kios_cli'

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
    maintainer='blackbird',
    maintainer_email='jicong_ao@163.com',
    description='kios command line interface',
    license='BBLAB',
    tests_require=['pytest'],
    entry_points={
        'ros2cli.command': [
            'kios = kios_cli.kios:KiosCommand',
        ],
    },
)
