from setuptools import setup

package_name = 'behavior_tree_learning'

setup(
    name=package_name,
    version='0.0.0',
    packages=['behavior_tree_learning', 'duplo_simulation', 'duplo_state_machine'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jstyrud',
    maintainer_email='45998292+jstyrud@users.noreply.github.com',
    description='Code for the paper Combining Planning and Learning of Behavior Trees for Robotic Assembly',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main=duplo_simulation.main:run_individual',
            'paper_run=duplo_simulation.main:paper_run',
            'paper_plots=duplo_simulation.main:paper_plots',
            'paper_figures=duplo_simulation.main:paper_figures'
        ],
    },
)
