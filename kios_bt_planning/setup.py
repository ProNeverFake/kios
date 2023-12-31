from setuptools import setup, find_packages

core_requirements = [
    "py-trees",
]

setup(
    name="kios_bt_planning",
    author="blackbird",
    description="Behavior Tree Planning for KIOS",
    version="0.1.0",
    packages=find_packages(),
    # include_package_data=True,
    # python_requires=">3.7,<3.9",
    install_requires=core_requirements,
)
