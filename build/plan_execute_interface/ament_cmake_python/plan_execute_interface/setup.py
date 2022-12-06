from setuptools import find_packages
from setuptools import setup

setup(
    name='plan_execute_interface',
    version='0.0.0',
    packages=find_packages(
        include=('plan_execute_interface', 'plan_execute_interface.*')),
)
