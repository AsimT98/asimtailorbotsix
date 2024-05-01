from setuptools import find_packages
from setuptools import setup

setup(
    name='asitlorbotsix_firmware',
    version='0.0.0',
    packages=find_packages(
        include=('asitlorbotsix_firmware', 'asitlorbotsix_firmware.*')),
)
