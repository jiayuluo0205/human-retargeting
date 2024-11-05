import os
import sys
from setuptools import find_packages, setup

if sys.version_info.major < 3 or (
    sys.version_info.major == 3 and sys.version_info.minor < 8
):
    print(
        "This library requires Python 3.8 or higher, but you are running "
        "Python {}.{}. The installation will likely fail.".format(
            sys.version_info.major, sys.version_info.minor
        )
    )


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    name="xarm6_interface",
    version="0.0.1",
    packages=find_packages(),
    description="",
    long_description=read("readme.md"),
    long_description_content_type="text/markdown",
    author="Zhixuan Xu",
    author_email="ariszxxu@gmail.com",
)