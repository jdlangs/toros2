#!/usr/bin/env python3

from setuptools import setup, find_packages

setup(
    name='toros2',
    version='0.1',
    packages=find_packages(),
    install_requires=[
        'Click',
        'inflection',
    ],
    entry_points='''
        [console_scripts]
        toros2=toros2.main:main
    ''',
)
