#!/usr/bin/env python3

from setuptools import setup

setup(
    name='toros2',
    version='0.0',
    py_modules=['toros2'],
    install_requires=[
        'Click',
    ],
    entry_points='''
        [console_scripts]
        toros2=toros2:main
    ''',
)
