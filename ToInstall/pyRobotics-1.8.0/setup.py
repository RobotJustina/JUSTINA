# -*- coding: utf-8 -*-
from distutils.core import setup
from pyrobotics.BB import __version__
setup(name='pyRobotics',
    version=__version__,
    author='Adrián Revuelta Cuauhtli',
    author_email='adrianrc.89@gmail.com',
    url='http://bioroboticsunam.github.io/pyRobotics',
    license='LICENSE.txt',
    data_files=[('', ['README', 'LICENSE.txt'])],
    description="A Python API to create modules that connect to our message-passing and shared varaibels hub 'BlackBoard'.",
    packages=['pyrobotics'])
