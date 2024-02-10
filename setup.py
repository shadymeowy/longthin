from setuptools import setup

setup(
    name='longthin',
    version='0.0.1',
    description='Long Thin Hauler',
    author='Tolga Demirdal',
    url='https://github.com/shadymeowy/longthin',
    setup_requires=[],
    install_requires=['drawing3d'],
    packages=['longthin', 'longthin.ltpacket'],
    entry_points={
        'console_scripts': [
            'longthin-sim = longthin.sim:main',
            'longthin-test = longthin.test:main',
            'longthin-bridge = longthin.ltpacket.bridge:main',
        ],
    },
)
