from setuptools import setup

setup(
    name='longthin',
    version='0.0.1',
    description='Long Thin Hauler',
    author='Tolga Demirdal',
    url='https://github.com/shadymeowy/longthin',
    setup_requires=[],
    install_requires=['drawing3d'],
    packages=[
        'longthin',
        'longthin.ltpacket',
        'longthin.model',
        'longthin.geometry',
        'longthin.graphics',
    ],
    entry_points={
        'console_scripts': [
            'longthin-basicsim = longthin.basic_sim:main',
            'longthin-test = longthin.test:main',
            'longthin-bridge = longthin.utils.bridge:main',
            'longthin-blink = longthin.utils.blink:main',
            'longthin-echo = longthin.utils.echo:main',
            'longthin-plot = longthin.utils.plot:main',
            'longthin-params = longthin.utils.params:main',
            'longthin-params-ui = longthin.utils.params_ui:main',
            'longthin-pfd = longthin.utils.pfd:main',
            'longthin-sim = longthin.utils.sim:main',
        ],
    },
)
