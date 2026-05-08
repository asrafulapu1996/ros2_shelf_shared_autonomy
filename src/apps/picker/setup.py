from setuptools import setup, find_packages

setup(
    name='picker',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/picker']),
        ('share/picker', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Simple picker package',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_selector=picker.keyboard_selector:main',
            'keyboard_trigger=picker.keyboard_trigger:main',
            'picker_node=picker.picker_node:main',
            'medicine_detector=picker.medicine_detector:main',
        ],
    },
)
