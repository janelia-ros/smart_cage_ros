import os
from glob import glob
from setuptools import setup

package_name = 'smart_cage_data_writer'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Peter Polidoro',
    author_email='peterpolidoro@gmail.com',
    maintainer='Peter Polidoro',
    maintainer_email='peterpolidoro@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Smart cage data writer ROS interface.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_data_writer_node ='
            ' smart_cage_data_writer.image_data_writer_node:main',
            'lickport_data_writer_node ='
            ' smart_cage_data_writer.lickport_data_writer_node:main',
            'tunnel_data_writer_node ='
            ' smart_cage_data_writer.tunnel_data_writer_node:main',
        ],
    },
)
