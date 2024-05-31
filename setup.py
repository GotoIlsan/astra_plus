from setuptools import find_packages, setup

package_name = 'astra_plus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JI HOON KIM',
    maintainer_email='jomes123@yonsei.ac.kr',
    description='Publish Depth Image, Color Image from Astra Plus',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astra_plus_node = astra_plus.astra_plus_node:main'
        ],
    },
)
