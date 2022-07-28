'''
Date: 2022-03-10 16:26:34
LastEditors: houhuixiang
'''
from setuptools import setup

package_name = 'movegogogo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='droid',
    maintainer_email='houhuixiang@cvte.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'movegogogo = movegogogo.movegogogo:main',
            'service = movegogogo.service:main',
        ],  
    },
)
