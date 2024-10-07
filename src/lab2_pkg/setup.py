from setuptools import setup

package_name = 'lab2_pkg'

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
    maintainer='meloncaully',
    maintainer_email='maloneingham@gmail.com',
    description='Safety node for emergency braking',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = lab2_pkg.safety_node:main'
        ],
    },
)
